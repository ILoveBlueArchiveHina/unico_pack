#!/usr/bin/env python3
"""倉儲巡檢無人機的總指揮節點（node name: process_manager）。

整個系統的最上層狀態機，負責四件事：
  1. MQTT 與 ROS 2 之間的橋接（對外唯一窗口）
  2. 飛行流程編排（起飛 → 巡檢 → 返航 → 降落）
  3. Lifecycle 節點的喚醒與休眠（待機時省電、降低算力負載）
  4. rosbag 的錄製與完成後上傳 NAS

MQTT 介面
  訂閱 warehouse/task/request      巡檢任務（單筆或陣列），area 為四角點像素座標
       warehouse/task/notification 中止通知（"suspend"），通常不會用到
  發布 warehouse/task/status       每 5 秒回報 status 與電量
       warehouse/task/feedback     0=任務成立 1=rosbag上傳成功 2=導航失敗 3=上傳失敗
       warehouse/task/cancelled    中止時回報尚未完成的 task_id

正常任務流程
  1. 收到任務 → 座標轉換（僑泰任務圖與導航 pgm 相差 180 度，需鏡射後縮放）
     → 存入 task_queue，新任務排隊，不打斷進行中的任務
  2. 系統若在待機，依序喚醒 ZED → Livox → FAST-LIO → Nav2，任一失敗即放棄任務
  3. 若尚未起飛：確認 EKF 原點 → GUIDED → arm → takeoff → 輪詢確認離地
  4. 飛往安全區（走道，無障礙物）再調整高度至巡檢高度 1.5 m
  5. 發 NavTask 給 mission_dispatcher，收到 ready_to_record_rosbag 就開始錄製
  6. 收到 NavResult → 停止錄製 → 回報 MQTT → 取下一個任務
  7. 佇列清空 → 升到返航高度 3.0 m → 飛回 HOME → 精準降落 → 上鎖
     → 背景執行緒上傳 rosbag 至 NAS → 讓各 Lifecycle 節點休眠

異常處理
  電量低於 50%     清空佇列、取消當前任務、立刻返航
  起飛前置失敗     _abort_flight_sequence()，狀態轉 error，需人工排除
  Nav2 無回應      force_landing()，交由飛控 LAND 模式降落

流程幾乎全以 callback chain 串接（非阻塞），只有等待落地與上傳 rosbag
另開執行緒；MQTT 由 paho 自己的背景執行緒處理。
"""
import rclpy
import math
import threading
import time
import shutil
from rclpy.node import Node
import json
import paho.mqtt.client as mqtt
import os
from datetime import datetime
from rclpy.qos import qos_profile_sensor_data

from campus_delivery_msgs.msg import NavTask, NavResult
from geometry_msgs.msg import Pose2D
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header, Bool, Float64
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode, MessageInterval
from mavros_msgs.msg import ExtendedState, State 
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

""" 初始參數（常數）"""
# 全域座標
RETURN_HOME_ALT = 3.0
INSPECTION_ALT = 1.5
BASE_ALT = 2.0
SAFE_ZONE_X = 15.0
SAFE_ZONE_Y = -6.0

TAKEOFF_ALT_ODOM = 1.0  # 機體座標往上1.0公尺，不是絕對高度

LOW_BATTERY_THRESHOLD = 50                  # 電量低於這個值觸發返航充電行為
PROCESS_STATE_INSPECTION = "INSPECTION"     # 進程狀態：巡檢
PROCESS_STATE_RETURN_HOME = "RETURN_HOME"   # 進程狀態：返航

class ProcessManager(Node):
    def __init__(self):
        super().__init__('process_manager')  # 節點名稱

        # declare parameters (input values are for simulation; otherwise, default values will be used in actual test)
        # 宣告參數（若為 SITL 則使用外部輸入，否則使用預設值來實際測試）
        self.declare_parameter("home_pose_x", 2.0)
        self.declare_parameter("home_pose_y", -2.05)
        self.declare_parameter("rosbag_folder_path", "/home/uni-co-jetson/rosbag")
        self.declare_parameter("mqtt_broker", "192.168.166.83")
        self.declare_parameter("safe_zone_x", SAFE_ZONE_X)
        self.declare_parameter("safe_zone_y", SAFE_ZONE_Y)
        self.declare_parameter("nas_mount_path", "/mnt/data")
        
        self.home_pose_x = self.get_parameter("home_pose_x").value
        self.home_pose_y = self.get_parameter("home_pose_y").value
        self.rosbag_folder_path = self.get_parameter("rosbag_folder_path").value
        self.mqtt_broker = self.get_parameter("mqtt_broker").value
        self.safe_zone_x = self.get_parameter("safe_zone_x").value
        self.safe_zone_y = self.get_parameter("safe_zone_y").value
        self.use_sim_time = self.get_parameter("use_sim_time").value
        self.nas_mount_path = self.get_parameter("nas_mount_path").value

        # ROS2 topics (publisher)
        self.task_publisher_ = self.create_publisher(NavTask, "/navigation_tasks", 1)               # 發給 mission_dispatcher 的任務
        self.cancel_current_task_pub = self.create_publisher(Bool, "/cancel_navigation", 1)         # 告訴 mission_dispatcher 取消目前任務
        self.start_vel_bridge_signal_pub = self.create_publisher(Bool, '/start_vel_bridging', 1)    # 告訴 cmd_vel_bridge 可以開始橋接命令的訊號
        self.start_control_alt_pub = self.create_publisher(Bool, '/start_alt_control', 1)           # 告訴 velocity_controller 可以開始控制高度的訊號
        self.set_flight_altitude_pub = self.create_publisher(Float64, "/set_flight_altitude", 1)    # 告訴 velocity_controller 目標高度值(單位:公尺)
        self._gp_origin_pub = self.create_publisher(
            GeoPointStamped, '/mavros/global_position/set_gp_origin', 1)                            # 設置 mavros EKF原點位置

        # ROS2 topics (subscriber)
        self.create_subscription(NavResult, '/navigation_result', self.result_callback, 1)                          # 接收 mission_dispatcher 回傳的結果
        self.create_subscription(ExtendedState, '/mavros/extended_state', self.mavros_extended_state_callback, 1)   # 監聽 mavros 的擴展狀態訊息(為了監控飛機是否落地)
        self.create_subscription(State, '/mavros/state', self.mavros_state_callback, 1)                             # 監控 mavros 是否有連線
        self.create_subscription(Bool, '/ready_to_record_rosbag', self.ready_to_record_rosbag_signal_sub, 1)        # 接收 mission_dispather 到達巡檢區域後的訊號(表示可以錄製rosbag)
        self.create_subscription(Bool, '/set_flight_alt_done', self.set_flight_altitude_callback, 1)                # 接收 velocity_controller 回傳高度調整完成的訊號
        self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos_profile_sensor_data)   # 監控 mavros 回傳的飛機電量
        self.create_subscription(GeoPointStamped, '/mavros/global_position/gp_origin', self._gp_origin_callback, 1) # 監控 mavros 回傳的EKF原點訊息(用來判斷EKF原點是否設置成功)

        # ROS2 Lifecycle client (用來管理各功能的運轉與關閉)
        self.precision_landing_lifecycle = self.create_client(ChangeState, '/precision_landing_lifecycle/change_state')   # lifecycle節點名稱有誤導致切換狀態總是失敗(已修復2026-07-04)
        self.rosbag_lifecycle_client = self.create_client(ChangeState, '/rosbag_lifecycle/change_state')
        self.rosbag_lifecycle_param = self.create_client(SetParameters, '/rosbag_lifecycle/set_parameters')
        self.fast_lio_lifecycle = self.create_client(ChangeState, '/fastlio_wrapper/change_state')
        self.zed_lifecycle = self.create_client(ChangeState, '/zed_camera_lifecycle/change_state')
        self.livox_lifecycle = self.create_client(ChangeState, '/livox_lifecycle/change_state')

        # Nav2 Client
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')    # Nav2主要用來導航的功能
        self.nav2_lifecycle = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')    # 管理Nav2的運轉與待機

        # mavros client
        self.mavros_takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')                  # 起飛 service
        self.mavros_arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')                      # 解鎖/上鎖 service
        self.mavros_mode_client = self.create_client(SetMode, '/mavros/set_mode')                           # 設置模式 service
        self.mavros_message_client = self.create_client(MessageInterval, '/mavros/set_message_interval')    # 請求發送目標訊息 service

        # Intialize variables
        self.task_queue = []
        self.complite_task_list = []
        self.remaining_tasks = []
        self.rosbag_folder_name = "testing"

        # State variables
        self.battery_percentage = None
        self.ready_receive_mqtt = False
        self.current_task = None
        self.is_processing = False
        self.is_returning_home = False
        self.system_activated = False
        self.altitude_state = BASE_ALT
        self.current_status = 'offline'
        self.process_state = "NONE"
        
        # Flight controller state
        self.mavros_state = State()
        self.mavros_extended_state = ExtendedState()
        self._gp_origin_set = False
        self._takeoff_altitude = 1.0
        self._takeoff_poll_timer = None
        self._takeoff_poll_count = 0

        # MQTT topics
        self.task_mqtt_topic = "warehouse/task/request"
        self.notification_mqtt_topic = "warehouse/task/notification"
        self.cancelled_mqtt_topic = "warehouse/task/cancelled"
        self.status_mqtt_topic = "warehouse/task/status"
        self.feedback_mqtt_topic = "warehouse/task/feedback"

        # Timer
        self.create_timer(5.0, self._status_report)                     # 定期向MQTT回報狀態的計時器
        self.set_ready_timer_ = self.create_timer(3.0, self._set_ready) # MQTT剛連上，過一段時間等待穩定

        # MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        # Bound the built-in reconnect backoff (default is up to 120s) so a
        # dropped connection doesn't sit idle too long before retrying.
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)

        try:
            # connect_async() does not block or raise if the broker is
            # unreachable yet; loop_start()'s background thread keeps
            # retrying (with the backoff above) until it succeeds, which
            # also covers the "broker is down when this node starts" case.
            self.client.connect_async(self.mqtt_broker, 1883, 60)
            self.client.loop_start()
            self.get_logger().info("MQTT connection started (async). Bridge started.")
        except Exception as e:
            self.get_logger().error(f"Cannot start MQTT connection: {e}")

    def _low_battery_process(self):
        """ 當電量低時執行 """
        self.remaining_task_ids = self._get_remaining_tasks_list()
        self.task_queue.clear()
        self.record_rosbag("off")
        self._cancel_current_task()
        self._fly_to_safe_zone(PROCESS_STATE_RETURN_HOME)

    def _cancel_current_task(self):
        """發布取消導航的訊息給 mission_dispatcher"""
        msg = Bool()
        msg.data = True
        self.cancel_current_task_pub.publish(msg)

    def _get_remaining_tasks_list(self):
        """取得剩餘還沒完成的任務列表"""
        remaining_task_ids = [task.task_id for task in self.task_queue]
        if self.is_processing and self.current_task:
            remaining_task_ids.insert(0, self.current_task.task_id)

        return remaining_task_ids

    
    # -------------------------------------------------------------------#
    #  Subscription callbacks                                            #
    #--------------------------------------------------------------------#

    def mavros_extended_state_callback(self, msg):
        self.mavros_extended_state = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def _gp_origin_callback(self, msg):
        self._gp_origin_set = True

    def battery_callback(self, msg):
        # MAVROS reports percentage as 0.0~1.0; publish it as an integer 0~100.
        # An unknown level comes in as NaN, which must not reach round().
        # MAVROS 輸出電量為 0.0 ~ 1.0 的值，回報MQTT前需要轉換成 0~100
        if math.isnan(msg.percentage):
            self.battery_percentage = None
        else:
            self.battery_percentage = round(msg.percentage * 100)
    
    # ------------------------------------------------------------------ #
    #  State helpers                                                       #
    # ------------------------------------------------------------------ #

    def _is_in_air(self):
        return self.mavros_extended_state.landed_state == ExtendedState.LANDED_STATE_IN_AIR

    def _wait_for_landed(self, poll_interval=1.0):
        """ 以固定頻率檢查飛機是否落地，落地後才執行後續流程 """
        while self._is_in_air():
            time.sleep(poll_interval)

        self.get_logger().info("Detected landed, upload rosbag.")
        self.altitude_state = BASE_ALT
        # 落地後上鎖
        req = CommandBool.Request()
        req.value = False
        self.mavros_arm_client.call_async(req)

        bridge_signal_ = Bool()
        bridge_signal_.data = False
        self.start_vel_bridge_signal_pub.publish(bridge_signal_)

        self._lifecycle_transition(
            self.precision_landing_lifecycle, Transition.TRANSITION_DEACTIVATE, "PrecisionLanding")

        threading.Thread(                       # 開額外執行序，避免上傳過程卡住主程式
            target=self._upload_rosbag_to_nas,
            args=(self.complite_task_list,),
            daemon=True
        ).start()

        self.complite_task_list = []
        self.deactivate_system()    # 降落後只需要留必要功能，剩下的可以休眠
        
        
    def _require_extended_data(self, id):
        """ 向 mavros 請求發送訊息，某些訊息mavros是不會主動發布的"""
        if not self.mavros_message_client.service_is_ready():
            self.get_logger().error("set_message_interval service not available.")
            return
        req = MessageInterval.Request()
        req.message_id = id
        req.message_rate = 1.0
        self.mavros_message_client.call_async(req)

    def _set_ready(self):
        """ Filter out old messages remaining on MQTT. """
        """ 可能會有殘留訊息在MQTT Broker裡，設定計時器濾掉 """
        self.set_ready_timer_.destroy()
        if not self.ready_receive_mqtt:
            self.ready_receive_mqtt = True
            self.current_status = 'idle'
            self.get_logger().info("Manager is now ready to accept tasks.")

            threading.Thread(
                target=self.nas_mount_set,
                daemon=True
            ).start()
        
    def _abort_flight_sequence(self):
        """ 當執行到該function就代表飛機出現需要手動排除的故障"""
        self.get_logger().error("Aborting task.")
        self.send_cancelled_task_list(self._get_remaining_tasks_list())
        self.is_processing = True
        self.current_status = 'error'

    # ------------------------------------------------------------------ #
    #  MQTT                                                              #
    # ------------------------------------------------------------------ #

    def on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self.get_logger().error(f"MQTT connect failed (rc={rc}). Will keep retrying.")
            return
        client.subscribe(self.task_mqtt_topic)
        client.subscribe(self.notification_mqtt_topic)
        self.get_logger().info(
            f"Connected to MQTT Broker. Subscribed to: {self.task_mqtt_topic}, {self.notification_mqtt_topic}")

    def on_disconnect(self, client, userdata, rc):
        if rc != 0:
            self.get_logger().warn(f"MQTT disconnected unexpectedly (rc={rc}). Reconnecting...")
        else:
            self.get_logger().info("MQTT disconnected.")

    def on_message(self, client, userdata, msg):
        """ 接收 MQTT 訊息的地方 """
        if not self.ready_receive_mqtt:  # 過濾殘留訊息
            self.get_logger().warn(
                f"Ignoring message on {msg.topic} (not ready yet, may be retained message)")
            return

        try:
            data = json.loads(msg.payload.decode('utf-8'))
            if msg.topic == self.task_mqtt_topic:     # 若接收到的是任務topic, 進入下一步訊息處理
                self.handle_navigation_task(data)
            elif msg.topic == self.notification_mqtt_topic:  # 取消巡檢用的 MQTT Topic，可有可無
                self.handle_notification(data)
        except json.JSONDecodeError:
            self.get_logger().error("Received invalid JSON format")
        except Exception as e:
            self.get_logger().error(f"Error converting message: {e}")

    def _status_report(self):
        """ 定期回報巡檢狀態與電量 """
        payload = json.dumps({
            "status": self.current_status,
            "battery": self.battery_percentage
        })
        self.client.publish(self.status_mqtt_topic, payload)

        if self._is_in_air() and self.battery_percentage is not None:
            if self.battery_percentage < LOW_BATTERY_THRESHOLD:
                self._low_battery_process()

    def send_feedback(self, task_id, result, failed_faces):
        """ 任務成立/Rosbag上傳成功/導航失敗/Rosbag上傳失敗 等都需要回報MQTT """
        # send feedback to MQTT server
        try:
            payload = json.dumps({
                "task_id": task_id,
                "result": result,        # 0=任務成立,1=Rosbag上傳成功,2=導航失敗,3=Rosbag上傳失敗
                "failed_faces": failed_faces,
                "timestamp": datetime.now().isoformat()
            })
            self.client.publish(self.feedback_mqtt_topic, payload)
        except Exception as e:
            self.get_logger().error(f"Failed to publish feedback: {e}")

    def send_cancelled_task_list(self, cancelled_ids):
        """ 如果有還未做完的任務在此回報 """
        try:
            payload = json.dumps({
                "cancelled_task_ids": cancelled_ids,
                "timestamp": datetime.now().isoformat()
            })
            self.client.publish(self.cancelled_mqtt_topic, payload)
            self.get_logger().warn(f"Mission terminated. Cancelled: {payload}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish cancellation report: {e}")

    # ------------------------------------------------------------------ #
    #  Task ingestion & queue                                              #
    # ------------------------------------------------------------------ #

    def handle_navigation_task(self, data):
        """ 處理接收到的 MQTT 訊息，儲存成任務，如果軟體還在休眠就走啟動程序 """
        task_list = data if isinstance(data, list) else [data]

        for task_data in task_list:
            ros_msg = NavTask()
            ros_msg.task_id = task_data.get("task_id", "")
            ros_msg.rosbag_path = task_data.get("rosbag_path", "")
            ros_msg.source_timestamp = str(task_data.get("timestamp", ""))

            area_coords = task_data.get("area", [])
            for i in range(0, len(area_coords) - 1, 2):
                wp_msg = Pose2D()
                x_raw = float(area_coords[i])
                y_raw = float(area_coords[i + 1])

                # Coordinate transform (1900x1000 to 1482x728).
                # The pgm is rotated 180 degrees against the upstream drawing:
                # both put their origin top-left, but the drawing's top-left
                # corner is the pgm's bottom-right one. So mirror the point about
                # the centre of the drivable area (100,100)~(1800,900) — i.e.
                # measure x from 1800 and y from 900 — before scaling.
                # 導航地圖與僑泰任務地圖翻轉了180度，充電站在導航地圖左上角，僑泰任務地圖則為右下角
                # 去除兩張地圖的牆壁像素厚度後，根據解析度比例進行縮放（1900x1000 to 1482x728）
                wp_msg.x = round(((1800 - x_raw) * 0.866 + 4) * 0.05, 2)
                wp_msg.y = round(-((900 - y_raw) * 0.89125 + 10) * 0.05, 2)
                ros_msg.waypoints.append(wp_msg)

            self.task_queue.append(ros_msg)   # 新來的訊息會排隊等待，不會打斷前面正在執行的任務
            self.get_logger().info(
                f"Queued Task {ros_msg.task_id}. Queue size: {len(self.task_queue)}")

        if not self.is_processing:
            if self.system_activated:
                self.process_queue()
            else:
                self.activate_system()

    def process_queue(self):
        """ Dispatch next task; initiate takeoff sequence if drone is still on the ground."""
        """ 執行巡檢流程，如果還未起飛則初始化起飛程序"""
        if self.is_processing:
            return

        if not self.task_queue:
            if self._is_in_air():
                self.get_logger().info("Queue empty. Returning home.")
                self._fly_to_safe_zone(PROCESS_STATE_RETURN_HOME)
            return

        self.is_processing = True
        self.current_status = 'processing'
        self.current_task = self.task_queue.pop(0)
        self.rosbag_folder_name = self.current_task.rosbag_path
        self.is_returning_home = False

        if not self.altitude_state == INSPECTION_ALT:   
        # 如果目前高度不在巡檢高度的話，
        # 已經起飛則飛到調整安全區調整高度;
        # 還未起飛則執行起飛程序;
        # 已經在巡檢高度就直接執行下一個任務
            if self._is_in_air():
                self._fly_to_safe_zone(PROCESS_STATE_INSPECTION)
            else:
                self.get_logger().info("Drone on ground. Starting flight sequence before task.")
                self._flight_sequence()
        else:
            self._execute_task(self.current_task)

    # ------------------------------------------------------------------ #
    #  Flight sequence: set mode → arm → takeoff → poll altitude          #
    # ------------------------------------------------------------------ #

    def _flight_sequence(self):
        """Async chain: confirm EKF origin → GUIDED mode → arm → takeoff → wait for air → execute."""
        """ 起飛前流程： 
            1. 確定 MAVROS 有連到沒斷線，且必要訊息都有發送
            2. 檢查 EKF原點 是否有設置（沒設置的話不能用指令起飛），有的話就直接走起飛流程，否則繼續設置流程
            3. 發布 EKF原點Topic，並以固定頻率檢查是否成功，成功後進起飛流程

        """
        if not self.mavros_mode_client.service_is_ready() or not self.mavros_state.connected:
            self.get_logger().error("SetMode service unavailable. Aborting task.")
            self._abort_flight_sequence()
            return

        if self.mavros_extended_state.landed_state == ExtendedState.LANDED_STATE_UNDEFINED:
            self._require_extended_data(245) # 請求 extended_state 用來判斷是否起飛/落地
            self._require_extended_data(49)  # 請求 gp_origin 訊息用來判斷是否成功設置EKF原點

        if self._gp_origin_set:
            self._takeoff_sequence()
            return

        self.get_logger().info("EKF origin not confirmed yet. Setting and waiting for confirmation...")
        retry_count = 0

        def _poll_gp_origin():
            """ One-shot-style poll: retry set_gp_origin every 3 s, abort after 15 s total.
                以固定頻率檢查是否成功：self._gp_origin_set 為 true 代表成功
            """
            # `timer` is assigned below, before this can first fire at t+1 s.
            nonlocal retry_count
            retry_count += 1

            if self._gp_origin_set:
                timer.destroy()
                self.get_logger().info("EKF origin confirmed. Continuing flight sequence.")
                self._takeoff_sequence()
                return

            if retry_count >= 15:
                timer.destroy()
                self.get_logger().error("Timed out waiting for EKF origin confirmation. Aborting task.")
                self._abort_flight_sequence()
                return

            if retry_count % 3 == 0:
                self.get_logger().warn("EKF origin still not confirmed. Retrying set_gp_origin...")
                self._require_extended_data(49) # 請求 gp_origin 訊息
                self._set_ekf_origin()

        self._set_ekf_origin()
        timer = self.create_timer(1.0, _poll_gp_origin)

    def _takeoff_sequence(self):
        """ 起飛流程：
            1. 設定飛行模式為GUIDED
            2. 呼叫解鎖Service
            3. 設定起飛高度（機體座標）並發送起飛指令
            4. 以固定頻率檢查飛機是否離地，離地後飛往安全調整高度
            5. 若上述任何一項失敗則放棄任務並回報error
        """
        # 第一步：設置GUIDED模式
        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        future = self.mavros_mode_client.call_async(req)

        def _call_mode_response(future):    # 模式設置是否成功會在這裡收到
            try:
                if not future.result().mode_sent:
                    self.get_logger().error("SetMode GUIDED failed. Aborting task.")
                    self._abort_flight_sequence()
                    return
            except Exception as e:
                self.get_logger().error(f"SetMode error: {e}")
                self._abort_flight_sequence()
                return

            self.get_logger().info("Mode GUIDED. Arming drone...")

            if not self.mavros_arm_client.service_is_ready():
                self.get_logger().error("Arming service unavailable. Aborting task.")
                self._abort_flight_sequence()
                return

            # 下一步：解鎖
            req = CommandBool.Request()
            req.value = True
            future = self.mavros_arm_client.call_async(req)
            future.add_done_callback(_call_armed_response)

        def _call_armed_response(future):   # 解鎖設置是否成功會在這裡收到
            try:
                if not future.result().success:
                    self.get_logger().error("Arming failed. Aborting task.")
                    self._abort_flight_sequence()
                    return
            except Exception as e:
                self.get_logger().error(f"Arming error: {e}")
                self._abort_flight_sequence()
                return

            self.get_logger().info(f"Armed. Taking off to {self._takeoff_altitude} m...")

            if not self.mavros_takeoff_client.service_is_ready():
                self.get_logger().error("Takeoff service unavailable. Aborting task.")
                self._abort_flight_sequence()
                return

            # 下一步：起飛
            req = CommandTOL.Request()
            req.altitude = float(self._takeoff_altitude)
            req.min_pitch = 0.0
            req.yaw = 0.0
            req.latitude = 0.0
            req.longitude = 0.0
            future = self.mavros_takeoff_client.call_async(req)
            future.add_done_callback(_call_takeoff_response)

        def _call_takeoff_response(future): # 起飛是否成功會在這裡收到
            try:
                if not future.result().success:
                    self.get_logger().error("Takeoff command rejected. Aborting task.")
                    self._abort_flight_sequence()
                    return
            except Exception as e:
                self.get_logger().error(f"Takeoff error: {e}")
                self._abort_flight_sequence()
                return

            self.get_logger().info("Takeoff command accepted. Waiting for altitude...")
            self._takeoff_poll_count = 0
            self._takeoff_poll_timer = self.create_timer(1, _poll_altitude) # 定期檢查飛機是否已經離地

        def _poll_altitude():
            """One-shot poll: cancel timer once airborne or timed out (20 s)."""
            self._takeoff_poll_count += 1

            if self._is_in_air():
                self._takeoff_poll_timer.destroy()
                self.get_logger().info("Drone airborne. Executing task.")

                # 下一步：飛到安全區調高度
                self._fly_to_safe_zone(PROCESS_STATE_INSPECTION)

            elif self._takeoff_poll_count >= 20:
                self._takeoff_poll_timer.destroy()
                self.get_logger().error("Timed out waiting for takeoff. Aborting task.")
                self._abort_flight_sequence()

        future.add_done_callback(_call_mode_response)

    def _set_ekf_origin(self):
        """ 設置EKF原點，位置在高雄港附近 """
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.latitude  = 22.6
        msg.position.longitude = 120.288
        msg.position.altitude  = 0.0
        self._gp_origin_pub.publish(msg)
        self.get_logger().info("EKF global origin set.")

    def _fly_to_safe_zone(self, process_state):
        """ 因光達上下皆有死角，因此調整高度前先飛往安全區，
            安全區定義為不會放貨物的走道
        """
        self.is_processing = True
        self.process_state = process_state

        # 發送速度命令橋接訊號，告訴橋接節點可以開始橋接
        # 必須在起飛完成後才能橋接速度命令，否則起飛動作將被打斷
        # 導致無法起飛
        bridge_signal_ = Bool()
        bridge_signal_.data = True
        self.start_vel_bridge_signal_pub.publish(bridge_signal_)

        # 設定目標點位置，發送給Nav2導航並等待結果
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.safe_zone_x
        goal_msg.pose.pose.position.y = self.safe_zone_y
        goal_msg.pose.pose.orientation.w = 1.0
        future = self.nav_to_pose_client.send_goal_async(goal_msg)

        def _fly_to_safe_zone_callback(f):
            goal_handle = f.result()
            if not goal_handle.accepted:
                self.get_logger().error("fly to safe-zone rejected!")
                self.is_returning_home = False
                self._cancel_current_task()
                self.return_home()
                return

            goal_handle.get_result_async().add_done_callback(_fly_to_safe_zone_done)

        def _fly_to_safe_zone_done(f):
            status = f.result().status
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(f"fly to safe-zone failed (status={status}). Force landing.")
                self.force_landing()
                return

            self.get_logger().info("Arrived at safe zone.")
            self._set_flight_altitude(process_state)

            return

        future.add_done_callback(_fly_to_safe_zone_callback)

    def _set_flight_altitude(self, process_state):
        """ 調整高度：負責節點為 velocity_controller 
            判斷現在是什麼程序階段，調整至對應高度(巡檢or返航)
            使用 topic 發布開始控制高度的訊號和目標高度值
            並等待velocity_controller回傳高度調整完成的topic
        """
        self.process_state = process_state
        msg = Float64()
        if process_state == PROCESS_STATE_INSPECTION:
            self.get_logger().info(f'Set alt to {INSPECTION_ALT} m')
            msg.data = INSPECTION_ALT

        elif process_state == PROCESS_STATE_RETURN_HOME:
            self.get_logger().info(f'Set alt to {RETURN_HOME_ALT} m')
            msg.data = RETURN_HOME_ALT
        
        signal_msg = Bool()
        signal_msg.data = True
        self.start_control_alt_pub.publish(signal_msg)
        self.set_flight_altitude_pub.publish(msg)

    def set_flight_altitude_callback(self, msg):
        """ 收到 velocity_controller回傳高度調整完成，
            根據目前程序階段決定下一步做什麼
        """
        if msg.data:
            if self.process_state == PROCESS_STATE_INSPECTION:
                self.altitude_state = INSPECTION_ALT
                self._execute_task(self.current_task)  # 執行任務
                return

            elif self.process_state == PROCESS_STATE_RETURN_HOME:
                self.altitude_state = RETURN_HOME_ALT
                self.return_home()  # 回家

    # ------------------------------------------------------------------ #
    #  Task execution & result                                           #
    # ------------------------------------------------------------------ #

    def _execute_task(self, task):
        """ 將任務topic發布至節點 mission_dispatcher 並等待其回傳結果 """
        self.task_publisher_.publish(task)
        self.get_logger().info(
            f"Executing Task {task.task_id} (Remaining in queue: {len(self.task_queue)})")
        self.send_feedback(task_id=task.task_id, result=0, failed_faces=[])     # 回報已接受任務

    def ready_to_record_rosbag_signal_sub(self, msg):
        """ 當 mission_dispatcher 成功到達任務點時會發布錄製rosbag訊號
            接收到該訊號後開始錄製rosbag
        """
        if msg.data:    # 如果訊號是true代表可以開始錄製rosbag
            self.record_rosbag("on", path=self.rosbag_folder_name)
            self.get_logger().info("Starting rosbag recording")

    def result_callback(self, msg):
        """ 接收 mission_dispatcher 回傳的結果並回報 MQTT，
            關閉rosbag錄製，接著執行下一個任務
        """
        self.get_logger().info(f"Received Result for Task {msg.task_id}: {msg.result}")

        if msg.result == 2:
            self.send_feedback(
                task_id=msg.task_id,
                result=msg.result,
                failed_faces=list(msg.failed_faces) if msg.failed_faces is not None else []
            )

        self.record_rosbag("off")

        if msg.result in (1, 2):
            self.is_processing = False
            self.complite_task_list.append(self.current_task)  # 紀錄有完成的任務，以便後面上傳rosbag
            self.process_queue()

    # ------------------------------------------------------------------ #
    #  Return to Home                                                      #
    # ------------------------------------------------------------------ #
    def return_home(self):
        """ 結束任務或任務取消時執行返航程序，監控飛機是否成功落地 """
        if self.is_returning_home:
            return

        self.is_returning_home = True
        self.is_processing = True

        if not self._is_in_air():
            self.get_logger().info("Drone already on ground. Skipping RTH.")
            return

        # 開另一個執行序監控飛機是否已經落地
        threading.Thread(
            target=self._wait_for_landed,
            daemon=True
        ).start()

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):  # 如果 Nav2 沒回應就迫降
            self.get_logger().error("Nav2 Action Server unavailable. Cannot return home. START FORCE LANDING")
            self.force_landing()
            return

        self.get_logger().info("Initiating Return to Home...")
        
        # 呼叫Nav2 設定HOME點並等待導航結果，返回HOME點失敗就迫降
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.home_pose_x
        goal_msg.pose.pose.position.y = self.home_pose_y
        goal_msg.pose.pose.orientation.w = 1.0

        future = self.nav_to_pose_client.send_goal_async(goal_msg)

        def _home_response_callback(f):
            goal_handle = f.result()
            
            if not goal_handle.accepted:
                self.get_logger().error("Return to Home rejected!")
                self.is_returning_home = False
                self._cancel_current_task()
                self.force_landing()
                return

            self.get_logger().info("Return to Home accepted.")
            goal_handle.get_result_async().add_done_callback(_home_result_callback)

        def _home_result_callback(f):   # 到達HOME點上方（降落台）後執行精準降落
            self.get_logger().info("Arrived at Home. Starting precision landing...")
            signal_msg = Bool()
            signal_msg.data = False
            self.start_control_alt_pub.publish(signal_msg)
            self.is_returning_home = False

            # 如果精準降落沒回應就執行迫降
            self._lifecycle_transition(
                self.precision_landing_lifecycle, Transition.TRANSITION_ACTIVATE, "PrecisionLanding",
                done_cb=lambda success: None if success else self.force_landing())

        future.add_done_callback(_home_response_callback)

    def force_landing(self):
        """ 強制降落用，無法正常降落之情況會使用到
            主要利用飛控的LAND模式進行降落
            降落動作完全交由飛控的安全機制處理
        """
        if not self._is_in_air():
            self.get_logger().info("Drone already on ground.")
            return

        if not self.mavros_mode_client.service_is_ready():  # 如果連 LAND 模式都不能設定，直接設定目標高度為地面
            self.get_logger().error("SetMode service unavailable.")
            signal_msg = Bool()
            signal_msg.data = True
            self.start_control_alt_pub.publish(signal_msg)
            msg = Float64()
            msg.data = -0.2  # 為了確實降落到地面
            self.set_flight_altitude_pub.publish(msg)
            return

        req = SetMode.Request()
        req.custom_mode = 'LAND'
        future = self.mavros_mode_client.call_async(req)
        self.current_status = 'error'
        
        def _on_done(f):
            try:
                if not f.result().mode_sent:
                    self.get_logger().error("Landing failed.")
            except Exception as e:
                self.get_logger().error(f"Landing error: {e}")

        future.add_done_callback(_on_done)

    # ------------------------------------------------------------------ #
    #  Notifications & termination                                         #
    # ------------------------------------------------------------------ #
    """ MQTT通知中止任務，通常不會用到 """
    def handle_notification(self, data):
        notification = data.get("notification", "")
        self.get_logger().info(f"Received Notification: {notification}")

        if notification == "suspend":
            self.handle_termination()

    def handle_termination(self):
        """Clear queue, report cancelled tasks, return home."""
        self.record_rosbag("off")

        cancelled_ids = self._get_remaining_tasks_list()

        self.task_queue.clear()
        self._cancel_current_task()

        self.send_cancelled_task_list(cancelled_ids)
        self._fly_to_safe_zone(PROCESS_STATE_RETURN_HOME)

    # ------------------------------------------------------------------ #
    #  Subprocesses                                                      #
    # ------------------------------------------------------------------ #

    def _lifecycle_transition(self, client, transition_id, label, done_cb=None):
        """ 用來啟動單一 Lifecycle 節點，是否啟動成功會由done_cb回傳結果 """
        if not client.service_is_ready():
            self.get_logger().error(f"{label} change_state service not available.")
            if done_cb:
                done_cb(False)
            return
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)

        def _on_done(f):
            result = f.result()
            success = result is not None and result.success
            if success:
                self.get_logger().info(f"{label} transition OK.")
            else:
                self.get_logger().error(f"{label} transition FAILED.")
            if done_cb:
                done_cb(success)

        future.add_done_callback(_on_done)

    def _nav2_lifecycle(self, command, label, done_cb=None):
        """ 用來管理 Nav2 運轉與待機 """
        if not self.nav2_lifecycle.service_is_ready():
            self.get_logger().error(f"{label} manage_nodes service not available.")
            if done_cb:
                done_cb(False)
            return
        req = ManageLifecycleNodes.Request()
        req.command = command
        future = self.nav2_lifecycle.call_async(req)

        def _on_done(f):
            result = f.result()
            success = result is not None and result.success
            if success:
                self.get_logger().info(f"{label} transition OK.")
            else:
                self.get_logger().error(f"{label} transition FAILED.")
            if done_cb:
                done_cb(success)

        future.add_done_callback(_on_done)

    def record_rosbag(self, action="on", path="default"):
        """ 用來錄製 rosbag 和設定檔案路徑 """
        transition_id = {
            "on":  Transition.TRANSITION_ACTIVATE,
            "off": Transition.TRANSITION_DEACTIVATE,
        }.get(action)
        if transition_id is None:
            self.get_logger().warn(f"Unknown action for record_rosbag: {action}")
            return

        if action == "on":
            now = datetime.now()
            hm = now.strftime("%H-%M")
            date_str, task_id = path.split("/")

            # 實際錄製的相對路徑：YYYY-MM-DD/YYYY-MM-DD-HH-MM_TaskID
            rel_dir = f"{date_str}/{date_str}-{hm}_{task_id}"
            output_path = f"{self.rosbag_folder_path}/{rel_dir}/sampling"

            # 把實際路徑寫回 task，讓之後上傳知道要抓哪個資料夾
            if self.current_task is not None:
                self.current_task.rosbag_path = rel_dir
            # if self.use_sim_time:
            #     self.get_logger().error(f'Starting recording ROS bag (simulation), Task: {self.current_task.task_id}, path:{output_path}')
            #     return
            if not self.rosbag_lifecycle_param.service_is_ready():
                self.get_logger().error("rosbag_node set_parameters service not available.")
                return

            # 把 rosbag 儲存路徑用參數寫入 rosbag_lifecycle
            param = Parameter()
            param.name = "output_path"
            param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=output_path)
            req = SetParameters.Request()
            req.parameters = [param]
            future = self.rosbag_lifecycle_param.call_async(req)
            future.add_done_callback(
                lambda f: self._lifecycle_transition(
                    self.rosbag_lifecycle_client, Transition.TRANSITION_ACTIVATE, f"Rosbag({path})")
            )
        else:
            self.get_logger().error(f'Stop recording ROS bag')
            self._lifecycle_transition(self.rosbag_lifecycle_client, transition_id, f"Rosbag({path})")

    def _upload_rosbag_to_nas(self, complite_task_list):
        """ 將已經完成的任務的 rosbag 上傳到 NAS """
        for task in complite_task_list:
            rel_dir = task.rosbag_path        # YYYY-MM-DD/YYYY-MM-DD-HH-MM_TaskID
            src = f"{self.rosbag_folder_path}/{rel_dir}"
            dst = f"{self.nas_mount_path}/{rel_dir}"

            # 先確認 NAS 有掛載
            if not self.use_sim_time:
                if not self.nas_mount_set():
                    self.get_logger().error("NAS not mounted, skip upload.")
                    return

            try:
                # if not self.use_sim_time:
                #     os.makedirs(os.path.dirname(dst), exist_ok=True)
                #     shutil.copytree(src, dst)
                
                os.makedirs(os.path.dirname(dst), exist_ok=True)    # 先在掛載點(/mnt/data)建一個資料夾 YYYY-MM-DD ，如果已經存在就跳過
                shutil.copytree(src, dst, dirs_exist_ok=True)       # 將檔案複製到掛載點，等於上傳到NAS上
                # 複製完成才會往下走
                self.get_logger().info(f"Rosbag uploaded: {task.task_id}")
                self.send_feedback(task_id=task.task_id, result=1, failed_faces=[])
            except Exception as e:  # 複製過程若因錯誤碼而中斷一律當作上傳失敗
                self.get_logger().error(f"Rosbag upload failed: {e}")
                self.send_feedback(task_id=task.task_id, result=3, failed_faces=[])

        self.is_processing = False

    def nas_mount_set(self, retries=3, delay=2.0):
        # /mnt/data 在 /etc/fstab 設為 x-systemd.automount：
        # 必須「存取掛載點內部」才會觸發 systemd 自動掛 NFS（不需要 sudo/密碼）。
        # os.path.ismount() 只看 autofs 佔位層，無法確認 NFS 真的掛好，故改用實際存取驗證。
        for _ in range(retries):
            try:
                os.listdir(self.nas_mount_path)          # 觸發 automount 並確認可讀
                if self._is_nfs_mounted(self.nas_mount_path):
                    self.get_logger().info(f"NAS mounted at {self.nas_mount_path}.")
                    return True
            except OSError as e:
                self.get_logger().warn(f"NAS not ready yet: {e}")
            time.sleep(delay)
        self.get_logger().error(f"NAS still not mounted at {self.nas_mount_path} after {retries} retries.")
        return False

    def _is_nfs_mounted(self, path):
        real = os.path.realpath(path)
        with open("/proc/mounts") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 3 and parts[1] == real and parts[2].startswith("nfs"):
                    return True
        return False

    def _delay_call(self, delay, callback):
        """ 延遲觸發用的function """
        def _activate():
            timer.destroy()
            callback()
        timer = self.create_timer(delay, _activate)

    def activate_system(self):
        """ 將巡檢需要用到的功能(Lifecycle節點)從待機狀態喚醒，
            利用 self._lifecycle_transition 的 done_cb 可以判斷是否成功，
            一步一步將各個功能喚醒，若失敗則放棄任務，等待故障排除。
        """
        delay = 3.0

        def _nav2_activated_callback(success):
            if success:
                self.system_activated = True
                self._delay_call(delay, self.process_queue)
            else:
                self.system_activated = False
                self._abort_flight_sequence()

        def _activate_nav2():
            self._nav2_lifecycle(
                ManageLifecycleNodes.Request.STARTUP, "Nav2",
                done_cb=_nav2_activated_callback
            )
        
        def _activate_fastlio():
            self._lifecycle_transition(
                self.fast_lio_lifecycle, Transition.TRANSITION_ACTIVATE, "Fast-lio",
                done_cb=lambda success: self._delay_call(delay, _activate_nav2) if success else self._abort_flight_sequence()
            )

        def _activate_livox():
            self._lifecycle_transition(
                self.livox_lifecycle, Transition.TRANSITION_ACTIVATE, "Livox-driver",
                done_cb=lambda success: self._delay_call(delay, _activate_fastlio) if success else self._abort_flight_sequence()
            )

        def _activate_zed():
            self._lifecycle_transition(
                self.zed_lifecycle, Transition.TRANSITION_ACTIVATE, "ZED-Wrapper",
                done_cb=lambda success: self._delay_call(delay, _activate_livox) if success else self._abort_flight_sequence()
            )

        if self.use_sim_time:
            _activate_nav2()
            return

        _activate_nav2()    # 一般測試的話則只要喚醒nav2就好，因為其他功能開啟時就在運作，
        # _activate_zed()   # 如果是正式測試流程的話請先喚醒zed，
                            # 因為會使用master.launch.py開啟所有節點，
                            # 此時各節點都在休眠，因此會需要將所有節點喚醒，
                            # 而一般測試時大部分功能開啟時就已經在運作，
                            # 因此只需要喚醒 Nav2就好。
        

    def deactivate_system(self):
        """ 將待機需要休眠的功能(Lifecycle節點)從運轉狀態切換成待機 """
        delay = 2.0
        self.system_activated = False
        
        def _deactivate_zed():
            self._lifecycle_transition(
                self.zed_lifecycle, Transition.TRANSITION_DEACTIVATE, "ZED-Wrapper")
        
        def _deactivate_livox():
            self._lifecycle_transition(
                self.livox_lifecycle, Transition.TRANSITION_DEACTIVATE, "Livox-driver",
                done_cb=lambda success: self._delay_call(delay, _deactivate_zed)
            )

        def _deactivate_fastlio():
            self._lifecycle_transition(
                self.fast_lio_lifecycle, Transition.TRANSITION_DEACTIVATE, "Fast-lio",
                done_cb=lambda success: self._delay_call(delay, _deactivate_livox)
            )
        
        def _deactivate_nav2():
            self._nav2_lifecycle(ManageLifecycleNodes.Request.RESET, "Nav2",
            done_cb=lambda success: self._delay_call(delay, _deactivate_fastlio)
            )

        if self.use_sim_time:
            self._nav2_lifecycle(ManageLifecycleNodes.Request.RESET, "Nav2")
            return
        
        self._nav2_lifecycle(ManageLifecycleNodes.Request.RESET, "Nav2")    # 一般測試時使用這行
        # _deactivate_nav2()    # 正式流程測時則使用這行

        


def main(args=None):
    rclpy.init(args=args)
    node = ProcessManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.loop_stop()
        node.client.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
