#!/usr/bin/env python3
import rclpy
import threading
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
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from mavros_msgs.msg import ExtendedState, State 
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


LOW_BATTERY_THRESHOLD = 0.5
RETURN_HOME_ALT = 3.0
INSPECTION_ALT = 1.5
SAFE_ZONE_X = 15.0
SAFE_ZONE_Y = -6.0
PROCESS_STATE_INSPECTION = "INSPECTION"
PROCESS_STATE_RETURN_HOME = "RETURN_HOME"

class ManagementNode(Node):
    def __init__(self):
        super().__init__('process_manager')

        # declare parameters (input values are for simulation; otherwise, default values will be used in actual test)
        self.declare_parameter("home_pose_x", 2.0)
        self.declare_parameter("home_pose_y", -2.05)
        self.declare_parameter("rosbag_folder_path", "/home/uni-co-jetson/rosbag")
        self.declare_parameter("mqtt_broker", "192.168.166.83")
        self.declare_parameter("safe_zone_x", SAFE_ZONE_X)
        self.declare_parameter("safe_zone_y", SAFE_ZONE_Y)
        
        self.home_pose_x = self.get_parameter("home_pose_x").value
        self.home_pose_y = self.get_parameter("home_pose_y").value
        self.rosbag_folder_path = self.get_parameter("rosbag_folder_path").value
        self.mqtt_broker = self.get_parameter("mqtt_broker").value
        self.safe_zone_x = self.get_parameter("safe_zone_x").value
        self.safe_zone_y = self.get_parameter("safe_zone_y").value
        self.use_sim_time = self.get_parameter("use_sim_time").value

        # ROS2 topics (publisher)
        self.task_publisher_ = self.create_publisher(NavTask, "navigation_tasks", 1)
        self.start_vel_bridge_signal_pub = self.create_publisher(Bool, 'start_vel_bridging', 1)
        self.cancel_current_task_pub = self.create_publisher(Bool, "cancel_navigation", 1)
        self.set_flight_altitude_pub = self.create_publisher(Float64, "set_flight_altitude", 1)
        self._gp_origin_pub = self.create_publisher(
            GeoPointStamped, '/mavros/global_position/set_gp_origin', 1)  # Setting EKF origin
        

        # ROS2 topics (subscriber)
        self.create_subscription(NavResult, 'navigation_result', self.result_callback, 1)
        self.create_subscription(ExtendedState, '/mavros/extended_state', self.mavros_extended_state_callback, 1)
        self.create_subscription(State, '/mavros/state', self.mavros_state_callback, 1)
        self.create_subscription(Bool, 'is_landed', self.is_landed_subscribe, 1)
        self.create_subscription(Bool, 'ready_to_record_rosbag', self.ready_to_record_rosbag_signal_sub, 1)
        self.create_subscription(Bool, 'set_flight_alt_done', self.set_flight_altitude_callback, 1)
        self.create_subscription(BatteryState, '/mavros/battery', self.battery_callback, qos_profile_sensor_data)
        

        # ROS2 client
        self._pl_change_state_cli = self.create_client(ChangeState, '/precision_landing_node/change_state')
        self._bag_change_state_cli = self.create_client(ChangeState, '/rosbag_node/change_state')
        self._bag_set_params_cli = self.create_client(SetParameters, '/rosbag_node/set_parameters')

        self.mavros_takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.mavros_arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mavros_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        

        # Timer
        self.create_timer(5.0, self._status_report)
        self.set_ready_timer_ = self.create_timer(5.0, self._set_ready)

        # Intialize variables
        self.task_queue = []
        self.complite_task_path = []
        self.remaining_tasks = []
        self.current_task = None
        self.is_processing = False
        self.is_returning_home = False
        self.landed = False
        self.landing_process = None
        self.rosbag_process = None
        self.rosbag_folder_name = "testing"
        self.battery_percentage = None
        self.ready_receive_mqtt = False
        self.mavros_state = State()
        self.mavros_extended_state = ExtendedState()
        self.current_status = 'offline'
        self.process_state = "NONE"
        self._takeoff_altitude = 1.0
        self._takeoff_poll_timer = None
        self._takeoff_poll_count = 0

        # MQTT topics
        self.task_mqtt_topic = "warehouse/task/request"
        self.notification_mqtt_topic = "warehouse/task/notification"
        self.cancelled_mqtt_topic = "warehouse/task/cancelled"
        self.status_mqtt_topic = "warehouse/task/status"
        self.feedback_mqtt_topic = "warehouse/task/feedback"

        # MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # NAS server
        self.nas_mount_path = "/mnt/data"

        try:
            self.client.connect(self.mqtt_broker, 1883, 60)
            self.client.loop_start()
            self.get_logger().info("Connected to MQTT Broker. Bridge started.")
        except Exception as e:
            self.get_logger().error(f"Cannot connect to MQTT: {e}")


    def _low_battery_process(self):
        """ 當電量低時執行 """
        self.remaining_task_ids = self._get_remaining_tasks_list()
        self.task_queue.clear()
        self.is_processing = False
        self.record_rosbag("off")
        self._cancel_current_task()
        self._fly_to_safe_zone(PROCESS_STATE_RETURN_HOME)

    def _cancel_current_task(self):
        msg = Bool()
        msg.data = True
        self.cancel_current_task_pub.publish(msg)

    def _get_remaining_tasks_list(self):
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

    def battery_callback(self, msg):
        self.battery_percentage = msg.percentage

    def is_landed_subscribe(self, msg):
        if msg.data:
            self.precision_landing("off")
    
    # ------------------------------------------------------------------ #
    #  State helpers                                                       #
    # ------------------------------------------------------------------ #

    def _is_in_air(self):
        return self.mavros_extended_state.landed_state == ExtendedState.LANDED_STATE_IN_AIR

    def _current_flight_mode(self):
        return self.mavros_state.mode

    def _set_ready(self):
        """ Filter out old messages remaining on MQTT. """
        self.set_ready_timer_.destroy()
        if not self.ready_receive_mqtt:
            self._set_ekf_origin()
            self.ready_receive_mqtt = True
            self.current_status = 'idle'
            self.get_logger().info("Manager is now ready to accept tasks.")

    # ------------------------------------------------------------------ #
    #  MQTT                                                              #
    # ------------------------------------------------------------------ #

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(self.task_mqtt_topic)
        client.subscribe(self.notification_mqtt_topic)
        self.get_logger().info(
            f"Subscribed to MQTT topics: {self.task_mqtt_topic}, {self.notification_mqtt_topic}")

    def on_message(self, client, userdata, msg):
        if not self.ready_receive_mqtt:
            self.get_logger().warn(
                f"Ignoring message on {msg.topic} (not ready yet, may be retained message)")
            return

        try:
            data = json.loads(msg.payload.decode('utf-8'))
            if msg.topic == self.task_mqtt_topic:
                self.handle_navigation_task(data)
            elif msg.topic == self.notification_mqtt_topic:
                self.handle_notification(data)
        except json.JSONDecodeError:
            self.get_logger().error("Received invalid JSON format")
        except Exception as e:
            self.get_logger().error(f"Error converting message: {e}")

    # ------------------------------------------------------------------ #
    #  Task ingestion & queue                                              #
    # ------------------------------------------------------------------ #

    def handle_navigation_task(self, data):
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

                # Coordinate transform (1900x1000 to 1482x728)
                wp_msg.x = round(((x_raw - 100) * 0.866 + 4) * 0.05, 2)
                wp_msg.y = round(-((y_raw - 100) * 0.89125 + 10) * 0.05, 2)
                ros_msg.waypoints.append(wp_msg)

            self.task_queue.append(ros_msg)
            self.get_logger().info(
                f"Queued Task {ros_msg.task_id}. Queue size: {len(self.task_queue)}")

        self.process_queue()

    def process_queue(self):
        """Dispatch next task; initiate takeoff sequence if drone is still on the ground."""
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

        if self._is_in_air():
            self._fly_to_safe_zone(PROCESS_STATE_INSPECTION)
        else:
            self.get_logger().info("Drone on ground. Starting flight sequence before task.")
            self._flight_sequence()

    # ------------------------------------------------------------------ #
    #  Flight sequence: set mode → arm → takeoff → poll altitude          #
    # ------------------------------------------------------------------ #

    def _flight_sequence(self):
        """Async chain: GUIDED mode → arm → takeoff → wait for air → execute."""
        if not self.mavros_mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("SetMode service unavailable. Aborting task.")
            self._abort_flight_sequence()
            return

        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        future = self.mavros_mode_client.call_async(req)

        def _call_mode_response(future):
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

            if not self.mavros_arm_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error("Arming service unavailable. Aborting task.")
                self._abort_flight_sequence()
                return

            req = CommandBool.Request()
            req.value = True
            future = self.mavros_arm_client.call_async(req)
            future.add_done_callback(_call_armed_response)

        def _call_armed_response(future):
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

            if not self.mavros_takeoff_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().error("Takeoff service unavailable. Aborting task.")
                self._abort_flight_sequence()
                return

            req = CommandTOL.Request()
            req.altitude = float(self._takeoff_altitude)
            req.min_pitch = 0.0
            req.yaw = 0.0
            req.latitude = 0.0
            req.longitude = 0.0
            future = self.mavros_takeoff_client.call_async(req)
            future.add_done_callback(_call_takeoff_response)

        def _call_takeoff_response(future):
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
            self._takeoff_poll_timer = self.create_timer(1, _poll_altitude)

        def _poll_altitude():
            """One-shot poll: cancel timer once airborne or timed out (10 s)."""
            self._takeoff_poll_count += 1

            if self._is_in_air():
                self._takeoff_poll_timer.destroy()
                self.get_logger().info("Drone airborne. Executing task.")
                self._fly_to_safe_zone(PROCESS_STATE_INSPECTION)
            elif self._takeoff_poll_count >= 20:
                self._takeoff_poll_timer.destroy()
                self.get_logger().error("Timed out waiting for takeoff. Aborting task.")
                self._abort_flight_sequence()

        future.add_done_callback(_call_mode_response)

    def _abort_flight_sequence(self):
        self.is_processing = False
        self.current_status = 'error'

    def _set_ekf_origin(self):
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.latitude  = 22.6
        msg.position.longitude = 120.288
        msg.position.altitude  = 0.0
        self._gp_origin_pub.publish(msg)
        self.get_logger().info("EKF global origin set.")

    # ------------------------------------------------------------------ #
    #  Task execution & result                                           #
    # ------------------------------------------------------------------ #

    def _execute_task(self, task):
        """Publish task to mission_dispatcher."""
        self.task_publisher_.publish(task)
        self.get_logger().info(
            f"Executing Task {task.task_id} (Remaining in queue: {len(self.task_queue)})")
        self.send_feedback(task_id=task.task_id, result=0, failed_faces=[])

    def result_callback(self, msg):
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
            self.complite_task_path.append(self.current_task.rosbag_path)
            self.process_queue()

    # ------------------------------------------------------------------ #
    #  Sensors & status                                                    #
    # ------------------------------------------------------------------ #

    def ready_to_record_rosbag_signal_sub(self, msg):
        if msg.data:
            self.record_rosbag("on", path=self.rosbag_folder_name)
            self.get_logger().info("Starting rosbag recording")

    def _status_report(self):
        payload = json.dumps({
            "status": self.current_status,
            "battery": self.battery_percentage
        })
        self.client.publish(self.status_mqtt_topic, payload)

        if self._is_in_air() and self.battery_percentage is not None:
            if self.battery_percentage < LOW_BATTERY_THRESHOLD:
                self._low_battery_process()

    def send_feedback(self, task_id, result, failed_faces):
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
    #  Return to Home                                                      #
    # ------------------------------------------------------------------ #
    # todo: 用進程狀態參數來決定飛到安全區後要回家還是巡檢
    def _fly_to_safe_zone(self, process_state):
        self.process_state = process_state
        bridge_signal_ = Bool()
        bridge_signal_.data = True
        self.start_vel_bridge_signal_pub.publish(bridge_signal_)
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
                self.is_processing = False
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
        self.process_state = process_state
        msg = Float64()
        if process_state == PROCESS_STATE_INSPECTION:
            self.get_logger().info(f'Set alt to {INSPECTION_ALT} m')
            msg.data = INSPECTION_ALT

        elif process_state == PROCESS_STATE_RETURN_HOME:
            self.get_logger().info(f'Set alt to {RETURN_HOME_ALT} m')
            msg.data = RETURN_HOME_ALT
        
        self.set_flight_altitude_pub.publish(msg)

    def set_flight_altitude_callback(self, msg):
        if msg.data:
            if self.process_state == PROCESS_STATE_INSPECTION:
                # todo: execute inspection process
                self._execute_task(self.current_task)
                return

            elif self.process_state == PROCESS_STATE_RETURN_HOME:
                self.return_home()

    def return_home(self):
        if self.is_returning_home:
            return

        self.is_returning_home = True
        if not self._is_in_air():
            self.get_logger().info("Drone already on ground. Skipping RTH.")
            
            return

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 Action Server unavailable. Cannot return home. START FORCE LANDING")
            self.force_landing()
            return

        self.get_logger().info("Initiating Return to Home...")
        self.is_processing = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.home_pose_x
        goal_msg.pose.pose.position.y = self.home_pose_y
        goal_msg.pose.pose.orientation.w = 1.0

        future = self.nav_to_pose_client.send_goal_async(goal_msg)

        def _home_response_callback(f):
            goal_handle = f.result()
            threading.Thread(
                target=self._upload_rosbag_to_nas,
                args=(self.complite_task_path,),
                daemon=True
            ).start()
            self.complite_task_path = []

            if not goal_handle.accepted:
                self.get_logger().error("Return to Home rejected!")
                self.is_processing = False
                self.is_returning_home = False
                self._cancel_current_task()
                self.force_landing()
                return

            self.get_logger().info("Return to Home accepted.")
            goal_handle.get_result_async().add_done_callback(_home_result_callback)

        def _home_result_callback(f):
            self.get_logger().info("Arrived at Home. Starting precision landing...")
            self.precision_landing("on")
            self.is_processing = False
            self.is_returning_home = False

        future.add_done_callback(_home_response_callback)

    def force_landing(self):
        if not self._is_in_air():
            self.get_logger().info("Drone already on ground.")
            return

        if not self.mavros_mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("SetMode service unavailable.")
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
        self.is_processing = False
        # Note: mission_dispatcher's active Nav2 goal is NOT cancelled here.
        # A future improvement is to publish a cancel signal to mission_dispatcher.

        self.send_cancelled_task_list(cancelled_ids)
        self._fly_to_safe_zone(PROCESS_STATE_RETURN_HOME)

    # ------------------------------------------------------------------ #
    #  Subprocesses                                                      #
    # ------------------------------------------------------------------ #

    def _lifecycle_transition(self, client, transition_id, label, done_cb=None):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"{label} change_state service not available.")
            if label == "PrecisionLanding":
                self.force_landing()
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
                self.force_landing()
            if done_cb:
                done_cb(success)

        future.add_done_callback(_on_done)

    def precision_landing(self, action="on"):
        transition_id = {
            "on":  Transition.TRANSITION_ACTIVATE,
            "off": Transition.TRANSITION_DEACTIVATE,
        }.get(action)
        if transition_id is None:
            self.get_logger().warn(f"Unknown action for precision_landing: {action}")
            return
        self._lifecycle_transition(self._pl_change_state_cli, transition_id, "PrecisionLanding")
            

    def record_rosbag(self, action="on", path="default"):
        transition_id = {
            "on":  Transition.TRANSITION_ACTIVATE,
            "off": Transition.TRANSITION_DEACTIVATE,
        }.get(action)
        if transition_id is None:
            self.get_logger().warn(f"Unknown action for record_rosbag: {action}")
            return

        if action == "on":
            
            output_path = f"{self.rosbag_folder_path}/{path}"
            if self.use_sim_time:
                self.get_logger().error(f'Starting recording ROS bag (simulation), Task: {self.current_task.task_id}, path:{output_path}')
                return
            if not self._bag_set_params_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("rosbag_node set_parameters service not available.")
                return
            param = Parameter()
            param.name = "output_path"
            param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=output_path)
            req = SetParameters.Request()
            req.parameters = [param]
            future = self._bag_set_params_cli.call_async(req)
            future.add_done_callback(
                lambda f: self._lifecycle_transition(
                    self._bag_change_state_cli, Transition.TRANSITION_ACTIVATE, f"Rosbag({path})")
            )
        else:
            self.get_logger().error(f'Stop recording ROS bag')
            self._lifecycle_transition(self._bag_change_state_cli, transition_id, f"Rosbag({path})")

    def threading_program(self, target_function, input):
        threading.Thread(
            target=target_function,
            args=(input,),
            daemon=True
        ).start()

    def _upload_rosbag_to_nas(self, complite_task_path):
        for task_path in complite_task_path:
            src = f"{self.rosbag_folder_path}/{task_path}"
            dst = f"{self.nas_mount_path}/{task_path}"

            # 先確認 NAS 有掛載
            if not os.path.ismount(self.nas_mount_path):
                self.get_logger().error("NAS not mounted. Skipping upload.")
                return

            try:
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.copytree(src, dst)
                # src_size = sum(f.stat().st_size for f in Path(src).rglob('*') if f.is_file())
                # dst_size = sum(f.stat().st_size for f in Path(dst).rglob('*') if f.is_file())
                # if src_size != dst_size:
                #     raise RuntimeError(f"Size mismatch: {src_size} vs {dst_size}")

                self.get_logger().info(f"Rosbag uploaded: {task_id}")
                self.send_feedback(task_id=task_id, result=1, failed_faces=[])
            except Exception as e:
                self.get_logger().error(f"Rosbag upload failed: {e}")
                self.send_feedback(task_id=task_id, result=3, failed_faces=[])

    def cleanup_subprocesses(self):
        self._lifecycle_transition(
            self._pl_change_state_cli, Transition.TRANSITION_DEACTIVATE, "PrecisionLanding")
        self._lifecycle_transition(
            self._bag_change_state_cli, Transition.TRANSITION_DEACTIVATE, "Rosbag")


def main(args=None):
    rclpy.init(args=args)
    node = ManagementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup_subprocesses()
        node.client.loop_stop()
        node.client.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
