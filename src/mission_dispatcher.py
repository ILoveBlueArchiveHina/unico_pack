#!/usr/bin/env python3
"""貨物環繞巡檢的任務派發節點（node name: mission_dispatcher）。

接收 process_manager 發來的巡檢任務，把「貨物的四個角點」換算成一圈
可以飛的環繞航點，逐面交給 Nav2 執行，最後把巡檢結果回報給 process_manager。
本節點只負責「繞著貨物飛一圈」，起飛、降落、高度、錄影、返航都不歸它管。

ROS 2 介面
  訂閱 navigation_tasks       (NavTask)  process_manager 發來的任務，需剛好 4 個 waypoint
       cancel_navigation      (Bool)     取消目前任務（低電量或人工中止時觸發）
       set_yaw_done           (Bool)     velocity_controller 回報航向已調整完成（已淘汰）
  發布 navigation_result      (NavResult) 巡檢結果，1=成功 2=失敗，附失敗的貨物面編號
       ready_to_record_rosbag (Bool)      已抵達巡檢區，通知 process_manager 開始錄製
       set_target_yaw         (Float64)   目標航向值，交給 velocity_controller 轉向（已淘汰）
       tracking_center        (Point)     貨物中心點，z=1 開始追蹤、z=0 結束追蹤
  Action follow_waypoints (Nav2 FollowWaypoints)  實際的飛行動作

任務流程
  1. 四個角點取平均得到貨物中心，再讓各角點沿「中心 → 角點」方向往外推 2.0 m，
     得到 4 個環繞航點，避免無人機貼著貨物飛。
  2. Phase 1 進場：依序嘗試這 4 個環繞航點，任何一點飛到就算進場成功，
     全部飛不到才判定任務失敗。抵達後通知開始錄製，並等 2 秒讓影像穩定。
  3. Phase 2 環繞：沿環繞航點逐面飛行，每飛完一面檢查 Nav2 是否回報 miss，
     miss 的面記錄到 failed_faces，但不中斷，繼續飛下一面。
  4. 四面走完後發布 NavResult（有任何一面失敗即回報失敗），並關閉航向追蹤。

航向控制（由參數 tracking_mode 決定，預設為 True）
  True （目前實際使用）本節點只把貨物中心發布到 tracking_center，
               路徑航向由 nav2_face_target_smoother 外掛規劃，
               讓整條路徑上的每個點都朝向貨物中心；
               MPPI controller 再依據這個航向同時控制轉向與移動，
               因此環繞過程中機頭全程對準貨物，不需要停下來轉向。
  False（已淘汰）舊做法：飛到定點才轉向，發布 set_target_yaw 給
               velocity_controller，收到 set_yaw_done 之後才飛下一面。
               因為預設是 True，這條路徑實際上不會走到，
               保留是為了日後可能還有用到的場合。

所有 Nav2 互動都以 callback chain 串接（非阻塞），並使用 ReentrantCallbackGroup，
讓取消訊號在導航進行中仍能即時被處理。
"""
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

# 訊息型別
from campus_delivery_msgs.msg import NavTask, NavResult  # 自訂訊息
from nav2_msgs.action import FollowWaypoints # Nav2 Action
from geometry_msgs.msg import PoseStamped, Point    # Nav2 目標點的訊息型別
from std_msgs.msg import Bool, Float64

PI = 3.1416

class MissionDispatcher(Node):
    def __init__(self):
        super().__init__('mission_dispatcher')

        # 用 Reentrant 讓取消訊號在導航 callback 執行中仍能被處理
        self.callback_group = ReentrantCallbackGroup()

        # 1. 訂閱 process_manager 發來的巡檢任務
        self.subscription = self.create_subscription(
            NavTask,
            'navigation_tasks',
            self.listener_callback,
            1,
            callback_group=self.callback_group)

        self.cancel_current_task_sub = self.create_subscription(    # 若接收到process_manager的取消導航訊息，
            Bool,                                                   # 立即取消目前導航。
            'cancel_navigation',
            self.cancel_current_task_callback,
            1,
            callback_group=self.callback_group
        )

        self.set_yaw_done_sub = self.create_subscription(   # 【已淘汰】如果tracking_mode=false，則不會持續面向貨物中心點，
            Bool,                                           # 而是依序調整面向角度，角度由velocity_controller調整，
            'set_yaw_done',                                 # 調整完成會發布該topic告訴此節點。
            self.set_yaw_done_callback,                     # 預設tracking_mode=true，因此實際上不會用到。
            1,
            callback_group=self.callback_group
        )

        self.set_target_yaw_pub = self.create_publisher(    # 【已淘汰】如果tracking_mode=false，則不會持續面向貨物中心點，
            Float64,                                        # 而是依序調整面向角度，發布該topic告訴velocity_controller
            'set_target_yaw',                               # 目標角度值。預設tracking_mode=true，因此實際上不會用到。
            1)

        self.result_pub = self.create_publisher(            # 回報該次任務巡檢結果至process_manager
            NavResult,
            'navigation_result',
            1)

        self.ready_to_record_rosbag_pub = self.create_publisher(    # 到達任務點後即可開始拍攝，傳送信號告訴process_manager
            Bool,                                                   # 可以開始錄製。
            'ready_to_record_rosbag',
            1
        )

        self.tracking_active_pub = self.create_publisher(   # 將貨物的中心點發布給
            Point,                                          # nav2_face_target_smoother::FaceTargetSmoother
            'tracking_center',                              # 讓該外掛知道如何規劃路徑航向，
            1                                               # 實際的轉向與移動則由 MPPI controller 執行
        )

        # 2. 建立 Nav2 Action Client，實際的飛行動作由 Nav2 執行
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints', callback_group=self.callback_group)

        # 航向追蹤狀態
        # true：航向交給 nav2_face_target_smoother 規劃、MPPI 執行（目前實際使用）
        # false：舊的定點轉向做法，已淘汰，僅保留備用
        self.declare_parameter("tracking_mode", True)
        self.tracking_mode = self.get_parameter("tracking_mode").value

        self.center_x = 0.0
        self.center_y = 0.0

        self.task_id = ''
        self.cargo_queue = [] # 待巡檢的貨物面佇列，依序執行
        self.current_leg_index = 0 # 目前巡檢到第幾面（1~4）
        self.failed_faces = [] # 記錄哪幾面沒有拍到
        self.expanded_points = [] # 往外推算後的 4 個環繞航點
        self.current_start_index = 0 # 目前嘗試的是第幾個進場點
        self._current_goal_handle = None
        self._is_task_cancelled = False

        self.get_logger().info("Mission Dispatcher Node Started. Waiting for tasks...")

    def listener_callback(self, msg):
        """ 收到任務，只接受剛好 4 個 waypoint 的貨物巡檢任務 """
        self.get_logger().info(f"Received Task ID: {msg.task_id} with {len(msg.waypoints)} points")
        self.task_id = msg.task_id

        if len(msg.waypoints) == 4:
             self.handle_cargo_task(msg.waypoints)
        else:
            self.get_logger().warn(f'Refuse to execute a task with {len(msg.waypoints)} waypoints.')


    def handle_cargo_task(self, waypoints_data):
        """ 把四個角點換算成環繞航點，並啟動 Phase 1 進場 """
        xs = [wp.x for wp in waypoints_data]
        ys = [wp.y for wp in waypoints_data]
        self.center_x = sum(xs) / 4.0
        self.center_y = sum(ys) / 4.0
        tracking_center = Point()
        tracking_center.x = self.center_x
        tracking_center.y = self.center_y
        tracking_center.z = 1.0         # z=1 代表通知外掛開始航向追蹤
        self.get_logger().info(f"Cargo Task Detected! Deep Inspection Mode. Center: ({self.center_x:.2f}, {self.center_y:.2f})")
        self.tracking_active_pub.publish(tracking_center)

        # 1. 將四個角點往外推，避免無人機貼著貨物飛
        expanded_points = []
        for i in range(4):
            dx = xs[i] - self.center_x
            dy = ys[i] - self.center_y

            # 正規化成單位向量
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                ux = dx / length
                uy = dy / length
            else:
                ux, uy = 0, 0

            # 新的點 = 原本的角點沿著中心往外 2.0 公尺
            new_x = xs[i] + ux * 2.0
            new_y = ys[i] + uy * 2.0
            expanded_points.append((new_x, new_y))

        # 2. 產生每一面的航段：目前角點 -> 下一個角點
        self.cargo_queue = []
        self.failed_faces = [] # 重設失敗清單

        for i in range(4):
            p_end = expanded_points[(i+1)%4]

            # 每個航段只放終點
            leg_points = [p_end]
            self.cargo_queue.append(leg_points)

        self.get_logger().info(f"Generated {len(self.cargo_queue)} inspection legs. Starting execution...")

        # Phase 1：先進場到第一個點，飛不到就換下一個點試
        self.expanded_points = expanded_points
        self.current_start_index = 0
        self.try_next_start_point()

    def try_next_start_point(self):
        """ 嘗試下一個可用的進場點，4 個都失敗才判定任務失敗 """
        if self.current_start_index >= len(self.expanded_points):
            self.get_logger().error('All 4 starting points failed! Reporting task failure.')
            self.finish_task(success=False)
            return

        x, y = self.expanded_points[self.current_start_index]
        self.get_logger().info(
            f"Phase 1: Trying start point {self.current_start_index+1}/{len(self.expanded_points)} ({x:.2f}, {y:.2f})...")
        self.start_approach(x, y)

    def start_approach(self, x, y):
        """ Phase 1：飛往進場用的環繞航點 """
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self.create_pose(x, y)]

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available for approach!')
            self.current_start_index += 1
            self.try_next_start_point()
            return

        future = self._action_client.send_goal_async(goal_msg)

        def approach_response_callback(future):
            """ Nav2 是否接受這個進場目標會在這裡收到 """
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(
                    f'Approach goal rejected for start point {self.current_start_index+1}/{len(self.expanded_points)}, trying next...')
                self.current_start_index += 1
                self.try_next_start_point()
                return

            self.get_logger().info('Approach goal accepted.')
            self._current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(approach_result_callback)

        def approach_result_callback(future):
            """ 進場飛行的結果會在這裡收到 """
            self._current_goal_handle = None
            if self._is_task_cancelled:
                self._is_task_cancelled = False
                return

            result = future.result().result
            if len(result.missed_waypoints) == 0:
                self.get_logger().info('Phase 1: Approach Complete. Starting Phase 2: Inspection Loop.')
                ros_msg = Bool()
                ros_msg.data = True
                self.ready_to_record_rosbag_pub.publish(ros_msg)

                self.current_start_index = 0
                self.current_leg_index = 0

                # 用一次性計時器等 2 秒（非阻塞），確保 rosbag 真的開始錄
                # 之後才進入環繞巡檢，計時器觸發後會自我銷毀。
                self._rosbag_ready_timer = self.create_timer(2.0, self._start_inspection_after_delay)
            else:
                self.get_logger().warn(
                    f'Phase 1: Approach failed for start point {self.current_start_index+1}/{len(self.expanded_points)}, trying next...')
                self.current_start_index += 1
                self.try_next_start_point()

        future.add_done_callback(approach_response_callback)

    def _start_inspection_after_delay(self):
        """ 一次性計時器的 callback：等 2 秒讓影像穩定後才開始巡檢 """
        self._rosbag_ready_timer.destroy()
        if self.tracking_mode:
            self.process_next_cargo_leg(failed_faces=self.failed_faces)
        else:
            self.set_target_yaw(self.current_leg_index)  # 已淘汰的定點轉向路徑

    def set_target_yaw(self, current_point):
        """ 【已淘汰】非鏡頭追蹤時會單獨調整無人機的航向，始其面向貨物一側並保持平行。
            預設 tracking_mode=true，航向改由 nav2_face_target_smoother 規劃、
            MPPI controller 執行，因此這條路徑實際上不會走到，僅保留備用。
        """
        msg = Float64()
        if current_point == 0:
            msg.data = -PI/2
        elif current_point == 1:
            msg.data = -PI
        elif current_point == 2:
            msg.data = PI/2
        elif current_point == 3 or current_point == 4:
            msg.data = 0.0
        self.set_target_yaw_pub.publish(msg)

    def set_yaw_done_callback(self, msg):
        """ 【已淘汰】velocity_controller調整完航向後，進行下一個貨物面拍攝。
            只有 tracking_mode=false 才會用到，預設不會走到這裡。
        """
        if msg.data:
            self.process_next_cargo_leg(failed_faces=self.failed_faces)
            return

    def create_pose(self, x, y):
        """ 把 x, y 包成 Nav2 需要的 map 座標系目標點 """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0 # 預設朝向，實際路徑航向由 nav2_face_target_smoother 規劃後覆寫
        return pose

    def process_next_cargo_leg(self, failed_faces=None):
        """ Phase 2：執行佇列中的下一個貨物面，佇列空了就結束任務 """
        if not self.cargo_queue:
            self.get_logger().info("All cargo legs completed!")
            if failed_faces:
                success=False
            else:
                success=True

            self.finish_task(success=success, failed_faces=failed_faces)
            return

        current_leg_points = self.cargo_queue.pop(0)
        self.current_leg_index += 1
        self.get_logger().info(f"Starting Leg {self.current_leg_index}/4 (FollowWaypoints Mode). Remaining: {len(self.cargo_queue)}")

        # 每一面都統一用 FollowWaypoints 執行
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self.create_pose(p[0], p[1]) for p in current_leg_points]

        if not self._action_client.wait_for_server(timeout_sec=5.0):
             self.get_logger().error('Nav2 Action Server not available!')
             self.finish_task(success=False)
             return

        future = self._action_client.send_goal_async(goal_msg)

        def cargo_leg_response_callback(future):
            """ Nav2 是否接受這一面的目標會在這裡收到 """
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'Cargo leg {self.current_leg_index} rejected!')
                self.finish_task(success=False)
                return

            self._current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(cargo_leg_result_callback)

        def cargo_leg_result_callback(future):
            """ 這一面飛完的結果會在這裡收到 """
            self._current_goal_handle = None
            if self._is_task_cancelled:
                self._is_task_cancelled = False
                return
            result = future.result().result

            # 檢查有沒有飛不到的航點，這個 goal 只有 1 個點（索引 0 為終點）
            missed = result.missed_waypoints

            # 終點（索引 0）沒飛到就記錄成失敗的面，但不中斷，繼續飛下一面
            if 0 in missed:
                self.get_logger().warn(f"Face {self.current_leg_index} Endpoint unreachable.")
                self.failed_faces.append(self.current_leg_index)  # 記錄拍攝失敗的貨物面

            if self.tracking_mode:
                self.process_next_cargo_leg(failed_faces=self.failed_faces)
            else:
                self.set_target_yaw(self.current_leg_index)  # 已淘汰的定點轉向路徑

        future.add_done_callback(cargo_leg_response_callback)

    def cancel_current_task_callback(self, msg):
        """ 收到取消訊號：中止 Nav2 目標、清空佇列並回報失敗 """
        if msg.data and self._current_goal_handle is not None:
            self._is_task_cancelled = True
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
            self.cargo_queue.clear()
            self.expanded_points = []
            self.finish_task(success=False)

    def finish_task(self, success, failed_faces=None):
        """ 結束任務：關閉航向追蹤並把結果回報給 process_manager """
        if failed_faces is None:
            failed_faces = []
        result_msg = NavResult()
        result_msg.task_id = self.task_id
        result_msg.failed_faces = failed_faces

        # 判斷邏輯：Phase 1 進場失敗直接算失敗；
        # 進場成功則看 Phase 2 有沒有哪一面拍攝失敗。
        if not success:
             result_msg.result = 2 # 進場失敗或其他錯誤
             self.get_logger().warn('Task Failed (Approach or Generic).')
        else:
             result_msg.result = 1
             self.get_logger().info('Task Completed Successfully!')

        tracking_center = Point()
        tracking_center.x = 0.0
        tracking_center.y = 0.0
        tracking_center.z = 0.0         # z=0 代表通知外掛關閉航向追蹤
        self.tracking_active_pub.publish(tracking_center)
        self.result_pub.publish(result_msg)



def main(args=None):
    rclpy.init(args=args)
    node = MissionDispatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
