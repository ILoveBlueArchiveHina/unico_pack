#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import Buffer, TransformListener
from action_msgs.msg import GoalStatus

# Message type
from campus_delivery_msgs.msg import NavTask, NavResult  # My custom messages
from nav2_msgs.action import FollowWaypoints # Nav2 Actions
from geometry_msgs.msg import PoseStamped, Point    # Nav2 target pose message type
from std_msgs.msg import Bool, Float64

PI = 3.1416

class Nav2Executor(Node):

    def __init__(self):
        super().__init__('mission_dispatcher')

        self.callback_group = ReentrantCallbackGroup()

        # 1. Subscribe to custom topic (from MQTT Bridge)
        self.subscription = self.create_subscription(
            NavTask,
            'navigation_tasks',
            self.listener_callback,
            1,
            callback_group=self.callback_group)

        self.cancel_current_task_sub = self.create_subscription(
            Bool,
            'cancel_navigation', 
            self.cancel_current_task_callback, 
            1,
            callback_group=self.callback_group
        )

        self.set_yaw_done_sub = self.create_subscription(
            Bool,
            'set_yaw_done', 
            self.set_yaw_done_callback, 
            1,
            callback_group=self.callback_group
        )

        self.set_target_yaw_pub = self.create_publisher(
            Float64,
            'set_target_yaw',
            1)

        self.result_pub = self.create_publisher(
            NavResult,
            'navigation_result',
            1)
        
        self.ready_to_record_rosbag_pub = self.create_publisher(
            Bool,
            'ready_to_record_rosbag',
            1
        )

        self.tracking_active_pub = self.create_publisher(
            Point,
            'tracking_center',
            1
        )

        # 2. Create Nav2 Action Client
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints', callback_group=self.callback_group)
        
        # Tracking State
        self.tracking_mode = True
        self.center_x = 0.0
        self.center_y = 0.0
        self.angular_gain = 1.5

        self.get_logger().info("Nav2 Executor Node Started. Waiting for tasks...")

        self.task_id = ''
        self.cargo_queue = [] # Queue for sequential cargo legs
        self.current_leg_index = 0 # Track which leg we are on (1-4)
        self.failed_faces = [] # Store which faces failed sampling
        self.expanded_points = [] # All 4 expanded starting points
        self.current_start_index = 0 # Which starting point we are trying
        self._current_goal_handle = None
        self._is_task_cancelled = False

    def listener_callback(self, msg):
        """Receive task from MQTT Bridge and call Nav2"""
        self.get_logger().info(f"Received Task ID: {msg.task_id} with {len(msg.waypoints)} points")
        self.task_id = msg.task_id

        if len(msg.waypoints) == 4:
             self.handle_cargo_task(msg.waypoints)
        else:
            self.get_logger().warn(f'Refuse to execute a task with {len(msg.waypoints)} waypoints.')


    def handle_cargo_task(self, waypoints_data):
        """Handle 4-waypoint cargo inspection with loop expansion"""
        xs = [wp.x for wp in waypoints_data]
        ys = [wp.y for wp in waypoints_data]
        self.center_x = sum(xs) / 4.0
        self.center_y = sum(ys) / 4.0
        tracking_center = Point()
        tracking_center.x = self.center_x
        tracking_center.y = self.center_y
        tracking_center.z = 1.0         # Signal for start tracking
        self.get_logger().info(f"Cargo Task Detected! Deep Inspection Mode. Center: ({self.center_x:.2f}, {self.center_y:.2f})")
        self.tracking_active_pub.publish(tracking_center)
        
        # 1. Expand corners
        expanded_points = []
        for i in range(4):
            dx = xs[i] - self.center_x
            dy = ys[i] - self.center_y
            
            # Normalize vector
            length = math.sqrt(dx*dx + dy*dy)
            if length > 0:
                ux = dx / length
                uy = dy / length
            else:
                ux, uy = 0, 0
                
            # New point = Old Point + 1.0m outward
            new_x = xs[i] + ux * 2.0
            new_y = ys[i] + uy * 2.0
            expanded_points.append((new_x, new_y))
            
        # 2. Generate legs (faces) logic: Corner -> NextCorner
        self.cargo_queue = []
        self.failed_faces = [] # Reset failure list
        
        for i in range(4):
            p_end = expanded_points[(i+1)%4]
            
            # Create Path: End
            leg_points = [p_end]
            self.cargo_queue.append(leg_points)
            
        self.get_logger().info(f"Generated {len(self.cargo_queue)} inspection legs. Starting execution...")
        
        # Phase 1: Approach the first point (try each expanded point on failure)
        self.expanded_points = expanded_points
        self.current_start_index = 0
        self.try_next_start_point()

    def try_next_start_point(self):
        """Try approaching the next available starting point. Fail task if all exhausted."""
        if self.current_start_index >= len(self.expanded_points):
            self.get_logger().error('All 4 starting points failed! Reporting task failure.')
            self.finish_task(success=False)
            return

        x, y = self.expanded_points[self.current_start_index]
        self.get_logger().info(
            f"Phase 1: Trying start point {self.current_start_index+1}/{len(self.expanded_points)} ({x:.2f}, {y:.2f})...")
        self.start_approach(x, y)

    def start_approach(self, x, y):
        """Phase 1: Navigate to the starting corner"""
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self.create_pose(x, y)]
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available for approach!')
            self.current_start_index += 1
            self.try_next_start_point()
            return
            
        future = self._action_client.send_goal_async(goal_msg)

        def approach_response_callback(future):
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
            self._current_goal_handle = None
            if self._is_task_cancelled:
                self._is_task_cancelled = False
                return

            result = future.result().result
            if len(result.missed_waypoints) == 0:
                self.get_logger().info('Phase 1: Approach Complete. Starting Phase 2: Inspection Loop.')
                if self.tracking_mode:
                    self.process_next_cargo_leg(failed_faces=self.failed_faces)
                else:
                    self.set_target_yaw(self.current_leg_index)

                rotate_count = 0
                self.current_start_index = 0
                if rotate_count > 0 and self.cargo_queue:
                    self.cargo_queue = self.cargo_queue[rotate_count:] + self.cargo_queue[:rotate_count]
                self.current_leg_index = 0
                ros_msg = Bool()
                ros_msg.data = True
                self.ready_to_record_rosbag_pub.publish(ros_msg)
                
            else:
                self.get_logger().warn(
                    f'Phase 1: Approach failed for start point {self.current_start_index+1}/{len(self.expanded_points)}, trying next...')
                self.current_start_index += 1
                self.try_next_start_point()

        future.add_done_callback(approach_response_callback)

    def set_target_yaw(self, current_point):
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
        if msg.data:
            # Rotate cargo_queue so inspection legs start from the successful point
            self.process_next_cargo_leg(failed_faces=self.failed_faces)
            return

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0 # Default orientation, will be overridden by tracker
        return pose

    def process_next_cargo_leg(self, failed_faces=None):
        """Execute next leg in the queue"""
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
        
        # Unified: Use FollowWaypoints for the leg (Mid -> End)
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self.create_pose(p[0], p[1]) for p in current_leg_points]
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
             self.get_logger().error('Nav2 Action Server not available!')
             self.finish_task(success=False)
             return
             
        future = self._action_client.send_goal_async(goal_msg)

        def cargo_leg_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'Cargo leg {self.current_leg_index} rejected!')
                self.finish_task(success=False)
                return

            self._current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(cargo_leg_result_callback)

        def cargo_leg_result_callback(future):
            self._current_goal_handle = None
            if self._is_task_cancelled:
                self._is_task_cancelled = False
                return
            result = future.result().result
            
            # Check missed waypoints
            # The goal had 1 point: [0: End]
            missed = result.missed_waypoints
            
            # Check if Endpoint (Index 0) was missed
            if 0 in missed:
                self.get_logger().warn(f"Face {self.current_leg_index} Endpoint unreachable.")
                self.failed_faces.append(self.current_leg_index)  # store failed cargo face
            
            if self.tracking_mode:
                self.process_next_cargo_leg(failed_faces=self.failed_faces)
            else:
                self.set_target_yaw(self.current_leg_index)

        future.add_done_callback(cargo_leg_response_callback)

    def cancel_current_task_callback(self, msg):
        if msg.data and self._current_goal_handle is not None:
            self._is_task_cancelled = True
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None
            self.cargo_queue.clear()
            self.expanded_points = []
            self.finish_task(success=False)

    def finish_task(self, success, failed_faces=None):
        if failed_faces is None:
            failed_faces = []
        result_msg = NavResult()
        result_msg.task_id = self.task_id
        result_msg.failed_faces = failed_faces
        
        # Logic: If Phase 1 (success arg) failed, return 1.
        # If Phase 1 succeeded, check if any sampling failed during Phase 2.
        if not success:
             result_msg.result = 2 # Approach failed or generic
             self.get_logger().warn('Task Failed (Approach or Generic).')
        else:
             result_msg.result = 1
             self.get_logger().info('Task Completed Successfully!')
             
        self.result_pub.publish(result_msg)

    

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Executor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()