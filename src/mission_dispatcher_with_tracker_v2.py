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
from geometry_msgs.msg import PoseStamped, Twist    # Nav2 target pose message type
from std_msgs.msg import Bool

class Nav2Executor(Node):

    def __init__(self):
        super().__init__('mission_dispatcher')

        self.callback_group = ReentrantCallbackGroup()

        # 1. Subscribe to custom topic (from MQTT Bridge)
        self.subscription = self.create_subscription(
            NavTask,
            'navigation_tasks',
            self.listener_callback,
            10,
            callback_group=self.callback_group)

        self.pub = self.create_publisher(
            NavResult,
            'navigation_result',
            10)
        
        self.ready_to_record_rosbag_pub = self.create_publisher(
            Bool,
            'ready_to_record_rosbag',
            10
        )

        # 2. Create Nav2 Action Client
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints', callback_group=self.callback_group)
        
        # 3. Tracker Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel_nav',
            self.cmd_vel_callback,
            10,
            callback_group=self.callback_group
        )
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Tracking State
        self.tracking_active = False
        self.center_x = 0.0
        self.center_y = 0.0
        self.angular_gain = 1.5

        self.get_logger().info("Nav2 Executor Node Started. Waiting for tasks...")

        self.task_id = 0
        self.cargo_queue = [] # Queue for sequential cargo legs
        self.current_leg_index = 0 # Track which leg we are on (1-4)
        self.failed_faces = [] # Store which faces failed sampling
        self.expanded_points = [] # All 4 expanded starting points
        self.current_start_index = 0 # Which starting point we are trying

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
        self.tracking_active = True
        
        self.get_logger().info(f"Cargo Task Detected! Deep Inspection Mode. Center: ({self.center_x:.2f}, {self.center_y:.2f})")
        
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

        idx = self.current_start_index
        x, y = self.expanded_points[idx]
        self.get_logger().info(
            f"Phase 1: Trying start point {idx+1}/{len(self.expanded_points)} ({x:.2f}, {y:.2f})...")
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
        future.add_done_callback(self.approach_response_callback)

    def approach_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'Approach goal rejected for start point {self.current_start_index+1}/{len(self.expanded_points)}, trying next...')
            self.current_start_index += 1
            self.try_next_start_point()
            return
            
        self.get_logger().info('Approach goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.approach_result_callback)

    def approach_result_callback(self, future):
        result = future.result().result
        self.cmd_pub.publish(Twist())  # brakes
        if len(result.missed_waypoints) == 0:
            self.get_logger().info('Phase 1: Approach Complete. Starting Phase 2: Inspection Loop.')
            # Rotate cargo_queue so inspection legs start from the successful point
            rotate_count = 0
            self.current_start_index = 0
            if rotate_count > 0 and self.cargo_queue:
                self.cargo_queue = self.cargo_queue[rotate_count:] + self.cargo_queue[:rotate_count]
            self.current_leg_index = 0
            ros_msg = Bool()
            ros_msg.data = True
            self.ready_to_record_rosbag_pub.publish(ros_msg)
            self.process_next_cargo_leg()
        else:
            self.get_logger().warn(
                f'Phase 1: Approach failed for start point {self.current_start_index+1}/{len(self.expanded_points)}, trying next...')
            self.current_start_index += 1
            self.try_next_start_point()

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
        future.add_done_callback(self.cargo_leg_response_callback)

    def cargo_leg_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Cargo leg {self.current_leg_index} rejected!')
            self.finish_task(success=False)
            return
            
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.cargo_leg_result_callback)

    def cargo_leg_result_callback(self, future):
        result = future.result().result
        self.cmd_pub.publish(Twist())  # brakes
        
        # Check missed waypoints
        # The goal had 1 point: [0: End]
        missed = result.missed_waypoints
        
        # Check if Endpoint (Index 0) was missed
        if 0 in missed:
            self.get_logger().warn(f"Face {self.current_leg_index} Endpoint unreachable.")
            self.failed_faces.append(self.current_leg_index)  # store failed cargo face
             
        # Continue to next leg
        self.process_next_cargo_leg(failed_faces=self.failed_faces)

    def finish_task(self, success, failed_faces=None):
        self.tracking_active = False
        result_msg = NavResult()
        result_msg.header.stamp = self.get_clock().now().to_msg()
        result_msg.header.frame_id = "map"
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
             
        self.pub.publish(result_msg)
        self.cmd_pub.publish(Twist())

    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected :(')
    #         return

    #     self.get_logger().info('Goal accepted! UAV is moving.')
    #     result_msg = NavResult()
    #     result_msg.header.stamp = self.get_clock().now().to_msg()
    #     result_msg.header.frame_id = "map"
    #     result_msg.task_id = self.task_id
    #     result_msg.result = 0
    #     self.pub.publish(result_msg)
    #     # Attach result_callback to know when navigation is complete
    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.get_result_callback)

    # def get_result_callback(self, future):
    #     """Callback after navigation is complete"""
    #     result = future.result().result
    #     # FollowWaypoints result usually contains missed_waypoints
    #     if len(result.missed_waypoints) == 0:
    #          self.finish_task(success=True)
    #     else:
    #          self.get_logger().warn(f'Task Finished but missed {len(result.missed_waypoints)} waypoints.')
    #          self.finish_task(success=False)

    def cmd_vel_callback(self, msg):
        """Intercept Nav2 cmd_vel and override angular velocity if tracking"""
        if not self.tracking_active:
            self.cmd_pub.publish(msg)
            return

        try:
            # Get current robot pose
            transform = self.tf_buffer.lookup_transform(
                'map',
                'body',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            # Extract yaw from quaternion
            q = transform.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            
        except Exception as e:
            # self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=1.0)
            # If TF fails, pass through original cmd_vel
            self.cmd_pub.publish(msg)
            return
        
        # Calculate desired heading (toward orbit center)
        dx = self.center_x - robot_x
        dy = self.center_y - robot_y
        desired_yaw = math.atan2(dy, dx)
        
        # Calculate yaw error (normalized to [-pi, pi])
        yaw_error = desired_yaw - robot_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        
        # Create new cmd_vel with overridden angular velocity
        new_cmd = Twist()
        new_cmd.linear.x = msg.linear.x
        new_cmd.linear.y = msg.linear.y
        new_cmd.linear.z = msg.linear.z
        
        # Override angular velocity with P-controller
        new_cmd.angular.z = self.angular_gain * yaw_error
        
        # Limit maximum angular velocity
        max_angular_vel = 0.5
        new_cmd.angular.z = max(-max_angular_vel, min(max_angular_vel, new_cmd.angular.z))
        
        # Publish
        self.cmd_pub.publish(new_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Executor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()