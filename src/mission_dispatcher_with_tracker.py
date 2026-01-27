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

    def listener_callback(self, msg):
        """Receive task from MQTT Bridge and call Nav2"""
        self.get_logger().info(f"Received Task ID: {msg.task_id} with {len(msg.waypoints)} points")
        self.task_id = msg.task_id

        # Call processing function
        self.send_goal_to_nav2(msg.waypoints)

    def send_goal_to_nav2(self, waypoints_data):
        # Check for 4-waypoint cargo task
        if len(waypoints_data) == 4:
             self.handle_cargo_task(waypoints_data)
        else:
             self.handle_standard_task(waypoints_data)

    def handle_cargo_task(self, waypoints_data):
        """Handle 4-waypoint cargo inspection with loop expansion"""
        xs = [wp.x for wp in waypoints_data]
        ys = [wp.y for wp in waypoints_data]
        self.center_x = sum(xs) / 4.0
        self.center_y = sum(ys) / 4.0
        self.tracking_active = True
        
        self.get_logger().info(f"Cargo Task Detected! Deep Inspection Mode. Center: ({self.center_x:.2f}, {self.center_y:.2f})")
        
        # 1. Expand corners by 1.0m
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
            new_x = xs[i] + ux * 1.0
            new_y = ys[i] + uy * 1.0
            expanded_points.append((new_x, new_y))
            
        # 2. Generate legs (faces) logic: Corner -> Mid -> NextCorner
        self.cargo_queue = []
        self.failed_faces = [] # Reset failure list
        
        for i in range(4):
            p_start = expanded_points[i]
            p_end = expanded_points[(i+1)%4]
            
            # Midpoint
            p_mid = ((p_start[0] + p_end[0])/2.0, (p_start[1] + p_end[1])/2.0)
            
            # Create Path: Mid -> End (Start excluded)
            leg_points = [p_mid, p_end]
            self.cargo_queue.append(leg_points)
            
        self.get_logger().info(f"Generated {len(self.cargo_queue)} inspection legs. Starting execution...")
        
        # Phase 1: Approach the first point (expanded_points[0])
        start_pose_x = expanded_points[0][0]
        start_pose_y = expanded_points[0][1]
        self.start_approach(start_pose_x, start_pose_y)

    def start_approach(self, x, y):
        """Phase 1: Navigate to the starting corner"""
        self.get_logger().info(f"Phase 1: Approaching start point ({x:.2f}, {y:.2f})...")
        
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self.create_pose(x, y)]
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available for approach!')
            self.finish_task(success=False)
            return
            
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.approach_response_callback)

    def approach_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Approach goal rejected!')
            self.finish_task(success=False)
            return
            
        self.get_logger().info('Approach goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.approach_result_callback)

    def approach_result_callback(self, future):
        result = future.result().result
        if len(result.missed_waypoints) == 0:
             self.get_logger().info('Phase 1: Approach Complete. Starting Phase 2: Inspection Loop.')
             self.current_leg_index = 0
             self.process_next_cargo_leg()
        else:
             self.get_logger().warn('Phase 1: Approach Failed.')
             self.finish_task(success=False)

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0 # Default orientation, will be overridden by tracker
        return pose

    def process_next_cargo_leg(self):
        """Execute next leg in the queue"""
        if not self.cargo_queue:
            self.get_logger().info("All cargo legs completed!")
            self.finish_task(success=True)
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
        
        # Check missed waypoints
        # The goal had 2 points: [0: Mid, 1: End]
        missed = result.missed_waypoints
        
        # Check if Midpoint (Index 0) was missed
        if 0 in missed:
             self.get_logger().warn(f"Face {self.current_leg_index} Sampling Failed (Midpoint unreachable). Skipped to Endpoint.")
             self.failed_faces.append(self.current_leg_index)
        
        # Check if Endpoint (Index 1) was missed 
        # If endpoint missed, we might be stuck, but we proceed to next leg anyway
        if 1 in missed:
             self.get_logger().warn(f"Face {self.current_leg_index} Endpoint unreachable.")
             # Potentially add to failed list or handle separately. For now, we assume critical if endpoint fails.
             # But user logic implies we just keep going.
             
        # Continue to next leg
        self.process_next_cargo_leg()

    def finish_task(self, success):
        self.tracking_active = False
        result_msg = NavResult()
        result_msg.header.stamp = self.get_clock().now().to_msg()
        result_msg.header.frame_id = "map"
        result_msg.task_id = self.task_id
        
        # Logic: If Phase 1 (success arg) failed, return 1.
        # If Phase 1 succeeded, check if any sampling failed during Phase 2.
        if not success:
             result_msg.result = 1 # Approach failed or generic
             self.get_logger().warn('Task Failed (Approach or Generic).')
        elif self.failed_faces:
             # Report the first failed face
             first_fail = self.failed_faces[0]
             result_msg.result = 10 + first_fail
             self.get_logger().warn(f'Task Completed with Sampling Failures. First Fail: Face {first_fail}')
        else:
             result_msg.result = 0
             self.get_logger().info('Task Completed Successfully!')
             
        self.pub.publish(result_msg)
        self.cmd_pub.publish(Twist())

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted! UAV is moving.')
        
        # Attach result_callback to know when navigation is complete
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback after navigation is complete"""
        result = future.result().result
        # FollowWaypoints result usually contains missed_waypoints
        if len(result.missed_waypoints) == 0:
             self.finish_task(success=True)
        else:
             self.get_logger().warn(f'Task Finished but missed {len(result.missed_waypoints)} waypoints.')
             self.finish_task(success=False)

    def cmd_vel_callback(self, msg):
        """Intercept Nav2 cmd_vel and override angular velocity if tracking"""
        if not self.tracking_active:
            self.cmd_pub.publish(msg)
            return

        try:
            # Get current robot pose
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
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