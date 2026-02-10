#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import Buffer, TransformListener
from action_msgs.msg import GoalStatus
from functools import partial
from collections import deque

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
        
        # XTE Monitoring State
        self.current_leg_start = (0.0, 0.0)
        self.current_leg_end = (0.0, 0.0)
        self.monitoring_timer = None
        self.current_goal_handle = None # Store handle to cancel if needed
        
        self.is_busy = False # (Optional now, but good for logging)
        self.execution_seq = 0 # Sequence ID to invalidate old callbacks

    def listener_callback(self, msg):
        """Receive task from MQTT Bridge and Overwrite current task"""
        self.get_logger().info(f"Received Task ID: {msg.task_id}. Overwriting/Starting...")
        
        # Increment execution sequence to invalidate ANY pending callbacks from previous tasks
        self.execution_seq += 1
        
        # Stop any previous monitoring or logic
        self.stop_monitoring()
        if hasattr(self, 'next_task_timer') and self.next_task_timer:
            self.next_task_timer.cancel()
            self.next_task_timer = None
            
        # Reset State
        self.is_busy = True
        self.task_id = msg.task_id
        self.tracking_active = False
        self.cargo_queue = []
        self.failed_faces = []
        
        # Start new task
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
            
            # Create Path: Endpoint Only
            leg_info = {'start': p_start, 'end': p_end}
            self.cargo_queue.append(leg_info)
            
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
            # self.finish_task(success=False) # Skip validation here as it's immediate
            return
            
        future = self._action_client.send_goal_async(goal_msg)
        # Pass current seq to callback
        future.add_done_callback(partial(self.approach_response_callback, seq=self.execution_seq))

    def approach_response_callback(self, future, seq):
        if seq != self.execution_seq: return
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Approach goal rejected!')
            self.finish_task(success=False, seq=seq)
            return
            
        self.get_logger().info('Approach goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self.approach_result_callback, seq=seq))

    def approach_result_callback(self, future, seq):
        if seq != self.execution_seq: return

        result = future.result().result
        if len(result.missed_waypoints) == 0:
             self.get_logger().info('Phase 1: Approach Complete. Starting Phase 2: Inspection Loop.')
             self.current_leg_index = 0
             self.process_next_cargo_leg()
        else:
             self.get_logger().warn('Phase 1: Approach Failed.')
             self.finish_task(success=False, seq=seq)

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
            self.finish_task(success=True, seq=self.execution_seq)
            return
            
        leg_info = self.cargo_queue.pop(0)
        self.current_leg_index += 1
        
        # Setup XTE Monitoring parameters
        self.current_leg_start = leg_info['start']
        self.current_leg_end = leg_info['end']
        
        self.get_logger().info(f"Starting Face {self.current_leg_index}/4. Target: {self.current_leg_end}")
        
        # Target: Endpoint only
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [self.create_pose(self.current_leg_end[0], self.current_leg_end[1])]
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
             self.get_logger().error('Nav2 Action Server not available!')
             self.finish_task(success=False, seq=self.execution_seq)
             return
             
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(partial(self.cargo_leg_response_callback, seq=self.execution_seq))

    def cargo_leg_response_callback(self, future, seq):
        if seq != self.execution_seq: return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Cargo leg {self.current_leg_index} rejected!')
            self.finish_task(success=False, seq=seq)
            return
            
        self.current_goal_handle = goal_handle
        
        # Start XTE Monitoring Timer (10Hz)
        self.monitoring_timer = self.create_timer(0.1, self.monitoring_callback)
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self.cargo_leg_result_callback, seq=seq))

    def monitoring_callback(self):
        """Check Cross-Track Error"""
        # Note: Monitoring timer runs freely, but checking self.execution_seq inside timer is hard because timer is persistent? 
        # Actually timer is recreated in cargo_leg_response_callback.
        # But if old timer is not cancelled properly (e.g. race condition), it might run.
        # We did self.stop_monitoring() in listener_callback, so it should be fine.
        try:
             # Get robot position
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0)
            )
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y
            
            # Line Segment P1->P2
            p1x, p1y = self.current_leg_start
            p2x, p2y = self.current_leg_end
            
            # Vector P1->P2
            dx = p2x - p1x
            dy = p2y - p1y
            
            # Vector P1->Robot
            drx = rx - p1x
            dry = ry - p1y
            
            # Project Robot onto line (dot product) / Length^2
            l2 = dx*dx + dy*dy
            if l2 == 0: return # Zero length leg?
            
            t = (drx*dx + dry*dy) / l2
            
            # Perpendicular distance check
            # Cross product method for 2D XTE: |dx*dry - dy*drx| / Sqrt(l2)
            cross_prod = abs(dx*dry - dy*drx)
            xte = cross_prod / math.sqrt(l2)
            
            # Threshold Check
            if xte > 1.5:
                self.get_logger().warn(f"Detour Detected! XTE: {xte:.2f}m > 1.5m. Marking Face {self.current_leg_index} as Failed (Sampling Error) but continuing navigation.")
                self.failed_faces.append(self.current_leg_index)
                self.stop_monitoring()
                # Do NOT Cancel Goal - let it complete the detour to avoid getting stuck
                    
        except Exception as e:
            pass

    def stop_monitoring(self):
        if self.monitoring_timer:
            self.monitoring_timer.cancel()
            self.monitoring_timer = None

    def cargo_leg_result_callback(self, future, seq):
        if seq != self.execution_seq: return
        self.stop_monitoring() # Ensure timer stopped
        
        result = future.result()
        status = result.status
        
        # Check if cancelled (by us due to XTE) or failed
        if status == GoalStatus.STATUS_CANCELED:
             self.get_logger().warn(f"Face {self.current_leg_index} Cancelled (Detour).")
             self.failed_faces.append(self.current_leg_index)
        elif status == GoalStatus.STATUS_ABORTED:
             self.get_logger().warn(f"Face {self.current_leg_index} Aborted.")
             self.failed_faces.append(self.current_leg_index)
        elif status == GoalStatus.STATUS_SUCCEEDED:
             self.get_logger().info(f"Face {self.current_leg_index} Succeeded.")
        else:
             # Other statuses
             pass
             
        # Continue to next leg
        self.process_next_cargo_leg()

    def finish_task(self, success, seq=None):
        """Finish Task and Report. Check sequence if provided."""
        if seq is not None and seq != self.execution_seq:
            self.get_logger().info(f"Ignoring finish_task for old sequence {seq}. Current: {self.execution_seq}")
            return

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
        
        self.is_busy = False

    def goal_response_callback(self, future, seq):
        if seq != self.execution_seq: return
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.is_busy = False 
            return

        self.get_logger().info('Goal accepted! UAV is moving.')
        
        # Attach result_callback to know when navigation is complete
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(partial(self.get_result_callback, seq=seq))

    def get_result_callback(self, future, seq):
        if seq != self.execution_seq: return

        """Callback after navigation is complete"""
        result = future.result().result
        # FollowWaypoints result usually contains missed_waypoints
        if len(result.missed_waypoints) == 0:
             self.finish_task(success=True, seq=seq)
        else:
             self.get_logger().warn(f'Task Finished but missed {len(result.missed_waypoints)} waypoints.')
             self.finish_task(success=False, seq=seq)

    def handle_standard_task(self, waypoints_data):
        self.get_logger().info("Standard Task Detected")
        # Standard logic
        goal_msg = FollowWaypoints.Goal()
        # Create standard poses
        poses = []
        for wp in waypoints_data:
            poses.append(self.create_pose(wp.x, wp.y))
        
        goal_msg.poses = poses
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
             self.get_logger().error('Nav2 Action Server not available!')
             self.is_busy = False
             return
             
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(partial(self.goal_response_callback, seq=self.execution_seq))

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()