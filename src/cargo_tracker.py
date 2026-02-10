#!/usr/bin/env python3
"""
Cargo Tracker Node

This node provides cargo tracking capability by:
1. Generating circular waypoints around a target point
2. Using Nav2's FollowWaypoints action for path planning and obstacle avoidance
3. Overriding the angular velocity to keep the drone facing the orbit center
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav2_msgs.action import FollowWaypoints
from tf2_ros import Buffer, TransformListener
import math


class CargoTracker(Node):
    def __init__(self):
        super().__init__('cargo_tracker')
        
        # Parameters
        # self.declare_parameter('target_x', 0.0)  # Cargo center X (map frame)
        # self.declare_parameter('target_y', 0.0)  # Cargo center Y (map frame)
        # self.declare_parameter('radius', 5.0)    # Cargo radius (meters)
        self.declare_parameter('num_waypoints', 6)  # Number of waypoints on the circle
        self.declare_parameter('angular_gain', 1.5)  # P-gain for heading control
        self.declare_parameter('auto_start', True)  # Auto-start orbit on launch
        
        # self.target_x = self.get_parameter('target_x').value
        # self.target_y = self.get_parameter('target_y').value
        # self.radius = self.get_parameter('radius').value
        self.num_waypoints = self.get_parameter('num_waypoints').value
        self.angular_gain = self.get_parameter('angular_gain').value
        
        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Action client for NavigateThroughPoses
        self.callback_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(
            self, 
            FollowWaypoints, 
            'follow_waypoints',
            callback_group=self.callback_group
        )
        
        # Subscribe to controller's cmd_vel and override angular velocity
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel_nav',
            self.cmd_vel_callback,
            10,
            callback_group=self.callback_group
        )

        # Subscribe target point topic
        self.target_sub = self.create_subscription(
            Point,
            'mission',
            self.target_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Publish final cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # State
        self.orbiting = False
        self.current_goal_handle = None
        
        self.get_logger().info('Cargo Tracker initialized, waiting for mission....')
        
        
        # Auto-start if enabled
        # if self.get_parameter('auto_start').value:
        #     self.get_logger().info('Auto-starting orbit...')
        #     self.start_orbit()

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.radius = msg.z
        self.get_logger().info(f'Target: ({self.target_x}, {self.target_y}), Radius: {self.radius}m')
        self.start_orbit()
    
    def start_orbit(self):
        """Generate circular waypoints and send to Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigate through poses action server not available!')
            return
        
        # Generate waypoints
        poses = []
        for i in range(self.num_waypoints):
            angle = 2.0 * math.pi * i / self.num_waypoints
            x = self.target_x + self.radius * math.cos(angle)
            y = self.target_y + self.radius * math.sin(angle)
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Set orientation to face toward the orbit center
            # This ensures Nav2 correctly judges goal arrival
            facing_angle = angle + math.pi  # Point inward (toward center)
            pose.pose.orientation.z = math.sin(facing_angle / 2.0)
            pose.pose.orientation.w = math.cos(facing_angle / 2.0)
            
            poses.append(pose)
        
        # Create action goal
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses
        
        self.get_logger().info(f'Sending {len(poses)} waypoints to Nav2...')
        
        # Send goal
        self.orbiting = True
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        send_goal_future.add_done_callback(self.nav_goal_response_callback)
    
    def nav_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Cargo goal rejected by Nav2!')
            self.orbiting = False
            return
        
        self.get_logger().info('Cargo goal accepted by Nav2')
        self.current_goal_handle = goal_handle
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle Nav2 feedback"""
        # NavigateThroughPoses feedback contains: current_pose, navigation_time, number_of_poses, etc.
        # Simply log that we're receiving feedback
        self.get_logger().debug(
            'Receiving navigation feedback from Nav2',
            throttle_duration_sec=5.0
        )
    
    def nav_result_callback(self, future):
        """Handle Nav2 result"""
        # result = future.result().result
        # self.get_logger().info(f'Cargo completed with result code: {result.result}')
        
        # Send zero velocity to stop the drone immediately
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        self.get_logger().info('Published zero velocity command to stop drone')
        
        # Optionally restart orbit (continuous loop)
        # Uncomment the next line for continuous orbiting
        # self.start_orbit()
        
        self.orbiting = False
    
    def cmd_vel_callback(self, msg):
        """Override angular velocity to face orbit center"""
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
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=1.0)
            # If TF fails, pass through original cmd_vel
            self.cmd_pub.publish(msg)
            return
        
        # Calculate desired heading (toward orbit center)
        dx = self.target_x - robot_x
        dy = self.target_y - robot_y
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
        
        # Limit maximum angular velocity to 0.5 rad/s
        max_angular_vel = 0.5
        new_cmd.angular.z = max(-max_angular_vel, min(max_angular_vel, new_cmd.angular.z))
        
        # Publish
        self.cmd_pub.publish(new_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CargoTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
