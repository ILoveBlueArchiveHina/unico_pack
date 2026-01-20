import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient

# Message type
from campus_delivery_msgs.msg import NavTask  # my custom messages
from nav2_msgs.action import FollowWaypoints # Nav2 Actions
from geometry_msgs.msg import PoseStamped    # Nav2 target pose message type

class Nav2Executor(Node):

    def __init__(self):
        super().__init__('mission_dispatcher')

        # 1. Subscribe to custom topic (from MQTT Bridge)
        self.subscription = self.create_subscription(
            NavTask,
            'navigation_tasks',
            self.listener_callback,
            10)

        # 2. Create Nav2 Action Client
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        self.get_logger().info("Nav2 Executor Node Started. Waiting for tasks...")

    def listener_callback(self, msg):
        """Receive task from MQTT Bridge and call Nav2"""
        self.get_logger().info(f"Received Task ID: {msg.task_id} with {len(msg.waypoints)} points")

        # Call processing function
        self.send_goal_to_nav2(msg.waypoints)

    def send_goal_to_nav2(self, waypoints_data):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = []

        # --- Key conversion block ---
        for wp in waypoints_data:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            
            # Convert theta (yaw) to Quaternion
            # Assuming roll=0, pitch=0, only yaw rotation
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(wp.theta / 2.0)
            pose.pose.orientation.w = math.cos(wp.theta / 2.0)
            
            goal_msg.poses.append(pose)
        # -------------------

        # Wait for Nav2 server to be available
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available!')
            return

        self.get_logger().info('Sending waypoints to Nav2...')
        
        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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
             self.get_logger().info('Task Completed Successfully!')
        else:
             self.get_logger().warn(f'Task Finished but missed {len(result.missed_waypoints)} waypoints.')

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Executor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()