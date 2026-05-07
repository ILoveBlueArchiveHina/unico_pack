#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class FastLioVisionBridge(Node):
    def __init__(self):
        super().__init__('fastlio_vision_bridge')

        # 宣告參數
        self.declare_parameter('fastlio_odom_topic', '/Odometry')
        self.declare_parameter('mavros_vision_pose_topic', '/mavros/vision_pose/pose_cov')
        self.declare_parameter('output_frame_id', 'odom')
        
        # 協方差縮放參數
        self.declare_parameter('position_cov_scale', 1000.0)      # 位置協方差放大倍數
        self.declare_parameter('orientation_cov_scale', 1000.0)   # 姿態協方差放大倍數
        self.declare_parameter('min_position_cov', 0.01)          # 最小位置協方差
        self.declare_parameter('min_orientation_cov', 0.01)       # 最小姿態協方差

        # 讀取參數
        fastlio_topic = self.get_parameter('fastlio_odom_topic').value
        mavros_topic = self.get_parameter('mavros_vision_pose_topic').value
        self.output_frame = self.get_parameter('output_frame_id').value

        self.pos_scale = self.get_parameter('position_cov_scale').value
        self.ori_scale = self.get_parameter('orientation_cov_scale').value
        self.min_pos_cov = self.get_parameter('min_position_cov').value
        self.min_ori_cov = self.get_parameter('min_orientation_cov').value
        
        # QoS 設定
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 訂閱 ZED Odometry
        self.sub = self.create_subscription(
            Odometry,
            fastlio_topic,
            self.odom_callback,
            10
        )

        # 發布到 MAVROS
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            mavros_topic,
            qos_profile
        )
        
        # 初始化計數器
        self.msg_count = 0

        # 輸出啟動資訊
        self.get_logger().info('=' * 50)
        self.get_logger().info('FAST-LIO2 Vision Bridge Started (Pose Only)')
        self.get_logger().info(f'  Subscribe: {fastlio_topic}')
        self.get_logger().info(f'  Publish:   {mavros_topic}')
        self.get_logger().info(f'  Frame ID:  {self.output_frame}')
        self.get_logger().info(f'  Position Cov Scale: {self.pos_scale}')
        self.get_logger().info(f'  Orientation Cov Scale: {self.ori_scale}')
        self.get_logger().info(f'  Min Position Cov: {self.min_pos_cov}')
        self.get_logger().info(f'  Min Orientation Cov: {self.min_ori_cov}')
        self.get_logger().info('=' * 50)

    def scale_covariance(self, original_cov):

        scaled_cov = list(original_cov)
        
        # 位置協方差 (對角線 0, 7, 14)
        for i in [0, 7, 14]:
            scaled_cov[i] = max(original_cov[i] * self.pos_scale, self.min_pos_cov)
        
        # 姿態協方差 (對角線 21, 28, 35)
        for i in [21, 28, 35]:
            scaled_cov[i] = max(original_cov[i] * self.ori_scale, self.min_ori_cov)
        
        # 非對角線元素也要適當縮放（保持相關性）
        for i in range(36):
            if i not in [0, 7, 14, 21, 28, 35]:
                # 根據行列位置決定用哪個縮放因子
                row = i // 6
                col = i % 6
                if row < 3 and col < 3:
                    scaled_cov[i] = original_cov[i] * self.pos_scale
                elif row >= 3 and col >= 3:
                    scaled_cov[i] = original_cov[i] * self.ori_scale
                else:
                    # 位置-姿態交叉項，用幾何平均
                    scaled_cov[i] = original_cov[i] * ((self.pos_scale * self.ori_scale) ** 0.5)
        
        return scaled_cov
        
    def odom_callback(self, msg: Odometry):
        # 建立輸出訊息
        vision_msg = PoseWithCovarianceStamped()

        # 保留 FAST-LIO2 原始時間戳（支援 EKF 延遲補償）
        vision_msg.header.stamp = msg.header.stamp
        
        # 覆蓋 Frame ID
        vision_msg.header.frame_id = self.output_frame

        # 複製位姿
        vision_msg.pose.pose = msg.pose.pose
        
        # 縮放協方差
        vision_msg.pose.covariance = self.scale_covariance(msg.pose.covariance)

        # 發布訊息
        self.pub.publish(vision_msg)

        # 定期輸出狀態
        self.msg_count += 1
        if self.msg_count % 10 == 0:
            pos = msg.pose.pose.position
            stamp = msg.header.stamp
            
            orig_cov = msg.pose.covariance[0]
            scaled_cov = vision_msg.pose.covariance[0]
            
            self.get_logger().info(
                f'[{self.msg_count}] '
                f'Time: {stamp.sec}.{stamp.nanosec // 1000000:03d} | '
                f'Pos: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) | '
                f'Cov: {orig_cov:.6f} -> {scaled_cov:.4f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FastLioVisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received shutdown signal...')
    finally:
        node.get_logger().info('Bridge stopped.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
