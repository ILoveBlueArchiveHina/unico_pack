#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget, State

class CopterVelocityControlFixed(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        
        # 發布速度命令
        self.pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            10
        )
        
        # 訂閱cmd_vel
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        
        # 訂閱狀態
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        # **關鍵1：必須持續發送（20Hz）**
        self.timer = self.create_timer(0.05, self.send_velocity)
        
        self.current_state = State()
        
        # 儲存目標速度
        self.target_velocity = Twist()
        
        self.get_logger().info("速度控制節點已啟動（修正版）")
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def cmd_callback(self, msg):
        """接收cmd_vel並儲存（不直接發送）"""
        self.target_velocity = msg
        
    def send_velocity(self):
        """由timer持續調用，發送速度命令"""
        # 只在已連接且在GUIDED模式時發送
        if not self.current_state.connected:
            return
        if self.current_state.mode != "GUIDED":
            return
            
        cmd = PositionTarget()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        
        # 選擇座標系
        # 方案A：機體座標系（機頭方向）
        cmd.coordinate_frame = PositionTarget.FRAME_BODY_NED
        
        # 方案B：全局座標系（固定方向）- 取消註釋來使用
        # cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # **關鍵2：修正type_mask，允許yaw_rate控制**
        # 僅忽略：位置(PX,PY,PZ) + 加速度(AFX,AFY,AFZ) + 偏航角(YAW)
        # 不忽略：速度(VX,VY,VZ) + 偏航率(YAW_RATE)
        cmd.type_mask = (
            PositionTarget.IGNORE_PX |       # 1
            PositionTarget.IGNORE_PY |       # 2  
            PositionTarget.IGNORE_PZ |       # 4
            PositionTarget.IGNORE_AFX |      # 64
            PositionTarget.IGNORE_AFY |      # 128
            PositionTarget.IGNORE_AFZ |      # 256
            PositionTarget.IGNORE_YAW        # 1024
        )  # = 1479
        
        # 設置速度
        cmd.velocity.x = self.target_velocity.linear.x
        cmd.velocity.y = self.target_velocity.linear.y
        cmd.velocity.z = self.target_velocity.linear.z
        cmd.yaw_rate = self.target_velocity.angular.z
        
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CopterVelocityControlFixed()
    
    node.get_logger().info("節點運行中...")
    node.get_logger().info("請先：1) 切換GUIDED模式  2) 解鎖  3) 起飛")
    node.get_logger().info("然後發送cmd_vel命令")
    
    rclpy.spin(node)

if __name__ == '__main__':
    main()
