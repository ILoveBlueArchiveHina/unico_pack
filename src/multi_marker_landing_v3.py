#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from aruco_msgs.msg import MarkerArray
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Bool
import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation as rot

class MultiMarkerLanding(Node):
    def __init__(self):
        super().__init__('multi_marker_landing')
        
        # 訂閱多個ArUco標記
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/marker_publisher/markers',
            self.markers_callback,
            10
        )
        
        # 訂閱飛控狀態
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        # 訂閱Extended State
        self.extended_state_sub = self.create_subscription(
            ExtendedState,
            '/mavros/extended_state',
            self.extended_state_callback,
            10
        )
        
        # 訂閱高度資訊 (來自 FAST-LIO Odometry)
        self.altitude_sub = self.create_subscription(
            Odometry,
            '/Odometry',
            self.altitude_callback,
            10
        )
        
        
        # 發布速度控制指令
        self.vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.is_landed_pub = self.create_publisher(
            Bool,
            '/is_landed',
            10
        )
        
        # TF 廣播器 (為 marker 發佈 transform)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # MAVROS arming service client
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.disarm_called = False  # 追蹤是否已呼叫上鎖命令
        
        # 系統狀態
        self.detected_markers = {}  # 儲存偵測到的外圈標記 {id: position}
        self.detected_inner_markers = {}  # 儲存偵測到的內圈標記 {id: position}
        self.last_marker_count = 0
        self.last_inner_marker_count = 0
        self.current_altitude = 0.0
        self.landing_center = None
        self.inner_center = None  # 內圈中心點
        self.marker_buffer = deque(maxlen=10)  # 位置緩衝，平滑濾波
        self.landing_complete = False  # 降落完成標記
        
        # 降落參數
        self.MARKER_IDS = [10, 20, 30, 40]  # 外圈4個標記ID
        self.INNER_MARKER_IDS = [100, 200, 300, 400]  # 內圈4個標記ID
        self.MIN_MARKERS_REQUIRED = 3  # 至少要看到3個標記
        self.DESCENT_SPEED = -0.1  # 下降速度 m/s
        
        # 控制增益
        self.Kp_xy = 0.3  # 水平控制增益
        self.Kp_yaw = 0.02  # Yaw 角速度控制增益
        
        # 對齊檢查參數
        self.ALIGNMENT_THRESHOLD_XY = 0.05  # 水平偏差閾值 (m)
        self.ALIGNMENT_THRESHOLD_YAW = 1.0  # Yaw 角度偏差閾值 (度)
        self.ALIGNMENT_HOLD_TIME = 0.5     # 對齊保持時間 (秒)
        self.current_time = 0.0             # 對齊時間初始化
        self.aligned_time = None            # 開始對齊的時間
        self.is_aligned = False             # 對齐狀態標記
        self.aligned_done = False
        self.last_alignment_log = False     # 對齐 log 標記
        
        # 控制迴圈 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        # 狀態變化 log flag
        self.last_no_center = False
        self.last_final_descent = False
        self.last_valid_time = self.get_clock().now() # 最後一次有效偵測時間

        self.get_logger().info('Multi-marker landing controller initialized')
        
    def markers_callback(self, msg):
        """處理多個ArUco標記資訊"""
        self.detected_markers.clear()
        self.detected_inner_markers.clear()
        
        for marker in msg.markers:
            # 處理外圈和內圈標記
            is_outer = marker.id in self.MARKER_IDS
            is_inner = marker.id in self.INNER_MARKER_IDS
            
            if is_outer or is_inner:
                # 相機座標系轉FRD機體座標系
                # 使用者要求：無人機的前方(x)對應 marker 的 y，無人機的 y 對應 marker 的 x
                # 根據原本的標記，保持 z 為 marker.pose.pose.position.y (向下為正時可調整)
                # 取得四元數並轉換為欧拉角（roll, pitch, yaw）
                qx = marker.pose.pose.orientation.x
                qy = marker.pose.pose.orientation.y
                qz = marker.pose.pose.orientation.z
                qw = marker.pose.pose.orientation.w
                
                # 使用 scipy 的 Rotation 將四元數轉成欧拉角（度數）
                rotation = rot.from_quat([qx, qy, qz, qw])
                euler_angles_rad = rotation.as_euler('xyz')  # roll, pitch, yaw (弧度)
                euler_angles_deg = np.degrees(euler_angles_rad)  # 轉換為度數
                
                position_frd = {
                    'id': marker.id,
                    'x': marker.pose.pose.position.y,    # drone forward = marker y
                    'y': marker.pose.pose.position.x,    # drone right = marker x (調整符號視需求)
                    'z': marker.pose.pose.position.z,    # drone down = marker z
                    'qx': qx,
                    'qy': qy,
                    'qz': qz,
                    'qw': qw,
                    'roll': euler_angles_deg[0],   # X軸旋轉（度數）
                    'pitch': euler_angles_deg[1],  # Y軸旋轉（度數）
                    'yaw': euler_angles_deg[2],    # Z軸旋轉（度數）
                }
                
                # 分別儲存到對應的字典
                if is_outer:
                    self.detected_markers[marker.id] = position_frd
                if is_inner:
                    self.detected_inner_markers[marker.id] = position_frd
        
        # 計算中心點邏輯：優先使用外圈，外圈不足時使用內圈
        has_outer = len(self.detected_markers) >= self.MIN_MARKERS_REQUIRED
        has_inner = len(self.detected_inner_markers) > 0
        
        if has_outer:
            # 外圈標記足夠 -> 只使用外圈中心點
            self.landing_center = self.calculate_geometric_center()
            self.inner_center = None
            self.marker_buffer.append(self.landing_center)
            
            # 發佈 TF 框架
            self.publish_marker_transforms()
            
            # Log資訊
            if len(self.detected_markers) != self.last_marker_count:
                self.get_logger().info(
                    f'Detected {len(self.detected_markers)} outer markers, using outer center'
                )
                self.last_marker_count = len(self.detected_markers)
            
            # 更新有效時間
            self.last_valid_time = self.get_clock().now()
                
        elif has_inner:
            # 外圈不足，但有內圈標記 -> 使用內圈中心點
            self.inner_center = self.calculate_inner_center()
            self.landing_center = self.inner_center
            self.marker_buffer.append(self.landing_center)
            
            # 發佈 TF 框架
            self.publish_marker_transforms()
            
            # Log 資訊
            if self.last_marker_count >= self.MIN_MARKERS_REQUIRED:
                self.get_logger().info(
                    f'Outer markers lost, switching to inner markers only '
                    f'({len(self.detected_inner_markers)} detected)'
                )
            self.last_marker_count = len(self.detected_markers)
            self.last_inner_marker_count = len(self.detected_inner_markers)
            
            # 更新有效時間
            self.last_valid_time = self.get_clock().now()
        else:
            # 外圈和內圈都不足
            self.get_logger().warn(
                f'Insufficient markers: outer={len(self.detected_markers)}, '
                f'inner={len(self.detected_inner_markers)}'
            )
    
    def calculate_geometric_center(self):
        """計算外圈多個標記的幾何中心點"""
        if not self.detected_markers:
            return None
        
        positions = list(self.detected_markers.values())
        
        # 計算平均位置
        avg_x = np.mean([p['x'] for p in positions])
        avg_y = np.mean([p['y'] for p in positions])
        avg_z = np.mean([p['z'] for p in positions])
        avg_yaw = np.mean([p['yaw'] for p in positions])
        
        return {
            'x': avg_x,
            'y': avg_y,
            'z': avg_z,
            'yaw': avg_yaw,
            'num_markers': len(positions)
        }
    
    def calculate_inner_center(self):
        """計算內圈標記的幾何中心點"""
        if not self.detected_inner_markers:
            return None
        
        positions = list(self.detected_inner_markers.values())
        
        # 計算平均位置
        avg_x = np.mean([p['x'] for p in positions])
        avg_y = np.mean([p['y'] for p in positions])
        avg_z = np.mean([p['z'] for p in positions])
        avg_yaw = np.mean([p['yaw'] for p in positions])
        
        return {
            'x': avg_x,
            'y': avg_y,
            'z': avg_z,
            'yaw': avg_yaw,
            'num_markers': len(positions)
        }
    
    def get_smoothed_center(self):
        """使用滑動平均平滑中心點位置"""
        if len(self.marker_buffer) == 0:
            return None
        
        avg_x = np.mean([c['x'] for c in self.marker_buffer])
        avg_y = np.mean([c['y'] for c in self.marker_buffer])
        avg_z = np.mean([c['z'] for c in self.marker_buffer])
        avg_yaw = np.mean([c['yaw'] for c in self.marker_buffer])
        
        return {'x': avg_x, 'y': avg_y, 'z': avg_z, 'yaw': avg_yaw}
    
    
    def altitude_callback(self, msg):
        """更新當前高度 (來自 FAST-LIO Odometry)"""
        self.current_altitude = msg.pose.pose.position.z
            
    def state_callback(self, msg):
        """更新飛控狀態"""
        self.current_state = msg
    
    def extended_state_callback(self, msg):
        """更新 Extended State"""
        self.current_extended_state = msg
    
    
    def call_disarm(self):
        """呼叫 MAVROS arming service 執行上鎖命令（暫時未使用）"""
        if self.disarm_called:
            return
        
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('MAVROS arming service 不可用')
            return
        
        request = CommandBool.Request()
        request.value = False  # False = 上鎖 (disarm)
        
        self.get_logger().info('� 已對齊 ArUco 標記，呼叫 MAVROS 執行上鎖命令...')
        
        future = self.arming_client.call_async(request)
        future.add_done_callback(self.disarm_callback)
        self.disarm_called = True
    
    def disarm_callback(self, future):
        """處理 arming service 回應"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('✅ 成功執行上鎖命令')
                msg=Bool()
                msg.data=True
                self.is_landed_pub(msg)  # 和 manager管理節點說已經降落完成，可以關閉精準降落程序
            else:
                self.get_logger().error('❌ 上鎖命令執行失敗')
        except Exception as e:
            self.get_logger().error(f'呼叫 arming service 時發生錯誤: {e}')
    
    def publish_marker_transforms(self):
        """發佈 TF 框架：以 4 個 marker 的幾何中心為原點"""
        if not self.landing_center or len(self.detected_markers) < 2:
            return
        
        # 時間戳記
        now = self.get_clock().now()
        
        # 1. 發佈 aruco_origin 框架（几何中心位置）
        # 這個框架代表 4 個 marker 的幾何中心，作為參考原點
        origin_transform = TransformStamped()
        origin_transform.header.stamp = now.to_msg()
        origin_transform.header.frame_id = 'camera_link'  # 來自相機座標系
        origin_transform.child_frame_id = 'aruco_origin'   # 幾何中心的新座標系
        
        # 中心位置
        origin_transform.transform.translation.x = self.landing_center['x']
        origin_transform.transform.translation.y = self.landing_center['y']
        origin_transform.transform.translation.z = self.landing_center['z']
        
        # 中心旋轉（從平均 yaw 角度）
        yaw_rad = np.radians(self.landing_center['yaw'])
        # 從 yaw 轉換為四元數
        q = rot.from_euler('z', yaw_rad).as_quat()  # [qx, qy, qz, qw]
        origin_transform.transform.rotation.x = q[0]
        origin_transform.transform.rotation.y = q[1]
        origin_transform.transform.rotation.z = q[2]
        origin_transform.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(origin_transform)
        
        # 2. 為每個 marker 發佈相對於 aruco_origin 的 TF
        for marker_id, marker_data in self.detected_markers.items():
            marker_transform = TransformStamped()
            marker_transform.header.stamp = now.to_msg()
            marker_transform.header.frame_id = 'aruco_origin'        # 父框架是几何中心
            marker_transform.child_frame_id = f'marker_{marker_id}'  # 子框架是各個 marker
            
            # 相對於幾何中心的位置
            marker_transform.transform.translation.x = marker_data['x'] - self.landing_center['x']
            marker_transform.transform.translation.y = marker_data['y'] - self.landing_center['y']
            marker_transform.transform.translation.z = marker_data['z'] - self.landing_center['z']
            
            # marker 本身的旋轉
            marker_transform.transform.rotation.x = marker_data['qx']
            marker_transform.transform.rotation.y = marker_data['qy']
            marker_transform.transform.rotation.z = marker_data['qz']
            marker_transform.transform.rotation.w = marker_data['qw']
            
            self.tf_broadcaster.sendTransform(marker_transform)
    
    def check_alignment(self, center):
        """檢查是否對齊 marker"""
        # 計算水平偏差
        xy_offset = np.sqrt(center['x']**2 + center['y']**2)
        # 計算 yaw 偏差（絕對值）
        yaw_offset = abs(center['yaw'])
        
        current_time = time.time()
        
        # 判斷是否在對齊範圍內
        if xy_offset < self.ALIGNMENT_THRESHOLD_XY and yaw_offset < self.ALIGNMENT_THRESHOLD_YAW:
            # 首次對齊，記錄時間
            if self.aligned_time is None:
                self.aligned_time = current_time
                self.get_logger().info(f'開始對齊 (偏差: xy={xy_offset:.3f}m, yaw={yaw_offset:.1f}°)')
            
            # 檢查是否持續對齊足夠時間
            if current_time - self.aligned_time >= self.ALIGNMENT_HOLD_TIME:
                self.aligned_done = True
                return True
        else:
            # 未對齊，重置時間
            self.aligned_time = None
        return False

    def control_loop(self):
        """主控制迴圈"""
        # --- 安全檢查：訊號超時保護 ---
        # 如果超過 1 秒沒有偵測到有效標記，視為訊號遺失
        now = self.get_clock().now()
        dt_lost = (now - self.last_valid_time).nanoseconds / 1e9
        
        if dt_lost > 1:
            if not self.last_no_center:
                self.get_logger().warn(f'🚨 標記訊號遺失 ({dt_lost:.2f}s)！停止移動')
                self.last_no_center = True
            
            # 發布零速度懸停
            self.vel_pub.publish(Twist())
            self.last_final_descent = False
            self.marker_buffer.clear() # 清除舊數據
            return

        # 獲取平滑後的中心點
        center = self.get_smoothed_center()
        
        if center is None:
            if not self.last_no_center:
                self.get_logger().warn('No valid landing center, hovering...')
                self.last_no_center = True
            self.last_final_descent = False
            return
        else:
            self.last_no_center = False
        
        # ===== 對齊檢查 =====
        self.is_aligned = self.check_alignment(center)
        
        # 計算水平偏差
        xy_offset = np.sqrt(center['x']**2 + center['y']**2)
        xy_aligned = xy_offset < self.ALIGNMENT_THRESHOLD_XY
        
        # 創建速度控制指令 (使用 Twist)
        vel_cmd = Twist()

        # 如果對齊成功且尚未呼叫 LAND 模式，則呼叫
        if self.aligned_done:
            vel_cmd.linear.z = self.DESCENT_SPEED

        # 計算水平方向誤差修正速度（永遠執行）
        vel_cmd.linear.x = -self.Kp_xy * center['x']
        vel_cmd.linear.y = -self.Kp_xy * center['y']
        
        # Yaw 角度對齊控制（只有在 XY 對齊後才進行旋轉）
        # 這樣可以避免旋轉時產生額外的位置偏移
        if xy_aligned:
            vel_cmd.angular.z = -self.Kp_yaw * center['yaw']
        else:
            vel_cmd.angular.z = 0.0  # XY 未對齊時不旋轉

        # 速度限制
        max_vel_xy = 0.1
        max_vel_z = 0.1
        max_ang_z = 0.1  # 最大角速度 rad/s

        # 防護 NaN/inf
        for axis in ('x', 'y', 'z'):
            v = getattr(vel_cmd.linear, axis)
            if not np.isfinite(v):
                setattr(vel_cmd.linear, axis, 0.0)
        
        # 防護角速度 NaN/inf
        ang_z = vel_cmd.angular.z
        if not np.isfinite(ang_z):
            vel_cmd.angular.z = 0.0

        vel_cmd.linear.x = float(np.clip(vel_cmd.linear.x, -max_vel_xy, max_vel_xy))
        vel_cmd.linear.y = float(np.clip(vel_cmd.linear.y, -max_vel_xy, max_vel_xy))
        vel_cmd.linear.z = float(np.clip(vel_cmd.linear.z, -max_vel_z, max_vel_z))
        vel_cmd.angular.z = float(np.clip(vel_cmd.angular.z, -max_ang_z, max_ang_z))
        
        # 發布速度指令
        self.vel_pub.publish(vel_cmd)

        # 判斷降落成功 - 使用 Extended State
        if hasattr(self, 'current_extended_state'):
            if self.current_extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                # 發布零速度
                zero_cmd = Twist()
                self.vel_pub.publish(zero_cmd)
                self.get_logger().info('Extended State 確認已著陸 (ON_GROUND)')
                
                # 嘗試上鎖確保安全
                if not self.disarm_called:
                    self.call_disarm()
                
                self.landing_complete = True
                return
            


def main(args=None):
    rclpy.init(args=args)
    node = MultiMarkerLanding()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # 檢查降落是否完成
            if node.landing_complete:
                node.get_logger().info('主程式檢測到降落完成，準備關閉')
                break
    except KeyboardInterrupt:
        node.get_logger().info('收到中斷信號，正在關閉...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
