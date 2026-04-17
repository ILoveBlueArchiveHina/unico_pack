#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import paho.mqtt.client as mqtt
import subprocess
import os
import signal
from datetime import datetime
from rclpy.qos import qos_profile_sensor_data

# Import message type
from campus_delivery_msgs.msg import NavTask, NavResult
from geometry_msgs.msg import Pose2D, PoseStamped, TransformStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header, Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import StaticTransformBroadcaster
from mavros_msgs.srv import CommandTOL,CommandBool, SetMode
from mavros_msgs import ExtendedState, State

class MqttToRosBridge(Node):

    def __init__(self):
        super().__init__('mqtt_bridge')

        # --- Setting parameters ---
        self.declare_parameter("home_pose_x",0.0)
        self.declare_parameter("home_pose_y",0.0)
        self.declare_parameter("rosbag_folder_path","/home/uni-co-jetson/rosbag")
        self.declare_parameter("mqtt_broker","192.168.166.83")

        self.home_pose_x = self.get_parameter("home_pose_x").value
        self.home_pose_y = self.get_parameter("home_pose_y").value
        self.rosbag_folder_path = self.get_parameter("rosbag_folder_path").value
        self.mqtt_broker = self.get_parameter("mqtt_broker").value

        self.nav_topic = "warehouse/task/request"
        self.notification_topic = "warehouse/task/notification"
        self.cancelled_topic = "warehouse/task/cancelled" # Topic for reporting back to server
        self.status_topic = "warehouse/task/status"
        self.feedback_topic = "warehouse/task/feedback"
        self.ros_topic = "navigation_tasks" # Republished ROS topic name

        # ---  ROS Publisher initialization ---
        self.publisher_ = self.create_publisher(NavTask, self.ros_topic, 1)
        self.ready_to_nav_pub = self.create_publisher(Bool, 'ready_to_nav', 1)
        
        # --- ROS Subscriber for feedback ---
        # User specified topic: /warehouse/task/feedback
        # Note: mission_dispatcher_with_tracker_v2.py publishes to 'navigation_result'
        # To make it work as requested, we subscribe to 'navigation_result' but could remap in launch file if needed.
        # Assuming the system publishes feedback here.
        self.result_sub = self.create_subscription(
            NavResult,
            'navigation_result',
            self.result_callback,
            1
        )

        self.mavros_extended_state_sub = self.create_subscription(
            ExtendedState,
            '/mavros/extended_state',
            self.mavros_extended_state_callback,
            1
        )

        self.mavros_state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.mavros_state_callback,
            1
        )

        self.create_subscription(
            Bool,
            'is_landed',
            self.is_landed_subscribe,
            1
        )

        self.create_subscription(
            Bool,
            'ready_to_record_rosbag',
            self.ready_to_record_rosbag_signal_sub,
            1
        )

        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Pose2D,
            '/initial_tf',
            self.initial_tf_callback,
            1
        )


        self.static_tf = StaticTransformBroadcaster(self)
        self.create_timer(5, self.status_report)

        # --- Nav2 Action Client (Return Home) ---
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.mavros_takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.mavros_arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mavros_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.req = CommandTOL.Request()


        # --- Manager State ---
        self.task_queue = []
        self.is_processing = False
        self.current_task = None
        self.is_returning_home = False # Flag to avoid repeated home commands
        self.landed = False
        self.landing_process = None # Track the landing process
        self.rosbag_process = None # Track the rosbag recording process
        self.rosbag_folder_name = "testing"
        self.failed_faces = []
        self.battery_percentage = None
        self.ready = False  # Startup protection: ignore messages until ready
        self.mavros_state = State()
        self.mavros_extended_state = ExtendedState()
        self.ready_to_nav = False
        self.current_status = 'offline'

        # --- MQTT Client initialization ---
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        try:
            self.client.connect(self.mqtt_broker, 1883, 60)
            self.client.loop_start() # Run MQTT in background
            self.get_logger().info(f"Connected to MQTT Broker. Bridge started.")
        except Exception as e:
            self.get_logger().error(f"Cannot connect to MQTT: {e}")

        # Startup protection: wait 3 seconds before accepting tasks
        self.create_timer(3.0, self._set_ready) 
    
    def mavros_extended_state_callback(self, msg):
        self.mavros_extended_state = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def initial_tf_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_init'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = msg.theta
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)  # 只需呼叫一次！

    def _set_ready(self):
        """Called once after startup delay to begin accepting tasks."""
        if not self.ready:
            self.ready = True
            self.get_logger().info("Manager is now ready to accept tasks.")
            self.current_status = 'idle'

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(self.nav_topic)
        client.subscribe(self.notification_topic)
        self.get_logger().info(f"Subscribed to MQTT topics: {self.nav_topic}, {self.notification_topic}")

    def on_message(self, client, userdata, msg):
        """Receive MQTT message -> Transform -> Publishing ROS Topic or Action"""
        if not self.ready:
            self.get_logger().warn(f"Ignoring message on {msg.topic} (not ready yet, may be retained message)")
            return

        try:
            payload_str = msg.payload.decode('utf-8')
            data = json.loads(payload_str)
            
            if msg.topic == self.nav_topic:
                self.handle_navigation_task(data)
            elif msg.topic == self.notification_topic:
                self.handle_notification(data)

        except json.JSONDecodeError:
            self.get_logger().error("Received invalid JSON format")
        except Exception as e:
            self.get_logger().error(f"Error converting message: {e}")

    def handle_navigation_task(self, data):
        task_list = data if isinstance(data, list) else [data]

        for task_data in task_list:
            # --- Create ROS messages (Must be created inside the loop) ---
            ros_msg = NavTask()
            
            # 1. Header
            ros_msg.header = Header()
            ros_msg.header.stamp = self.get_clock().now().to_msg()
            ros_msg.header.frame_id = "map" # Reference tf frame

            # 2. Basic information
            ros_msg.task_id = task_data.get("task_id", "")
            ros_msg.rosbag_path = task_data.get("rosbag_path", "") 
            ros_msg.source_timestamp = str(task_data.get("timestamp", ""))

            # 3. Area parsing [x1,y1,x2,y2,x3,y3,x4,y4]
            if "area" in task_data:
                area_coords = task_data.get("area", [])
                # 確保長度是偶數，並且每兩個一組取 (x, y)
                if len(area_coords) >= 2:
                    for i in range(0, len(area_coords), 2):
                        # 防止陣列長度奇數導致 index out of range
                        if i + 1 < len(area_coords):
                            wp_msg = Pose2D()

                            # 座標轉換
                            x_raw = float(area_coords[i])
                            y_raw = float(area_coords[i+1])
                            wp_msg.x = round(((x_raw-100)*0.866 + 4)*0.05, 2)
                            wp_msg.y = round(-((y_raw-100)*0.89125 + 10) * 0.05, 2)
                            wp_msg.theta = 0.0  # no specific direction
                            ros_msg.waypoints.append(wp_msg)
                            

            # --- Queue Logic ---
            self.task_queue.append(ros_msg)
            self.get_logger().info(f"Queued Task {ros_msg.task_id}. Queue size: {len(self.task_queue)}")

        
        # Try to process immediately if idle
        # If we are returning home, we don't interrupt immediately, 
        # we wait for home return to finish (is_processing will be True)
        self.process_queue()

    def process_queue(self):
        """Check queue and publish next task if idle"""
        if self.is_processing:
            return
            
        if not self.task_queue:
            self.get_logger().info("Task Queue is empty, return home.")
            self.return_home()
            return
            
        # Pop next task
        next_task = self.task_queue.pop(0)
        self.is_processing = True
        self.current_task = next_task
        self.rosbag_folder_name = self.current_task.rosbag_path
        self.is_returning_home = False # Ensure we clear this flag if set
        
        # Publish
        self.publisher_.publish(next_task)

        self.get_logger().info(f"Executing Task {next_task.task_id} (Remaining in queue: {len(self.task_queue)})")

        self.send_feedback(
            task_id=self.current_task.task_id,
            result=0,
            failed_faces=[]
        )

    def send_takeoff(self, altitude):
        # 設定參數 (對應 CLI 的 {altitude: 1})
        self.req.altitude = float(altitude)
        # 其他常用參數預設值 (通常設為 0)
        self.req.min_pitch = 0.0
        self.req.yaw = 0.0
        self.req.latitude = 0.0
        self.req.longitude = 0.0
        
        # 非同步呼叫
        self.future = self.mavros_takeoff_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def result_callback(self, msg):
        """Handle completion feedback from mission dispatcher"""
        self.get_logger().info(f"Received Result for Task {msg.task_id}: {msg.result}")
        
        # Mark current processing as done
        

        self.send_feedback(
            task_id=msg.task_id,
            result=msg.result,
            failed_faces=list(msg.failed_faces) if msg.failed_faces is not None else []
        )

        self.record_rosbag("off")
        # Trigger next
        if msg.result in (1, 2):
            self.is_processing = False
            self.process_queue()

    def battery_callback(self, msg):
        self.battery_percentage = msg.percentage

    def is_landed_subscribe(self, msg):
        if msg.data:
            self.precision_landing("off")

    def ready_to_record_rosbag_signal_sub(self, msg):
        if msg.data:
            self.record_rosbag("on",path=self.rosbag_folder_name)
            self.get_logger().info(f"start to record rosbag(debug)")

    def status_report(self):
        task_status = {
        "status": self.current_status,
        "battery": self.battery_percentage
        }
            
        payload = json.dumps(task_status)
        self.client.publish(self.status_topic, payload)


    def send_feedback(self, task_id, result, failed_faces):
        try:
            feedback_msgs = {
                "task_id": task_id,
                "result": result,
                "failed_faces": failed_faces,
                "timestamp": datetime.now().isoformat()
            }
            payload = json.dumps(feedback_msgs)
            self.client.publish(self.feedback_topic, payload)
        except Exception as e:
            self.get_logger().error(f"Failed to publish feedback report: {e}")

    

    def return_home(self):
        """Send specific Nav2 action to go to (0,0)"""
        if self.is_returning_home:
            return # Already going home

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 Action Server not available! Cannot return home.")
            return

        self.get_logger().info("Initiating Return to Home (0, 0)...")
        self.is_processing = True # Occupy the processor
        self.is_returning_home = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        # Home Position (0,-20)
        goal_msg.pose.pose.position.x = self.home_pose_x
        goal_msg.pose.pose.position.y = self.home_pose_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info("Sending Home Goal...")
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.home_response_callback)

    def home_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Return to Home rejected!")
            self.is_processing = False
            self.is_returning_home = False
            return

        self.get_logger().info("Return to Home accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.home_result_callback)

    def home_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Arrived at Home (or action finished).")

        # self.precision_landing("on")
        self.is_processing = False
        self.is_returning_home = False

    def handle_notification(self, data):
        notification = data.get("notification", "")
        self.get_logger().info(f"Received Notification: {notification}")
        
        # if notification == "all_tasks_finished":
        #     self.schedule_shutdown_next_noon()
            
        if notification == "suspend":
            self.handle_termination()

    def handle_termination(self):
        """Clear queue and report cancelled tasks"""
        # Stop recording rosbag
        self.record_rosbag("off")

        # Collect IDs
        cancelled_ids = [task.task_id for task in self.task_queue]
        
        # Add current task if processing
        if self.is_processing and self.current_task:
             cancelled_ids.insert(0, self.current_task.task_id)
        
        # Clear Queue
        self.task_queue.clear()
        self.is_processing = False # Reset processing flag? 
        # Note: If a task is currently executing (is_processing=True), simply clearing queue won't stop it 
        # unless we also send a Cancel to ROS. a
        # For now, we strictly follow request: report queue items.
        
        # Construct Report
        report = {
            "cancelled_task_ids": cancelled_ids,
            "timestamp": datetime.now().isoformat()
        }
        
        try:
            payload = json.dumps(report)
            self.client.publish(self.cancelled_topic, payload)
            self.get_logger().warn(f"Mission Terminated. Report sent: {payload}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish cancellation report: {e}")

        self.return_home()
        
    def precision_landing(self, action="on"):
        """Execute precision landing launch file with On/Off control"""
        
        if action == "on":
            # Check if already running
            if self.landing_process and self.landing_process.poll() is None:
                self.get_logger().warn("Precision Landing is already running. Ignoring start request.")
                return

            launch_cmd = "ros2 launch unico_pack precision_landing.launch.py"
            self.landed = True # Set flag (logic depends on usage)
            self.get_logger().info(f"Starting Precision Landing: {launch_cmd}")
            try:
                # Run in background and track process. Use setsid to create a new process group.
                self.landing_process = subprocess.Popen(launch_cmd, shell=True, preexec_fn=os.setsid)
            except Exception as e:
                self.get_logger().error(f"Failed to launch precision landing: {e}")

        elif action == "off":
            if self.landing_process and self.landing_process.poll() is None:
                self.get_logger().info("Stopping Precision Landing process...")
                try:
                    # Kill the whole process group
                    os.killpg(os.getpgid(self.landing_process.pid), signal.SIGINT)
                    self.landing_process.wait(timeout=2)
                    self.get_logger().info("Precision Landing stopped.")
                except subprocess.TimeoutExpired:
                     os.killpg(os.getpgid(self.landing_process.pid), signal.SIGKILL)
                     self.get_logger().warn("Precision Landing killed forcefully.")
                except Exception as e:
                    self.get_logger().error(f"Failed to stop precision landing: {e}")
            else:
                 self.get_logger().info("Precision Landing is not running.")
        
        else:
            self.get_logger().warn(f"Unknown action for precision_landing: {action}")

    def record_rosbag(self, action="on", path="defult"):
        """Record rosbag with On/Off control
        Usage:
            self.record_rosbag("on")   # Start recording
            self.record_rosbag("off")  # Stop recording and save
        """

        if action == "on":
            # Check if already recording
            if self.rosbag_process and self.rosbag_process.poll() is None:
                self.get_logger().warn("Rosbag is already recording. Ignoring start request.")
                return

            # Generate timestamped output directory
            output_dir = f"{self.rosbag_folder_path}/{path}"

            # Record topics
            topics = [
                "/zed/zed_node/rgb/color/rect/camera_info",
                "/zed/zed_node/rgb/color/rect/image/compressed",
                "/zed/zed_node/depth/depth_registered",
                "/zed/zed_node/depth/camera_info",
                "/zed/zed_node/imu/data",
                "/tf",
                "/tf_static"
            ]
            topics_str = " ".join(topics)

            record_cmd = f"ros2 bag record -o {output_dir} {topics_str}"
            self.get_logger().info(f"Starting Rosbag Recording: {record_cmd}")
            try:
                self.rosbag_process = subprocess.Popen(
                    record_cmd, shell=True, preexec_fn=os.setsid
                )
            except Exception as e:
                self.get_logger().error(f"Failed to start rosbag recording: {e}")

        elif action == "off":
            if self.rosbag_process and self.rosbag_process.poll() is None:
                self.get_logger().info("Stopping Rosbag Recording...")
                try:
                    # SIGINT lets ros2 bag flush and save properly
                    os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGINT)
                    self.rosbag_process.wait(timeout=5)
                    self.get_logger().info("Rosbag Recording stopped and saved.")
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self.rosbag_process.pid), signal.SIGKILL)
                    self.get_logger().warn("Rosbag Recording killed forcefully.")
                except Exception as e:
                    self.get_logger().error(f"Failed to stop rosbag recording: {e}")
            else:
                self.get_logger().info("Rosbag Recording is not running.")

        else:
            self.get_logger().warn(f"Unknown action for record_rosbag: {action}")

    def cleanup_subprocesses(self):
        """Terminate all child subprocesses to prevent orphan processes."""
        for name, proc in [("Precision Landing", self.landing_process),
                           ("Rosbag Recording", self.rosbag_process)]:
            if proc and proc.poll() is None:
                self.get_logger().info(f"Cleaning up {name} (PID: {proc.pid})...")
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                    proc.wait(timeout=3)
                    self.get_logger().info(f"{name} stopped.")
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    self.get_logger().warn(f"{name} killed forcefully.")
                except Exception as e:
                    self.get_logger().error(f"Failed to clean up {name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttToRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup_subprocesses()
        node.client.loop_stop()
        node.client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()