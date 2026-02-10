#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import paho.mqtt.client as mqtt
import subprocess
import time
import os
import signal
from datetime import datetime, timedelta

# Import message type
from campus_delivery_msgs.msg import NavTask, NavResult
from geometry_msgs.msg import Pose2D, PoseStamped
from std_msgs.msg import Header, Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class MqttToRosBridge(Node):

    def __init__(self):
        super().__init__('mqtt_bridge')

        # --- Setting parameters ---
        self.mqtt_broker = "broker.emqx.io"
        self.nav_topic = "uav/navigation/tasks"
        self.notification_topic = "warehouse/task/notification"
        self.status_topic = "warehouse/task/remaining" # Topic for reporting back to server
        self.ros_topic = "navigation_tasks" # Republished ROS topic name

        # ---  ROS Publisher initialization ---
        self.publisher_ = self.create_publisher(NavTask, self.ros_topic, 10)
        
        # --- ROS Subscriber for feedback ---
        # User specified topic: /warehouse/task/feedback
        # Note: mission_dispatcher_with_tracker_v2.py publishes to 'navigation_result'
        # To make it work as requested, we subscribe to 'navigation_result' but could remap in launch file if needed.
        # Assuming the system publishes feedback here.
        self.result_sub = self.create_subscription(
            NavResult,
            'navigation_result',
            self.result_callback,
            10
        )

        self.create_subscription(
            Bool,
            'is_landed',
            self.is_landed_subscribe,
            10
        )

        # --- Nav2 Action Client (Return Home) ---
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- Manager State ---
        self.task_queue = []
        self.is_processing = False
        self.current_task = None
        self.is_returning_home = False # Flag to avoid repeated home commands
        self.landed = False
        self.landing_process = None # Track the landing process

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

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(self.nav_topic)
        client.subscribe(self.notification_topic)
        self.get_logger().info(f"Subscribed to MQTT topics: {self.nav_topic}, {self.notification_topic}")

    def on_message(self, client, userdata, msg):
        """Receive MQTT message -> Transform -> Publishing ROS Topic or Action"""
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
            # JSON 範例中沒有 command，給予預設值或保留空字串
            ros_msg.command = task_data.get("command", "navigate") 
            ros_msg.source_timestamp = str(task_data.get("timestamp", ""))

            # 3. Waypoints / Area parsing
            # 優先檢查是否有 'area' (扁平陣列 [x,y,x,y])
            if "area" in task_data:
                area_coords = task_data.get("area", [])
                # 確保長度是偶數，並且每兩個一組取 (x, y)
                if len(area_coords) >= 2:
                    for i in range(0, len(area_coords), 2):
                        # 防止陣列長度奇數導致 index out of range
                        if i + 1 < len(area_coords):
                            wp_msg = Pose2D()
                            wp_msg.x = float(area_coords[i])
                            wp_msg.y = float(area_coords[i+1])
                            wp_msg.theta = 0.0  # Area 通常只標示範圍，沒有方向，預設為 0
                            ros_msg.waypoints.append(wp_msg)
                            
            # 相容舊有的 'waypoints' 格式 (物件列表 [{'x':1, 'y':2}])
            elif "waypoints" in task_data:
                json_waypoints = task_data.get("waypoints", [])
                for wp_data in json_waypoints:
                    wp_msg = Pose2D()
                    wp_msg.x = float(wp_data.get("x", 0.0))
                    wp_msg.y = float(wp_data.get("y", 0.0))
                    wp_msg.theta = float(wp_data.get("yaw", 0.0))
                    ros_msg.waypoints.append(wp_msg)

        # --- Queue Logic ---
            self.task_queue.append(ros_msg)
            self.get_logger().info(f"Queued Task {ros_msg.task_id}. Queue size: {len(self.task_queue)}")

        
        # Try to process immediately if idle
        # If we are returning home, we don't interrupt immediately, 
        # we wait for home return to finish (is_processing will be True)
        self.process_queue()

    def result_callback(self, msg):
        """Handle completion feedback from mission dispatcher"""
        self.get_logger().info(f"Received Result for Task {msg.task_id}: {msg.result}")
        
        # Mark current processing as done
        if msg.result == 1: # User stipulated 1 is success? Or maybe logic was inverted?
            # Sticking to user's code edit
            self.is_processing = False
        
        # Trigger next
        self.process_queue()

    def is_landed_subscribe(self, msg):
        if msg.data:
            self.precision_landing("off")

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
        self.is_returning_home = False # Ensure we clear this flag if set
        
        # Publish
        self.publisher_.publish(next_task)
        self.get_logger().info(f"Executing Task {next_task.task_id} (Remaining in queue: {len(self.task_queue)})")

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
        goal_msg.pose.pose.position.x = 0.0
        goal_msg.pose.pose.position.y = 0.0
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
        self.precision_landing("on")
        self.is_processing = False
        self.is_returning_home = False

    def handle_notification(self, data):
        notification = data.get("notification", "")
        self.get_logger().info(f"Received Notification: {notification}")
        
        if notification == "all_tasks_finished":
            self.schedule_shutdown_next_noon()
            
        elif notification == "suspend":
            self.handle_termination()

    def handle_termination(self):
        """Clear queue and report cancelled tasks"""
        # if not self.task_queue:
        #     self.get_logger().info("Termination received, but queue is empty.")
        #     return

        # Collect IDs
        cancelled_ids = [task.task_id for task in self.task_queue]
        
        # Add current task if processing
        if self.is_processing and self.current_task:
             cancelled_ids.insert(0, self.current_task.task_id)
        
        # Clear Queue
        self.task_queue.clear()
        self.is_processing = False # Reset processing flag? 
        # Note: If a task is currently executing (is_processing=True), simply clearing queue won't stop it 
        # unless we also send a Cancel to ROS. 
        # For now, we strictly follow request: report queue items.
        
        # Construct Report
        report = {
            "cancelled_task_ids": cancelled_ids,
            "timestamp": datetime.now().isoformat()
        }
        
        try:
            payload = json.dumps(report)
            self.client.publish(self.status_topic, payload)
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

            launch_cmd = "ros2 launch unico_pack precision_landing_sitl.launch.py"
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

    def schedule_shutdown_next_noon(self):
        # Calculate duration until Today 18:35
        now = datetime.now()
        target_time = now.replace(hour=10, minute=0, second=0, microsecond=0)
        
        # If already passed 18:35, schedule for tomorrow 18:35 (or handle as immediate/error? Assuming tomorrow for safety)
        if target_time < now:
            target_time += timedelta(days=1)
            
        duration_seconds = (target_time - now).total_seconds()
        
        # Set absolute wake timestamp based on current system epoch + duration
        # This avoids timezone object confusion (UTC vs Local Datetime)
        wake_timestamp = int(time.time() + duration_seconds)
        
        self.get_logger().warn(f"Shutdown sequence initiated. System will wake up in {duration_seconds/60:.2f} minutes (Target: {target_time.strftime('%H:%M:%S')})")
        
        try:
            # 1. Set RTC Wake Alarm (Requires sudo/root permissions)
            # Clear previous alarm
            cmd_clear = "echo 0 | sudo tee /sys/class/rtc/rtc0/wakealarm"
            subprocess.run(cmd_clear, shell=True, check=True)
            
            # Set new alarm
            cmd_set = f"echo {wake_timestamp} | sudo tee /sys/class/rtc/rtc0/wakealarm"
            subprocess.run(cmd_set, shell=True, check=True)
            
            self.get_logger().info("RTC Wake Alarm set successfully.")
            
            # 2. Countdown and Suspend (Sleep Mode)
            self.get_logger().warn("System suspending (sleep mode) in 5 seconds...")
            time.sleep(5)
            
            # Use 'systemctl suspend' instead of 'shutdown'
            subprocess.run("sudo systemctl suspend", shell=True, check=True)
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"System command failed: {e}. Check sudo permissions.")
        except Exception as e:
            self.get_logger().error(f"Shutdown failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MqttToRosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.loop_stop()
        node.client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()