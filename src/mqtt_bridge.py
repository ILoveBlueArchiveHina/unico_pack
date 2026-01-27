#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import paho.mqtt.client as mqtt
import subprocess
import time
from datetime import datetime, timedelta

# Import message type
from campus_delivery_msgs.msg import NavTask
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header

class MqttToRosBridge(Node):

    def __init__(self):
        super().__init__('mqtt_bridge')

        # --- Setting parameters ---
        self.mqtt_broker = "broker.emqx.io"
        self.nav_topic = "uav/navigation/tasks"
        self.notification_topic = "warehouse/task/notification"
        self.ros_topic = "navigation_tasks" # Republished ROS topic name

        # ---  ROS Publisher initialization ---
        self.publisher_ = self.create_publisher(NavTask, self.ros_topic, 10)

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
        # --- Create ROS messages ---
        ros_msg = NavTask()
        
        # 1. Header
        ros_msg.header = Header()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.header.frame_id = "map" # Refference tf frame

        # 2. basic information
        ros_msg.task_id = data.get("task_id", "")
        ros_msg.command = data.get("command", "")
        ros_msg.source_timestamp = str(data.get("timestamp", ""))

        # 3. Waypoints list
        json_waypoints = data.get("waypoints", [])
        for wp_data in json_waypoints:
            wp_msg = Pose2D()
            # use float() type
            wp_msg.x = float(wp_data.get("x", 0.0))
            wp_msg.y = float(wp_data.get("y", 0.0))
            wp_msg.theta = float(wp_data.get("yaw", 0.0))
            ros_msg.waypoints.append(wp_msg)

        # --- Publishing messages ---
        self.publisher_.publish(ros_msg)
        self.get_logger().info(f"Forwarded Task {ros_msg.task_id} with {len(ros_msg.waypoints)} waypoints.")

    def handle_notification(self, data):
        notification = data.get("notification", "")
        self.get_logger().info(f"Received Notification: {notification}")
        
        if notification == "all_tasks_finished":
            self.schedule_shutdown_next_noon()

    def schedule_shutdown_next_noon(self):
        # Calculate wake time: Next Day 12:00 PM
        now = datetime.now()
        tomorrow = now + timedelta(days=1)
        # Set to Noon (12:00:00)
        # resume_time = datetime(tomorrow.year, tomorrow.month, tomorrow.day, 12, 0, 0)
        resume_time = datetime(now.year, now.month, now.day, 17, 30, 0)
        # resume_time = datetime.now() + timedelta(minutes=5)
        wake_timestamp = int(resume_time.timestamp())
        
        self.get_logger().warn(f"Shutdown sequence initiated. System will wake up at {resume_time} (Timestamp: {wake_timestamp})")
        
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