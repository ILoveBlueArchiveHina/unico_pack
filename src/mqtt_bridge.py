import rclpy
from rclpy.node import Node
import json
import paho.mqtt.client as mqtt

# Import message type
from campus_delivery_msgs.msg import NavTask
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header

class MqttToRosBridge(Node):

    def __init__(self):
        super().__init__('mqtt_bridge')

        # --- Setting parameters ---
        self.mqtt_broker = "127.0.0.1"
        self.mqtt_topic = "uav/navigation/tasks"
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
        client.subscribe(self.mqtt_topic)
        self.get_logger().info(f"Subscribed to MQTT topic: {self.mqtt_topic}")

    def on_message(self, client, userdata, msg):
        """Receive MQTT message -> Transform -> Publishing ROS Topic"""
        try:
            payload_str = msg.payload.decode('utf-8')
            data = json.loads(payload_str)
            
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
                wp_msg.theta = float(wp_data.get("theta", 0.0))
                ros_msg.waypoints.append(wp_msg)

            # --- Publishing messages ---
            self.publisher_.publish(ros_msg)
            self.get_logger().info(f"Forwarded Task {ros_msg.task_id} with {len(ros_msg.waypoints)} waypoints.")

        except json.JSONDecodeError:
            self.get_logger().error("Received invalid JSON format")
        except Exception as e:
            self.get_logger().error(f"Error converting message: {e}")

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