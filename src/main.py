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

from campus_delivery_msgs.msg import NavTask, NavResult
from geometry_msgs.msg import Pose2D, PoseStamped, TransformStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header, Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import StaticTransformBroadcaster
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from mavros_msgs.msg import ExtendedState, State 
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class MqttToRosBridge(Node):

    def __init__(self):
        super().__init__('mqtt_bridge')

        self.declare_parameter("home_pose_x", 0.0)
        self.declare_parameter("home_pose_y", 0.0)
        self.declare_parameter("rosbag_folder_path", "/home/uni-co-jetson/rosbag")
        self.declare_parameter("mqtt_broker", "192.168.166.83")

        self.home_pose_x = self.get_parameter("home_pose_x").value
        self.home_pose_y = self.get_parameter("home_pose_y").value
        self.rosbag_folder_path = self.get_parameter("rosbag_folder_path").value
        self.mqtt_broker = self.get_parameter("mqtt_broker").value

        self.nav_topic = "warehouse/task/request"
        self.notification_topic = "warehouse/task/notification"
        self.cancelled_topic = "warehouse/task/cancelled"
        self.status_topic = "warehouse/task/status"
        self.feedback_topic = "warehouse/task/feedback"
        self.ros_topic = "navigation_tasks"

        self.publisher_ = self.create_publisher(NavTask, self.ros_topic, 1)
        self.start_vel_bridge_signal_pub = self.create_publisher(Bool, 'start_vel_bridging', 1)
        self._pl_change_state_cli = self.create_client(
            ChangeState, '/precision_landing_node/change_state')
        self._bag_change_state_cli = self.create_client(
            ChangeState, '/rosbag_node/change_state')
        self._bag_set_params_cli = self.create_client(
            SetParameters, '/rosbag_node/set_parameters')

        self.result_sub = self.create_subscription(
            NavResult, 'navigation_result', self.result_callback, 1)

        self.create_subscription(
            ExtendedState, '/mavros/extended_state',
            self.mavros_extended_state_callback, 1)

        self.create_subscription(
            State, '/mavros/state',
            self.mavros_state_callback, 1)

        self.create_subscription(
            Bool, 'is_landed', self.is_landed_subscribe, 1)

        self.create_subscription(
            Bool, 'ready_to_record_rosbag',
            self.ready_to_record_rosbag_signal_sub, 1)

        self.create_subscription(
            BatteryState, '/mavros/battery',
            self.battery_callback, qos_profile_sensor_data)

        self.create_subscription(
            Pose2D, '/initial_tf', self.initial_tf_callback, 1)

        self.static_tf = StaticTransformBroadcaster(self)
        self.create_timer(5, self.status_report)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.mavros_takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.mavros_arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mavros_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        self.task_queue = []
        self.is_processing = False
        self.current_task = None
        self.is_returning_home = False
        self.landed = False
        self.landing_process = None
        self.rosbag_process = None
        self.rosbag_folder_name = "testing"
        self.battery_percentage = None
        self.ready_receive_mqtt = False
        self.mavros_state = State()
        self.mavros_extended_state = ExtendedState()
        self.current_status = 'offline'
        self._takeoff_altitude = 1.0
        self._takeoff_poll_timer = None
        self._takeoff_poll_count = 0
        self._takeoff_poll_task = None

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        try:
            self.client.connect(self.mqtt_broker, 1883, 60)
            self.client.loop_start()
            self.get_logger().info("Connected to MQTT Broker. Bridge started.")
        except Exception as e:
            self.get_logger().error(f"Cannot connect to MQTT: {e}")

        self.create_timer(3.0, self._set_ready)

    # ------------------------------------------------------------------ #
    #  State helpers                                                       #
    # ------------------------------------------------------------------ #

    def mavros_extended_state_callback(self, msg):
        self.mavros_extended_state = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def _is_in_air(self):
        return self.mavros_extended_state.landed_state == ExtendedState.LANDED_STATE_IN_AIR

    def initial_tf_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_init'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = msg.theta
        t.transform.rotation.w = 1.0
        self.static_tf.sendTransform(t)

    def _set_ready(self):
        if not self.ready_receive_mqtt:
            self.ready_receive_mqtt = True
            self.current_status = 'idle'
            self.get_logger().info("Manager is now ready to accept tasks.")

    # ------------------------------------------------------------------ #
    #  MQTT                                                                #
    # ------------------------------------------------------------------ #

    def on_connect(self, client, userdata, flags, rc):
        client.subscribe(self.nav_topic)
        client.subscribe(self.notification_topic)
        self.get_logger().info(
            f"Subscribed to MQTT topics: {self.nav_topic}, {self.notification_topic}")

    def on_message(self, client, userdata, msg):
        if not self.ready_receive_mqtt:
            self.get_logger().warn(
                f"Ignoring message on {msg.topic} (not ready yet, may be retained message)")
            return

        try:
            data = json.loads(msg.payload.decode('utf-8'))
            if msg.topic == self.nav_topic:
                self.handle_navigation_task(data)
            elif msg.topic == self.notification_topic:
                self.handle_notification(data)
        except json.JSONDecodeError:
            self.get_logger().error("Received invalid JSON format")
        except Exception as e:
            self.get_logger().error(f"Error converting message: {e}")

    # ------------------------------------------------------------------ #
    #  Task ingestion & queue                                              #
    # ------------------------------------------------------------------ #

    def handle_navigation_task(self, data):
        task_list = data if isinstance(data, list) else [data]

        for task_data in task_list:
            ros_msg = NavTask()
            ros_msg.task_id = task_data.get("task_id", "")
            ros_msg.rosbag_path = task_data.get("rosbag_path", "")
            ros_msg.source_timestamp = str(task_data.get("timestamp", ""))

            area_coords = task_data.get("area", [])
            for i in range(0, len(area_coords) - 1, 2):
                wp_msg = Pose2D()
                x_raw = float(area_coords[i])
                y_raw = float(area_coords[i + 1])
                wp_msg.x = round(((x_raw - 100) * 0.866 + 4) * 0.05, 2)
                wp_msg.y = round(-((y_raw - 100) * 0.89125 + 10) * 0.05, 2)
                ros_msg.waypoints.append(wp_msg)

            self.task_queue.append(ros_msg)
            self.get_logger().info(
                f"Queued Task {ros_msg.task_id}. Queue size: {len(self.task_queue)}")

        self.process_queue()

    def process_queue(self):
        """Dispatch next task; initiate takeoff sequence if drone is still on the ground."""
        if self.is_processing:
            return

        if not self.task_queue:
            if self._is_in_air():
                self.get_logger().info("Queue empty. Returning home.")
                self.return_home()
            return

        self.is_processing = True
        self.current_status = 'processing'
        next_task = self.task_queue.pop(0)
        self.current_task = next_task
        self.rosbag_folder_name = next_task.rosbag_path
        self.is_returning_home = False

        if self._is_in_air():
            self._execute_task(next_task)
        else:
            self.get_logger().info("Drone on ground. Starting flight sequence before task.")
            self._flight_sequence(next_task)

    # ------------------------------------------------------------------ #
    #  Flight sequence: set mode → arm → takeoff → poll altitude          #
    # ------------------------------------------------------------------ #

    def _flight_sequence(self, task):
        """Async chain: GUIDED mode → arm → takeoff → wait for air → execute."""
        if not self.mavros_mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("SetMode service unavailable. Aborting task.")
            self._abort_flight_sequence()
            return

        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        future = self.mavros_mode_client.call_async(req)
        future.add_done_callback(lambda f: self._on_mode_set(f, task))

    def _on_mode_set(self, future, task):
        try:
            if not future.result().mode_sent:
                self.get_logger().error("SetMode GUIDED failed. Aborting task.")
                self._abort_flight_sequence()
                return
        except Exception as e:
            self.get_logger().error(f"SetMode error: {e}")
            self._abort_flight_sequence()
            return

        self.get_logger().info("Mode GUIDED. Arming drone...")

        if not self.mavros_arm_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Arming service unavailable. Aborting task.")
            self._abort_flight_sequence()
            return

        req = CommandBool.Request()
        req.value = True
        future = self.mavros_arm_client.call_async(req)
        future.add_done_callback(lambda f: self._on_armed(f, task))

    def _on_armed(self, future, task):
        try:
            if not future.result().success:
                self.get_logger().error("Arming failed. Aborting task.")
                self._abort_flight_sequence()
                return
        except Exception as e:
            self.get_logger().error(f"Arming error: {e}")
            self._abort_flight_sequence()
            return

        self.get_logger().info(f"Armed. Taking off to {self._takeoff_altitude} m...")

        if not self.mavros_takeoff_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Takeoff service unavailable. Aborting task.")
            self._abort_flight_sequence()
            return

        req = CommandTOL.Request()
        req.altitude = float(self._takeoff_altitude)
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        future = self.mavros_takeoff_client.call_async(req)
        future.add_done_callback(lambda f: self._on_takeoff_sent(f, task))

    def _on_takeoff_sent(self, future, task):
        try:
            if not future.result().success:
                self.get_logger().error("Takeoff command rejected. Aborting task.")
                self._abort_flight_sequence()
                return
        except Exception as e:
            self.get_logger().error(f"Takeoff error: {e}")
            self._abort_flight_sequence()
            return

        self.get_logger().info("Takeoff command accepted. Waiting for altitude...")
        self._takeoff_poll_count = 0
        self._takeoff_poll_task = task
        self._takeoff_poll_timer = self.create_timer(1, self._poll_altitude)

    def _poll_altitude(self):
        """One-shot poll: cancel timer once airborne or timed out (10 s)."""
        self._takeoff_poll_count += 1

        if self._is_in_air():
            self._takeoff_poll_timer.cancel()
            self._takeoff_poll_timer = None
            self.get_logger().info("Drone airborne. Executing task.")
            self._execute_task(self._takeoff_poll_task)
            self._takeoff_poll_task = None
        elif self._takeoff_poll_count >= 20:
            self._takeoff_poll_timer.cancel()
            self._takeoff_poll_timer = None
            self.get_logger().error("Timed out waiting for takeoff. Aborting task.")
            self._abort_flight_sequence()

    def _abort_flight_sequence(self):
        self.is_processing = False
        self.current_status = 'error'

    # ------------------------------------------------------------------ #
    #  Task execution & result                                             #
    # ------------------------------------------------------------------ #

    def _execute_task(self, task):
        """Publish task to mission_dispatcher."""
        self.publisher_.publish(task)
        bridge_signal_ = Bool()
        bridge_signal_.data = True
        self.start_vel_bridge_signal_pub.publish(bridge_signal_)
        self.get_logger().info(
            f"Executing Task {task.task_id} (Remaining in queue: {len(self.task_queue)})")
        self.send_feedback(task_id=task.task_id, result=0, failed_faces=[])

    def result_callback(self, msg):
        self.get_logger().info(f"Received Result for Task {msg.task_id}: {msg.result}")

        self.send_feedback(
            task_id=msg.task_id,
            result=msg.result,
            failed_faces=list(msg.failed_faces) if msg.failed_faces is not None else [])

        self.record_rosbag("off")

        if msg.result in (1, 2):
            self.is_processing = False
            self.current_status = 'idle'
            self.process_queue()

    # ------------------------------------------------------------------ #
    #  Sensors & status                                                    #
    # ------------------------------------------------------------------ #

    def battery_callback(self, msg):
        self.battery_percentage = msg.percentage

    def is_landed_subscribe(self, msg):
        if msg.data:
            self.precision_landing("off")

    def ready_to_record_rosbag_signal_sub(self, msg):
        if msg.data:
            self.record_rosbag("on", path=self.rosbag_folder_name)
            self.get_logger().info("Starting rosbag recording")

    def status_report(self):
        payload = json.dumps({
            "status": self.current_status,
            "battery": self.battery_percentage
        })
        self.client.publish(self.status_topic, payload)

    def send_feedback(self, task_id, result, failed_faces):
        try:
            payload = json.dumps({
                "task_id": task_id,
                "result": result,
                "failed_faces": failed_faces,
                "timestamp": datetime.now().isoformat()
            })
            self.client.publish(self.feedback_topic, payload)
        except Exception as e:
            self.get_logger().error(f"Failed to publish feedback: {e}")

    # ------------------------------------------------------------------ #
    #  Return to Home                                                      #
    # ------------------------------------------------------------------ #

    def return_home(self):
        if self.is_returning_home:
            return

        if not self._is_in_air():
            self.get_logger().info("Drone already on ground. Skipping RTH.")
            return

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 Action Server unavailable. Cannot return home.")
            return

        self.get_logger().info("Initiating Return to Home...")
        self.is_processing = True
        self.is_returning_home = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.home_pose_x
        goal_msg.pose.pose.position.y = self.home_pose_y
        goal_msg.pose.pose.orientation.w = 1.0

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
        goal_handle.get_result_async().add_done_callback(self.home_result_callback)

    def home_result_callback(self, future):
        self.get_logger().info("Arrived at Home. Starting precision landing...")
        self.precision_landing("on")
        self.current_status = 'charging'
        self.is_processing = False
        self.is_returning_home = False

    # ------------------------------------------------------------------ #
    #  Notifications & termination                                         #
    # ------------------------------------------------------------------ #

    def handle_notification(self, data):
        notification = data.get("notification", "")
        self.get_logger().info(f"Received Notification: {notification}")

        if notification == "suspend":
            self.handle_termination()

    def handle_termination(self):
        """Clear queue, report cancelled tasks, return home."""
        self.record_rosbag("off")

        cancelled_ids = [task.task_id for task in self.task_queue]
        if self.is_processing and self.current_task:
            cancelled_ids.insert(0, self.current_task.task_id)

        self.task_queue.clear()
        self.is_processing = False
        # Note: mission_dispatcher's active Nav2 goal is NOT cancelled here.
        # A future improvement is to publish a cancel signal to mission_dispatcher.

        try:
            payload = json.dumps({
                "cancelled_task_ids": cancelled_ids,
                "timestamp": datetime.now().isoformat()
            })
            self.client.publish(self.cancelled_topic, payload)
            self.get_logger().warn(f"Mission terminated. Cancelled: {payload}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish cancellation report: {e}")

        self.return_home()

    # ------------------------------------------------------------------ #
    #  Subprocesses                                                        #
    # ------------------------------------------------------------------ #

    def _lifecycle_transition(self, client, transition_id, label):
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"{label} change_state service not available.")
            return
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f"{label} transition OK.")
            if f.result().success
            else self.get_logger().error(f"{label} transition FAILED.")
        )

    def precision_landing(self, action="on"):
        transition_id = {
            "on":  Transition.TRANSITION_ACTIVATE,
            "off": Transition.TRANSITION_DEACTIVATE,
        }.get(action)
        if transition_id is None:
            self.get_logger().warn(f"Unknown action for precision_landing: {action}")
            return
        self._lifecycle_transition(self._pl_change_state_cli, transition_id, "PrecisionLanding")

    def record_rosbag(self, action="on", path="default"):
        transition_id = {
            "on":  Transition.TRANSITION_ACTIVATE,
            "off": Transition.TRANSITION_DEACTIVATE,
        }.get(action)
        if transition_id is None:
            self.get_logger().warn(f"Unknown action for record_rosbag: {action}")
            return

        if action == "on":
            output_path = f"{self.rosbag_folder_path}/{path}"
            if not self._bag_set_params_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error("rosbag_node set_parameters service not available.")
                return
            param = Parameter()
            param.name = "output_path"
            param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=output_path)
            req = SetParameters.Request()
            req.parameters = [param]
            future = self._bag_set_params_cli.call_async(req)
            future.add_done_callback(
                lambda f: self._lifecycle_transition(
                    self._bag_change_state_cli, Transition.TRANSITION_ACTIVATE, f"Rosbag({path})")
            )
        else:
            self._lifecycle_transition(self._bag_change_state_cli, transition_id, f"Rosbag({path})")

    def cleanup_subprocesses(self):
        self._lifecycle_transition(
            self._pl_change_state_cli, Transition.TRANSITION_DEACTIVATE, "PrecisionLanding")
        self._lifecycle_transition(
            self._bag_change_state_cli, Transition.TRANSITION_DEACTIVATE, "Rosbag")


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
