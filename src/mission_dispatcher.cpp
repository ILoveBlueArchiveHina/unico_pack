#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// 替換成你實際的 custom messages include 路徑
#include <campus_delivery_msgs/msg/nav_task.hpp>
#include <campus_delivery_msgs/msg/nav_result.hpp>

#include <cmath>
#include <vector>
#include <deque>
#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

struct Point2D {
    double x;
    double y;
};

class Nav2Executor : public rclcpp::Node {
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFW = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    Nav2Executor() : Node("mission_dispatcher") {
        // Callback group 設定 (可重入)
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        // 1. Subscribe to NavTask
        task_sub_ = this->create_subscription<campus_delivery_msgs::msg::NavTask>(
            "navigation_tasks", 10,
            std::bind(&Nav2Executor::listener_callback, this, _1), sub_opt);

        // Publishers
        result_pub_ = this->create_publisher<campus_delivery_msgs::msg::NavResult>(
            "navigation_result", 10);
            
        rosbag_ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "ready_to_record_rosbag", 10);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 2. Action Client
        action_client_ = rclcpp_action::create_client<FollowWaypoints>(
            this, "follow_waypoints", callback_group_);

        // 3. TF Listener Setup
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel_nav", 10,
            std::bind(&Nav2Executor::cmd_vel_callback, this, _1), sub_opt);

        RCLCPP_INFO(this->get_logger(), "Nav2 Executor Node Started. Waiting for tasks...");
    }

private:
    // State Variables
    bool tracking_active_ = false;
    double center_x_ = 0.0;
    double center_y_ = 0.0;
    double angular_gain_ = 1.5;

    std::string task_id_ = ""; // 改為字串
    std::deque<std::vector<Point2D>> cargo_queue_;
    int current_leg_index_ = 0;
    std::vector<uint8_t> failed_faces_;
    std::vector<Point2D> expanded_points_;
    size_t current_start_index_ = 0;

    // ROS interfaces
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<campus_delivery_msgs::msg::NavTask>::SharedPtr task_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<campus_delivery_msgs::msg::NavResult>::SharedPtr result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rosbag_ready_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void listener_callback(const campus_delivery_msgs::msg::NavTask::SharedPtr msg) {
        // %d 改成 %s，並加上 msg->task_id.c_str() 轉換為 C 字串
        RCLCPP_INFO(this->get_logger(), "Received Task ID: %s with %zu points", msg->task_id.c_str(), msg->waypoints.size());
        task_id_ = msg->task_id;

        if (msg->waypoints.size() == 4) {
            handle_cargo_task(msg->waypoints);
        } else {
            RCLCPP_WARN(this->get_logger(), "Refuse to execute a task with %zu waypoints.", msg->waypoints.size());
        }
    }

    template<typename T>
    void handle_cargo_task(const std::vector<T>& waypoints_data) {
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& wp : waypoints_data) {
            sum_x += wp.x;
            sum_y += wp.y;
        }
        center_x_ = sum_x / 4.0;
        center_y_ = sum_y / 4.0;
        tracking_active_ = true;

        RCLCPP_INFO(this->get_logger(), "Cargo Task Detected! Deep Inspection Mode. Center: (%.2f, %.2f)", center_x_, center_y_);

        expanded_points_.clear();
        for (size_t i = 0; i < 4; ++i) {
            double dx = waypoints_data[i].x - center_x_;
            double dy = waypoints_data[i].y - center_y_;
            double length = std::sqrt(dx * dx + dy * dy);
            
            double ux = (length > 0) ? (dx / length) : 0.0;
            double uy = (length > 0) ? (dy / length) : 0.0;

            Point2D p;
            p.x = waypoints_data[i].x + ux * 2.0;
            p.y = waypoints_data[i].y + uy * 2.0;
            expanded_points_.push_back(p);
        }

        cargo_queue_.clear();
        failed_faces_.clear();

        for (size_t i = 0; i < 4; ++i) {
            Point2D p_end = expanded_points_[(i + 1) % 4];
            std::vector<Point2D> leg_points = { p_end };
            cargo_queue_.push_back(leg_points);
        }

        RCLCPP_INFO(this->get_logger(), "Generated %zu inspection legs. Starting execution...", cargo_queue_.size());

        current_start_index_ = 0;
        try_next_start_point();
    }

    void try_next_start_point() {
        if (current_start_index_ >= expanded_points_.size()) {
            RCLCPP_ERROR(this->get_logger(), "All 4 starting points failed! Reporting task failure.");
            finish_task(false);
            return;
        }

        size_t idx = current_start_index_;
        double x = expanded_points_[idx].x;
        double y = expanded_points_[idx].y;
        
        RCLCPP_INFO(this->get_logger(), "Phase 1: Trying start point %zu/%zu (%.2f, %.2f)...", 
                    idx + 1, expanded_points_.size(), x, y);
        start_approach(x, y);
    }

    void start_approach(double x, double y) {
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server not available for approach!");
            current_start_index_++;
            try_next_start_point();
            return;
        }

        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses.push_back(create_pose(x, y));

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&Nav2Executor::approach_response_callback, this, _1);
        send_goal_options.result_callback =
            std::bind(&Nav2Executor::approach_result_callback, this, _1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void approach_response_callback(const GoalHandleFW::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "Approach goal rejected for start point %zu/%zu, trying next...", 
                        current_start_index_ + 1, expanded_points_.size());
            current_start_index_++;
            try_next_start_point();
        } else {
            RCLCPP_INFO(this->get_logger(), "Approach goal accepted.");
        }
    }

    void approach_result_callback(const GoalHandleFW::WrappedResult& result) {
        cmd_pub_->publish(geometry_msgs::msg::Twist()); // brakes
        
        if (result.result->missed_waypoints.empty()) {
            RCLCPP_INFO(this->get_logger(), "Phase 1: Approach Complete. Starting Phase 2: Inspection Loop.");
            current_start_index_ = 0;
            current_leg_index_ = 0;
            
            std_msgs::msg::Bool ros_msg;
            ros_msg.data = true;
            rosbag_ready_pub_->publish(ros_msg);
            
            process_next_cargo_leg();
        } else {
            RCLCPP_WARN(this->get_logger(), "Phase 1: Approach failed for start point %zu/%zu, trying next...", 
                        current_start_index_ + 1, expanded_points_.size());
            current_start_index_++;
            try_next_start_point();
        }
    }

    geometry_msgs::msg::PoseStamped create_pose(double x, double y) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    void process_next_cargo_leg() {
        if (cargo_queue_.empty()) {
            RCLCPP_INFO(this->get_logger(), "All cargo legs completed!");
            finish_task(failed_faces_.empty());
            return;
        }

        std::vector<Point2D> current_leg_points = cargo_queue_.front();
        cargo_queue_.pop_front();
        current_leg_index_++;
        
        RCLCPP_INFO(this->get_logger(), "Starting Leg %d/4 (FollowWaypoints Mode). Remaining: %zu", 
                    current_leg_index_, cargo_queue_.size());

        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server not available!");
            finish_task(false);
            return;
        }

        auto goal_msg = FollowWaypoints::Goal();
        for (const auto& p : current_leg_points) {
            goal_msg.poses.push_back(create_pose(p.x, p.y));
        }

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&Nav2Executor::cargo_leg_response_callback, this, _1);
        send_goal_options.result_callback =
            std::bind(&Nav2Executor::cargo_leg_result_callback, this, _1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void cargo_leg_response_callback(const GoalHandleFW::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Cargo leg %d rejected!", current_leg_index_);
            finish_task(false);
        }
    }

    void cargo_leg_result_callback(const GoalHandleFW::WrappedResult& result) {
        cmd_pub_->publish(geometry_msgs::msg::Twist()); // brakes
        
        auto missed = result.result->missed_waypoints;
        if (std::find(missed.begin(), missed.end(), 0) != missed.end()) {
            RCLCPP_WARN(this->get_logger(), "Face %d Endpoint unreachable.", current_leg_index_);
            // 將 int 強制轉型為 uint8_t 存入
            failed_faces_.push_back(static_cast<uint8_t>(current_leg_index_));
        }
        
        process_next_cargo_leg();
    }

    void finish_task(bool success) {
        tracking_active_ = false;
        campus_delivery_msgs::msg::NavResult result_msg;
        result_msg.header.stamp = this->get_clock()->now();
        result_msg.header.frame_id = "map";
        result_msg.task_id = task_id_;
        result_msg.failed_faces = failed_faces_;

        if (!success) {
            result_msg.result = 2; // Approach failed or generic
            RCLCPP_WARN(this->get_logger(), "Task Failed (Approach or Generic).");
        } else {
            result_msg.result = 1;
            RCLCPP_INFO(this->get_logger(), "Task Completed Successfully!");
        }

        result_pub_->publish(result_msg);
        cmd_pub_->publish(geometry_msgs::msg::Twist());
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!tracking_active_) {
            cmd_pub_->publish(*msg);
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(
                "map", "body", tf2::TimePointZero, tf2::durationFromSec(0.1));
        } catch (const tf2::TransformException & ex) {
            // TF lookup 失敗時，直接送出原速度
            cmd_pub_->publish(*msg);
            return;
        }

        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;

        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, robot_yaw;
        m.getRPY(roll, pitch, robot_yaw);

        // 計算期望角度 (朝向 orbit center)
        double dx = center_x_ - robot_x;
        double dy = center_y_ - robot_y;
        double desired_yaw = std::atan2(dy, dx);

        // 計算 yaw error 並將其正規化到 [-pi, pi]
        double yaw_error = desired_yaw - robot_yaw;
        yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));

        // 產生新的控制指令
        geometry_msgs::msg::Twist new_cmd = *msg;
        new_cmd.angular.z = angular_gain_ * yaw_error;

        // 限制最大角速度
        const double max_angular_vel = 0.5;
        new_cmd.angular.z = std::clamp(new_cmd.angular.z, -max_angular_vel, max_angular_vel);

        cmd_pub_->publish(new_cmd);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 因為有 Action Client 以及高頻率的 Callbacks，建議使用 MultiThreadedExecutor
    auto node = std::make_shared<Nav2Executor>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}