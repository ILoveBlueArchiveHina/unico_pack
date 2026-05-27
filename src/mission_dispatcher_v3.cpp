#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <campus_delivery_msgs/msg/nav_task.hpp>
#include <campus_delivery_msgs/msg/nav_result.hpp>

using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

class MissionDispatcher : public rclcpp::Node {
public:
    MissionDispatcher() : Node("mission_dispatcher") {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = cb_group_;

        task_sub_ = this->create_subscription<campus_delivery_msgs::msg::NavTask>(
            "navigation_tasks", 1,
            std::bind(&MissionDispatcher::task_callback, this, std::placeholders::_1),
            sub_opts);

        cancel_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "cancel_navigation", 1,
            std::bind(&MissionDispatcher::cancel_callback, this, std::placeholders::_1),
            sub_opts);

        result_pub_   = this->create_publisher<campus_delivery_msgs::msg::NavResult>("navigation_result", 1);
        rosbag_pub_   = this->create_publisher<std_msgs::msg::Bool>("ready_to_record_rosbag", 1);
        tracking_pub_ = this->create_publisher<geometry_msgs::msg::Point>("tracking_center", 1);

        action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints", cb_group_);

        RCLCPP_INFO(this->get_logger(), "Mission Dispatcher V3 started. Waiting for tasks...");
    }

private:
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Subscription<campus_delivery_msgs::msg::NavTask>::SharedPtr task_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cancel_sub_;
    rclcpp::Publisher<campus_delivery_msgs::msg::NavResult>::SharedPtr result_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rosbag_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr tracking_pub_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;

    std::string task_id_;
    double center_x_ = 0.0;
    double center_y_ = 0.0;

    std::deque<std::vector<std::pair<double, double>>> cargo_queue_;
    std::vector<std::pair<double, double>> expanded_points_;
    int current_start_index_ = 0;
    int current_leg_index_   = 0;
    std::vector<uint8_t> failed_faces_;

    GoalHandle::SharedPtr current_goal_handle_;
    bool is_task_cancelled_ = false;

    geometry_msgs::msg::PoseStamped create_pose(double x, double y) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = this->get_clock()->now();
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    void task_callback(const campus_delivery_msgs::msg::NavTask::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Task ID: %s with %zu points",
                    msg->task_id.c_str(), msg->waypoints.size());
        task_id_ = msg->task_id;

        if (msg->waypoints.size() == 4) {
            handle_cargo_task(msg->waypoints);
        } else {
            RCLCPP_WARN(this->get_logger(), "Refuse to execute a task with %zu waypoints.", msg->waypoints.size());
        }
    }

    void cancel_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && current_goal_handle_ != nullptr) {
            is_task_cancelled_ = true;
            action_client_->async_cancel_goal(current_goal_handle_);
            current_goal_handle_ = nullptr;
            cargo_queue_.clear();
            expanded_points_.clear();
            finish_task(false);
        }
    }

    void handle_cargo_task(const std::vector<geometry_msgs::msg::Pose2D> & waypoints) {
        double xs[4], ys[4];
        double sum_x = 0.0, sum_y = 0.0;
        for (int i = 0; i < 4; ++i) {
            xs[i] = waypoints[i].x;
            ys[i] = waypoints[i].y;
            sum_x += xs[i];
            sum_y += ys[i];
        }
        center_x_ = sum_x / 4.0;
        center_y_ = sum_y / 4.0;

        geometry_msgs::msg::Point tracking_center;
        tracking_center.x = center_x_;
        tracking_center.y = center_y_;
        tracking_center.z = 1.0;
        tracking_pub_->publish(tracking_center);

        RCLCPP_INFO(this->get_logger(), "Cargo Task Detected! Center: (%.2f, %.2f)", center_x_, center_y_);

        expanded_points_.clear();
        for (int i = 0; i < 4; ++i) {
            double dx = xs[i] - center_x_;
            double dy = ys[i] - center_y_;
            double length = std::sqrt(dx * dx + dy * dy);
            double ux = (length > 0.0) ? dx / length : 0.0;
            double uy = (length > 0.0) ? dy / length : 0.0;
            expanded_points_.push_back({xs[i] + ux * 2.0, ys[i] + uy * 2.0});
        }

        cargo_queue_.clear();
        failed_faces_.clear();
        for (int i = 0; i < 4; ++i) {
            auto p_end = expanded_points_[(i + 1) % 4];
            cargo_queue_.push_back({p_end});
        }

        RCLCPP_INFO(this->get_logger(), "Generated %zu inspection legs. Starting execution...", cargo_queue_.size());

        current_start_index_ = 0;
        try_next_start_point();
    }

    void try_next_start_point() {
        if (current_start_index_ >= static_cast<int>(expanded_points_.size())) {
            RCLCPP_ERROR(this->get_logger(), "All 4 starting points failed! Reporting task failure.");
            finish_task(false);
            return;
        }

        auto [x, y] = expanded_points_[current_start_index_];
        RCLCPP_INFO(this->get_logger(), "Phase 1: Trying start point %d/%zu (%.2f, %.2f)...",
                    current_start_index_ + 1, expanded_points_.size(), x, y);
        start_approach(x, y);
    }

    void start_approach(double x, double y) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server not available for approach!");
            current_start_index_++;
            try_next_start_point();
            return;
        }

        FollowWaypoints::Goal goal;
        goal.poses = {create_pose(x, y)};

        auto opts = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        opts.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
            approach_response_callback(handle);
        };
        opts.result_callback = [this](const GoalHandle::WrappedResult & result) {
            approach_result_callback(result);
        };
        action_client_->async_send_goal(goal, opts);
    }

    void approach_response_callback(GoalHandle::SharedPtr handle) {
        if (!handle) {
            RCLCPP_WARN(this->get_logger(), "Approach goal rejected for start point %d/%zu, trying next...",
                        current_start_index_ + 1, expanded_points_.size());
            current_start_index_++;
            try_next_start_point();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Approach goal accepted.");
        current_goal_handle_ = handle;
    }

    void approach_result_callback(const GoalHandle::WrappedResult & result) {
        current_goal_handle_ = nullptr;
        if (is_task_cancelled_) {
            is_task_cancelled_ = false;
            return;
        }

        if (result.result->missed_waypoints.empty()) {
            RCLCPP_INFO(this->get_logger(), "Phase 1: Approach Complete. Starting Phase 2: Inspection Loop.");
            current_leg_index_ = 0;

            // 從成功接近的點之後開始走
            std::rotate(cargo_queue_.begin(),
                        cargo_queue_.begin() + current_start_index_,
                        cargo_queue_.end());

            // 刪除 queue 中目標為失敗接近點的 leg（indices 0..current_start_index_-1）
            for (int k = 0; k < current_start_index_; ++k) {
                const auto & failed_pt = expanded_points_[k];
                cargo_queue_.erase(
                    std::remove_if(cargo_queue_.begin(), cargo_queue_.end(),
                        [&failed_pt](const std::vector<std::pair<double, double>> & leg) {
                            return !leg.empty() &&
                                   leg[0].first  == failed_pt.first &&
                                   leg[0].second == failed_pt.second;
                        }),
                    cargo_queue_.end());
            }

            RCLCPP_INFO(this->get_logger(), "Queue after pruning: %zu legs remain.", cargo_queue_.size());

            current_start_index_ = 0;

            std_msgs::msg::Bool ros_msg;
            ros_msg.data = true;
            rosbag_pub_->publish(ros_msg);
            process_next_cargo_leg();
        } else {
            RCLCPP_WARN(this->get_logger(), "Phase 1: Approach failed for start point %d/%zu, trying next...",
                        current_start_index_ + 1, expanded_points_.size());
            current_start_index_++;
            try_next_start_point();
        }
    }

    void process_next_cargo_leg() {
        if (cargo_queue_.empty()) {
            RCLCPP_INFO(this->get_logger(), "All cargo legs completed!");
            finish_task(true);
            return;
        }

        auto current_leg_points = cargo_queue_.front();
        cargo_queue_.pop_front();
        current_leg_index_++;
        RCLCPP_INFO(this->get_logger(), "Starting Leg %d/4. Remaining: %zu",
                    current_leg_index_, cargo_queue_.size());

        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server not available!");
            finish_task(false);
            return;
        }

        FollowWaypoints::Goal goal;
        for (auto & [x, y] : current_leg_points) {
            goal.poses.push_back(create_pose(x, y));
        }

        auto opts = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        opts.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
            cargo_leg_response_callback(handle);
        };
        opts.result_callback = [this](const GoalHandle::WrappedResult & result) {
            cargo_leg_result_callback(result);
        };
        action_client_->async_send_goal(goal, opts);
    }

    void cargo_leg_response_callback(GoalHandle::SharedPtr handle) {
        if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "Cargo leg %d rejected!", current_leg_index_);
            finish_task(false);
            return;
        }
        current_goal_handle_ = handle;
    }

    void cargo_leg_result_callback(const GoalHandle::WrappedResult & result) {
        current_goal_handle_ = nullptr;
        if (is_task_cancelled_) {
            is_task_cancelled_ = false;
            return;
        }

        for (auto idx : result.result->missed_waypoints) {
            if (idx == 0) {
                RCLCPP_WARN(this->get_logger(), "Face %d Endpoint unreachable.", current_leg_index_);
                failed_faces_.push_back(static_cast<uint8_t>(current_leg_index_));
            }
        }

        process_next_cargo_leg();
    }

    void finish_task(bool success) {
        geometry_msgs::msg::Point stop;
        stop.z = 0.0;
        tracking_pub_->publish(stop);

        campus_delivery_msgs::msg::NavResult result_msg;
        result_msg.task_id      = task_id_;
        result_msg.failed_faces = failed_faces_;

        if (!success) {
            result_msg.result = 2;
            RCLCPP_WARN(this->get_logger(), "Task Failed (Approach or Generic).");
        } else {
            result_msg.result = 1;
            RCLCPP_INFO(this->get_logger(), "Task Completed Successfully!");
        }

        result_pub_->publish(result_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionDispatcher>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
