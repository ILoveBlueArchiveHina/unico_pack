#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

using std::placeholders::_1;

class NavVelocityTracker : public rclcpp::Node {
public:
    NavVelocityTracker() : Node("nav_velocity_tracker"), tracking_active_(false) {
        
        // TF Listener 設定
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 接收來自 Python Dispatcher 的追蹤中心點
        center_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/tracking_center", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                center_x_ = msg->x;
                center_y_ = msg->y;
                tracking_active_ = (msg->z > 0.0); // 利用 Z > 0 作為啟用追蹤的開關
                RCLCPP_INFO(this->get_logger(), "Tracking State Updated: %s, Center: (%.2f, %.2f)", 
                            tracking_active_ ? "ON" : "OFF", center_x_, center_y_);
            });

        // 訂閱 Nav2 輸出的原始速度
        cmd_nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", rclcpp::SystemDefaultsQoS(),
            std::bind(&NavVelocityTracker::cmd_vel_callback, this, _1));

        // 發布修正後的速度給飛控
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "C++ Velocity Tracker initialized.");
    }

private:
    bool tracking_active_;
    double center_x_ = 0.0;
    double center_y_ = 0.0;
    const double angular_gain_ = 1.5;
    const double max_angular_vel_ = 0.5;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr center_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_nav_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!tracking_active_) {
            // 如果沒開啟追蹤，直接透傳 (Pass-through)
            cmd_pub_->publish(*msg);
            return;
        }

        double linear_speed_sq = (msg->linear.x * msg->linear.x) + (msg->linear.y * msg->linear.y);
        
        if (linear_speed_sq < 0.00001) {
            // 如果nav2採取剎車動作就別旋轉
            cmd_pub_->publish(*msg);
            return;
        }


        geometry_msgs::msg::TransformStamped transform;
        try {
            // C++ 的 TF 查詢速度比 Python 快上數十倍
            transform = tf_buffer_->lookupTransform(
                "map", "body", tf2::TimePointZero, tf2::durationFromSec(0.05));
        } catch (const tf2::TransformException & ex) {
            cmd_pub_->publish(*msg);
            return;
        }

        double robot_x = transform.transform.translation.x;
        double robot_y = transform.transform.translation.y;
        
        double dx = center_x_ - robot_x;
        double dy = center_y_ - robot_y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist < 1e-6) {
            cmd_pub_->publish(*msg);
            return;
        }

        // 使用原本 Python 腳本中的複數運算邏輯 (避開 Euler gimbal lock)
        double qx = transform.transform.rotation.x;
        double qy = transform.transform.rotation.y;
        double qz = transform.transform.rotation.z;
        double qw = transform.transform.rotation.w;

        // Robot heading (r_cos, r_sin)
        double r_cos = qw * qw - qz * qz;
        double r_sin = 2.0 * qw * qz;

        // Desired heading (d_cos, d_sin)
        double d_cos = dx / dist;
        double d_sin = dy / dist;

        // Error rotation
        double e_cos = d_cos * r_cos + d_sin * r_sin;
        double e_sin = d_sin * r_cos - d_cos * r_sin;

        // Angular P-controller
        double yaw_error = std::atan2(e_sin, e_cos);
        double angular_z = angular_gain_ * yaw_error;
        angular_z = std::clamp(angular_z, -max_angular_vel_, max_angular_vel_);

        // Velocity vector rotation
        geometry_msgs::msg::Twist new_cmd;
        new_cmd.linear.x = e_cos * msg->linear.x + e_sin * msg->linear.y;
        new_cmd.linear.y = -e_sin * msg->linear.x + e_cos * msg->linear.y;
        new_cmd.linear.z = msg->linear.z;
        new_cmd.angular.z = angular_z;

        cmd_pub_->publish(new_cmd);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavVelocityTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}