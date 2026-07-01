#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <cmath>
#include <algorithm>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class NavVelocityTracker : public rclcpp::Node {
public:
    NavVelocityTracker() : Node("velocity_controller"), tracking_active_(false) {

        // TF Listener 設定
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        set_yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/set_target_yaw", 1,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                target_yaw_ = msg->data;
                is_on_yaw_ = false;
            });

        // 訂閱 Nav2 輸出的原始速度
        cmd_nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav", rclcpp::SystemDefaultsQoS(),
            std::bind(&NavVelocityTracker::cmd_vel_callback, this, _1));

        set_flight_altitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/set_flight_altitude", 1,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                target_altitude_ = msg->data;
                is_on_altitude_ = false;
                RCLCPP_INFO(this->get_logger(), "Receive alt command, target alt: %f", target_altitude_);

            });

        tracking_center_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/tracking_center", 10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg) {
                center_x_ = msg->x;
                center_y_ = msg->y;
                tracking_active_ = (msg->z > 0.0); // 利用 Z > 0 作為啟用追蹤的開關
                RCLCPP_INFO(this->get_logger(), "Tracking State Updated: %s, Center: (%.2f, %.2f)", 
                            tracking_active_ ? "ON" : "OFF", center_x_, center_y_);
            });

        set_altitude_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/set_flight_alt_done", 1);
        set_yaw_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/set_yaw_done", 1);
        // 發布修正後的速度給飛控
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        controller_timer_ = this->create_wall_timer(
            100ms, std::bind(&NavVelocityTracker::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "C++ Velocity Tracker initialized.");
    }

private:
    bool tracking_active_;
    double center_x_ = 0.0;
    double center_y_ = 0.0;
    bool is_on_yaw_= true;
    bool start_control_alt = false;
    bool is_on_altitude_ = true;
    double target_yaw_ = 0.0;
    const double yaw_kp_ = 1.0;
    const double angular_gain_ = 1.5;
    const double max_angular_vel_ = 0.5;
    const double z_vel_kp_ = 0.5;
    const double max_z_vel_ = 0.5;   // m/s
    const double maximum_z_tolerance_ = 0.05;  // m
    const double yaw_tolerance_ = 0.05;     // rad
    double target_altitude_;
    

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_nav_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_flight_altitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr set_yaw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr tracking_center_sub_;
    
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr set_altitude_done_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr set_yaw_done_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::TimerBase::SharedPtr controller_timer_;

    std::optional<geometry_msgs::msg::TransformStamped>
    lookup_transform(const std::string & target, const std::string & source) {
        try {
            return tf_buffer_->lookupTransform(
                target, source, tf2::TimePointZero, tf2::durationFromSec(0.05));
        } catch (const tf2::TransformException &) {
            return std::nullopt;
        }
    }

    double orientation_controller(const double & target, const double & current) {
        double error = std::atan2(std::sin(target - current), std::cos(target - current));
        if (std::abs(error) < yaw_tolerance_) {
            return 0.0;
        }
        double yaw_vel = error * yaw_kp_;
        yaw_vel = std::clamp(yaw_vel, -max_angular_vel_, max_angular_vel_);
        return yaw_vel;
    }

    double altitude_controller(const double & target, const double & current) {
        double error = target - current;
        if (std::abs(error) < maximum_z_tolerance_) {
            return 0.0;
        }
        double z_vel = error * z_vel_kp_;
        z_vel = std::clamp(z_vel, -max_z_vel_, max_z_vel_);
        return z_vel;
    }

    double tracking_orientation_culculate(double current_x,
        double current_y, double qw, double qz) {

        double dx = center_x_ - current_x;
        double dy = center_y_ - current_y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist < 1e-6) {
            return 0.0;
        }

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
        double angular_z = yaw_kp_ * yaw_error;
        return std::clamp(angular_z, -max_angular_vel_, max_angular_vel_);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

        auto transform = lookup_transform("map", "body");
        if (!transform) {
            cmd_pub_->publish(*msg);
            return;
        }
        double current_z = transform->transform.translation.z;
        geometry_msgs::msg::Twist new_cmd = *msg;
        if (start_control_alt) {
            new_cmd.linear.z = altitude_controller(target_altitude_, current_z);
        }

        double linear_speed_sq = (msg->linear.x * msg->linear.x) + (msg->linear.y * msg->linear.y);
        if (linear_speed_sq < 0.0001) {
            // 如果nav2採取剎車動作就別旋轉
            cmd_pub_->publish(*msg);
            return;
        }

        if (tracking_active_) {
            double current_x = transform->transform.translation.x;
            double current_y = transform->transform.translation.y;
            double qz = transform->transform.rotation.z;
            double qw = transform->transform.rotation.w;
            new_cmd.angular.z = tracking_orientation_culculate(current_x, current_y, qw, qz);
        }
        
        cmd_pub_->publish(new_cmd);
    }

    void control_loop() {
        if (is_on_altitude_ && is_on_yaw_) {
            return;
        }

        auto transform = lookup_transform("map", "body");
        if (!transform) {
            return;
        }
        double current_z = transform->transform.translation.z;
        double qz = transform->transform.rotation.z;
        double qw = transform->transform.rotation.w;
        double current_yaw = std::atan2(2.0 * qw * qz, qw * qw - qz * qz);
        
        geometry_msgs::msg::Twist new_cmd;
        
        if (!is_on_altitude_) {
            new_cmd.linear.z = altitude_controller(target_altitude_, current_z);
            if (new_cmd.linear.z == 0.0) {
                is_on_altitude_ = true;
                start_control_alt = true;
                std_msgs::msg::Bool msg;
                msg.data = true;
                set_altitude_done_pub_->publish(msg);
            }
        }
        
        if (!is_on_yaw_ && !tracking_active_) {
            new_cmd.angular.z = orientation_controller(target_yaw_, current_yaw);
            if (new_cmd.angular.z == 0.0) {
                is_on_yaw_ = true;
                std_msgs::msg::Bool msg;
                msg.data = true;
                set_yaw_done_pub_->publish(msg);
            }
        }
        
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