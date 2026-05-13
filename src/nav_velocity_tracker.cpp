#include <rclcpp/rclcpp.hpp>
#include <optional>
#include <cmath>
#include <algorithm>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class NavVelocityTracker : public rclcpp::Node {
public:
    NavVelocityTracker() : Node("nav_velocity_tracker"), tracking_active_(false) {
        declare_parameter<double>("yaw_align_threshold", 0.5);
        yaw_align_threshold_ = get_parameter("yaw_align_threshold").as_double();

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

        set_flight_altitude_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/set_flight_altitude", 1,
            std::bind(&NavVelocityTracker::set_altitude_callback, this, _1));

        set_altitude_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/set_flight_alt_done", 1);

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
    const double angular_gain_ = 1.5;
    const double max_angular_vel_ = 0.5;
    const double z_vel_kp = 0.5;
    const double max_z_vel = 0.5;   // m/s
    const double MAXIMUM_Z_TOLERANCE = 0.05;  // m
    const double MINIMUM_Z_VEL = 0.05;     // m/s
    double yaw_align_threshold_;
    double target_altitude_;
    bool is_on_altitude_ = true;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr center_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_nav_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr set_flight_altitude_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr set_altitude_done_pub_;
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

    void set_altitude_callback(const std_msgs::msg::Float32::SharedPtr msg){
        target_altitude_ = msg->data;
        is_on_altitude_ = false;
        return;
    }

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


        
        auto transform = lookup_transform("map", "body");
        if (!transform) {
            cmd_pub_->publish(*msg);
            return;
        }

        double robot_x = transform->transform.translation.x;
        double robot_y = transform->transform.translation.y;
        double robot_z = transform->transform.translation.z;
        // double qx = transform->transform.rotation.x;
        // double qy = transform->transform.rotation.y;
        double qz = transform->transform.rotation.z;
        double qw = transform->transform.rotation.w;
        
        double dx = center_x_ - robot_x;
        double dy = center_y_ - robot_y;
        double dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist < 1e-6) {
            cmd_pub_->publish(*msg);
            return;
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
        double angular_z = angular_gain_ * yaw_error;
        angular_z = std::clamp(angular_z, -max_angular_vel_, max_angular_vel_);

        // 角度差過大時先原地旋轉，不轉發線速度
        if (std::abs(yaw_error) > yaw_align_threshold_) {
            geometry_msgs::msg::Twist rotate_cmd;
            rotate_cmd.angular.z = angular_z;
            cmd_pub_->publish(rotate_cmd);
            return;
        }

        // Velocity vector rotation
        geometry_msgs::msg::Twist new_cmd;
        new_cmd.linear.x = e_cos * msg->linear.x + e_sin * msg->linear.y;
        new_cmd.linear.y = -e_sin * msg->linear.x + e_cos * msg->linear.y;

        double z_pos_error = target_altitude_ - robot_z;
        double z_vel = z_pos_error * z_vel_kp;
        z_vel = std::clamp(z_vel, -max_z_vel, max_z_vel);

        if (is_on_altitude_ && std::abs(z_pos_error) > MAXIMUM_Z_TOLERANCE) {
            new_cmd.linear.z = z_vel;
        } else {
            new_cmd.linear.z = msg->linear.z;
        }
        
        new_cmd.angular.z = angular_z;

        cmd_pub_->publish(new_cmd);
    }

    void control_loop() {
        if (is_on_altitude_) {
            return;
        }

        auto transform = lookup_transform("map", "body");
        if (!transform) {
            return;
        }

        double robot_z = transform->transform.translation.z;
        double z_pos_error = target_altitude_ - robot_z;
        if (std::abs(z_pos_error) < MAXIMUM_Z_TOLERANCE) {
            is_on_altitude_ = true;
            std_msgs::msg::Bool msg;
            msg.data = true;
            set_altitude_done_pub_->publish(msg);
            return;
        }
        double z_vel = z_pos_error * z_vel_kp;
        z_vel = std::clamp(z_vel, -max_z_vel, max_z_vel);
        
        geometry_msgs::msg::Twist new_cmd;
        new_cmd.linear.z = z_vel;
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