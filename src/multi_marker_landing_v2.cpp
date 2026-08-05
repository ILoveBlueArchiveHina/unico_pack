#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <aruco_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <deque>
#include <map>
#include <vector>
#include <numeric>
#include <algorithm>

using namespace std::chrono_literals;

struct MarkerData {
    int id;
    double x, y, z;
    double qx, qy, qz, qw;
    double roll, pitch, yaw; // in degrees
};

struct CenterData {
    double x, y, z, yaw;
    int num_markers;
};

class MultiMarkerLanding : public rclcpp::Node {
public:
    MultiMarkerLanding() : Node("multi_marker_landing_cpp") {
        // 參數初始化
        // MARKER_IDS = {10, 20, 30, 440};
        // INNER_MARKER_IDS = {550};
        MARKER_IDS = {10, 20, 30, 40};
        INNER_MARKER_IDS = {100, 200, 300, 400};
        
        // Publishers
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        
        // Subscribers
        marker_sub_ = this->create_subscription<aruco_msgs::msg::MarkerArray>(
            "/marker_publisher/markers", 1,
            std::bind(&MultiMarkerLanding::markers_callback, this, std::placeholders::_1));
             
        // TF Broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // Timer (10Hz)
        timer_ = this->create_wall_timer(100ms, std::bind(&MultiMarkerLanding::control_loop, this));
        
        last_valid_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Multi-marker landing controller initialized");
    }

private:
    // 降落參數
    std::vector<int> MARKER_IDS;
    std::vector<int> INNER_MARKER_IDS;
    const int MIN_MARKERS_REQUIRED = 3;
    const double HALF_DISTANCE_BETWEEN_MARKERS = 0.25;  // m
    const double DESCENT_SPEED = -0.1;

    const double MAXIMUM_XY_SPEED = 0.3;
    const double MAXIMUM_ANG_SPEED = 0.3;  // rad/s

    const double Kp_xy = 0.5;
    const double Kp_yaw = 0.3;
    const double Ki_xy = 0.1;
    const double Ki_yaw = 0.05;

    // I 項只在誤差落在此帶寬內才累積，避免大誤差/遠距離移動時 windup
    const double INTEGRAL_BAND_XY = 0.15;   // m
    const double INTEGRAL_BAND_YAW = 2.0;   // deg
    const double I_MAX_XY = 0.15;           // m/s, Ki*integral 貢獻上限
    const double I_MAX_YAW = 0.15;          // rad/s, Ki*integral 貢獻上限
    const double CONTROL_DT = 0.1;          // 對應 100ms timer

    const double ALIGNMENT_THRESHOLD_XY = 0.08;
    const double ALIGNMENT_THRESHOLD_YAW = 3.0;
    const double ALIGNMENT_HOLD_TIME = 0.3;
    
    // 狀態變數
    std::map<int, MarkerData> detected_markers_;
    std::map<int, MarkerData> detected_inner_markers_;
    std::deque<CenterData> marker_buffer_;
    
    bool has_landing_center_ = false;
    CenterData landing_center_;
    
    int last_marker_count_ = 0;
    int last_inner_marker_count_ = 0;
    
    double aligned_time_ = -1.0;
    bool aligned_done_ = false;

    double integral_x_ = 0.0;
    double integral_y_ = 0.0;
    double integral_yaw_ = 0.0;
    
    bool last_no_center_ = false;
    rclcpp::Time last_valid_time_;
    

    // Publishers / Subscribers / Services
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    void markers_callback(const aruco_msgs::msg::MarkerArray::SharedPtr msg) {
        detected_markers_.clear();
        detected_inner_markers_.clear();
        
        for (const auto& marker : msg->markers) {
            bool is_outer = std::find(MARKER_IDS.begin(), MARKER_IDS.end(), marker.id) != MARKER_IDS.end();
            bool is_inner = std::find(INNER_MARKER_IDS.begin(), INNER_MARKER_IDS.end(), marker.id) != INNER_MARKER_IDS.end();
            
            if (is_outer || is_inner) {
                tf2::Quaternion q(
                    marker.pose.pose.orientation.x,
                    marker.pose.pose.orientation.y,
                    marker.pose.pose.orientation.z,
                    marker.pose.pose.orientation.w);
                
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                
                MarkerData md;
                md.id = marker.id;
                md.x = marker.pose.pose.position.x;
                md.y = marker.pose.pose.position.y;
                md.z = marker.pose.pose.position.z;
                md.qx = q.x(); md.qy = q.y(); md.qz = q.z(); md.qw = q.w();
                md.roll = roll * 180.0 / M_PI;
                md.pitch = pitch * 180.0 / M_PI;
                md.yaw = yaw * 180.0 / M_PI;
                
                if (is_outer) detected_markers_[marker.id] = md;
                if (is_inner) detected_inner_markers_[marker.id] = md;
            } 
        }
        
        bool has_outer = detected_markers_.size() > 0;
        bool has_more_outer = detected_markers_.size() >= (size_t)MIN_MARKERS_REQUIRED;
        bool has_inner = detected_inner_markers_.size() > 0;
        
        if (has_outer) {
            if (has_more_outer){
                has_landing_center_ = calculate_center(detected_markers_, landing_center_);
                // has_landing_center_ = less_marker_estimate(detected_markers_, landing_center_);
                if (has_landing_center_) {
                    update_buffer_and_tf(landing_center_);
                    if (detected_markers_.size() != (size_t)last_marker_count_) {
                        RCLCPP_INFO(this->get_logger(), "Detected %zu outer markers, using outer center", detected_markers_.size());
                        last_marker_count_ = detected_markers_.size();
                    }
                    last_valid_time_ = this->now();
                }
            } else {
                has_landing_center_ = less_marker_estimate(detected_markers_, landing_center_);
                if (has_landing_center_) {
                    update_buffer_and_tf(landing_center_);
                    last_valid_time_ = this->now();
                }
            }
            
        } else if (has_inner) {
            has_landing_center_ = calculate_center(detected_inner_markers_, landing_center_);
            if (has_landing_center_) {
                update_buffer_and_tf(landing_center_);
                if (last_marker_count_ >= MIN_MARKERS_REQUIRED) {
                    RCLCPP_INFO(this->get_logger(), "Outer markers lost, switching to inner markers only (%zu detected)", detected_inner_markers_.size());
                }
                last_marker_count_ = detected_markers_.size();
                last_inner_marker_count_ = detected_inner_markers_.size();
                last_valid_time_ = this->now();
            }
        } else {
            has_landing_center_ = false;
            RCLCPP_WARN(this->get_logger(), "Insufficient markers: outer=%zu, inner=%zu", detected_markers_.size(), detected_inner_markers_.size());
        }
    }

    bool calculate_center(const std::map<int, MarkerData>& markers, CenterData& center) {
        if (markers.empty()) return false;
        
        double sum_x = 0, sum_y = 0, sum_z = 0, sum_yaw = 0;
        for (const auto& pair : markers) {
            sum_x += pair.second.x;
            sum_y += pair.second.y;
            sum_z += pair.second.z;
            sum_yaw += pair.second.yaw;
        }
        
        int n = markers.size();
        center.x = sum_x / n;
        center.y = sum_y / n;
        center.z = sum_z / n;
        center.yaw = sum_yaw / n;
        center.num_markers = n;
        return true;
    }

    // Estimate center from 1–2 outer markers using known fixed offsets (yaw=0).
    // Layout (drone view looking down):
    //   10(top-left)  20(top-right)
    //   30(bot-left)  40(bot-right)
    // MarkerData.x = ROS pose.y (lateral, left +), MarkerData.y = ROS pose.x (forward +)
    bool less_marker_estimate(const std::map<int, MarkerData>& markers, CenterData& center) {
        if (markers.empty()) return false;
        

        const double H = HALF_DISTANCE_BETWEEN_MARKERS;
        // offset from center to each marker: center = marker - offset
        const std::map<int, std::pair<double, double>> offsets = {
            // 請注意： 相機座標y+朝上，但marker擺放y+朝下，因此下面象限定義y正負須相反
            {10, {-H, -H}},   // top-left:     left(-x), forward(-y)
            {20, { H, -H}},   // top-right:    right(x), forward(-y)
            {30, {-H,  H}},   // bottom-left:  left(-x), backward(y)
            {40, { H,  H}},   // bottom-right: right(x), backward(y)
        };

        double sum_cx = 0, sum_cy = 0, sum_cz = 0, sum_yaw = 0;
        int count = 0;

        for (const auto& pair : markers) {
            auto it = offsets.find(pair.first);
            if (it == offsets.end()) continue;
            const auto& m = pair.second;
            double dx = it->second.first;
            double dy = it->second.second;
            double theta = m.yaw * M_PI / 180.0;

            double dx_rot = dx * cos(theta) - dy * sin(theta);
            double dy_rot = dx * sin(theta) + dy * cos(theta);

            switch (pair.first) {
                case 10:
                    // 左上
                    RCLCPP_WARN(this->get_logger(), "id=%d, x=%f, y=%f,angle=%f",m.id, m.x, m.y, m.yaw);
                    // dx_rot = H * cos(theta) - dy * sin(theta);
                    // dy_rot = dx * sin(theta) + dy * cos(theta);
                    break;
                case 20:
                    // 右上
                    RCLCPP_WARN(this->get_logger(), "id=%d, x=%f, y=%f,angle=%f",m.id, m.x, m.y, m.yaw);
                    // dx_rot = dx * cos(theta) - dy * sin(theta);
                    // dy_rot = dx * sin(theta) + dy * cos(theta);
                    break;
                case 30:
                    // 左下
                    RCLCPP_WARN(this->get_logger(), "id=%d, x=%f, y=%f,angle=%f",m.id, m.x, m.y, m.yaw);
                    // dx_rot = dx * cos(theta) - dy * sin(theta);
                    // dy_rot = dx * sin(theta) + dy * cos(theta);
                    break;
                case 40:
                    // 右下
                    RCLCPP_WARN(this->get_logger(), "id=%d, x=%f, y=%f,angle=%f",m.id, m.x, m.y, m.yaw);
                    // dx_rot = dx * cos(theta) - dy * sin(theta);
                    // dy_rot = dx * sin(theta) + dy * cos(theta);
                    break;
                default:
                    continue;
            }

            

            sum_cx  += m.x - dx_rot;
            sum_cy  += m.y - dy_rot;
            sum_cz  += m.z;
            sum_yaw += m.yaw;
            count++;
        }

        if (count == 0) return false;

        center.x = sum_cx / count;
        center.y = sum_cy / count;
        center.z = sum_cz / count;
        center.yaw = sum_yaw / count;
        center.num_markers = count;

        RCLCPP_INFO(this->get_logger(), "less_marker_estimate: %d marker(s), est_center=(%.3f, %.3f, %.3f)",
                    count, center.x, center.y, center.z);
        return true;
    }

    void update_buffer_and_tf(const CenterData& center) {
        if (marker_buffer_.size() >= 10) marker_buffer_.pop_front();
        marker_buffer_.push_back(center);
        // publish_marker_transforms();
    }

    bool get_smoothed_center(CenterData& smoothed) {
        if (marker_buffer_.empty()) return false;
        
        double sum_x = 0, sum_y = 0, sum_z = 0, sum_yaw = 0;
        for (const auto& c : marker_buffer_) {
            sum_x += c.x; sum_y += c.y; sum_z += c.z; sum_yaw += c.yaw;
        }
        
        int n = marker_buffer_.size();
        smoothed.x = sum_x / n;
        smoothed.y = sum_y / n;
        smoothed.z = sum_z / n;
        smoothed.yaw = sum_yaw / n;
        return true;
    }

    // void publish_marker_transforms() {
    //     if (!has_landing_center_ || detected_markers_.size() < 2) return;
        
    //     rclcpp::Time now = this->now();
    //     geometry_msgs::msg::TransformStamped t_origin;
    //     t_origin.header.stamp = now;
    //     t_origin.header.frame_id = "camera_link";
    //     t_origin.child_frame_id = "aruco_origin";
        
    //     t_origin.transform.translation.x = landing_center_.x;
    //     t_origin.transform.translation.y = landing_center_.y;
    //     t_origin.transform.translation.z = landing_center_.z;
        
    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, landing_center_.yaw * M_PI / 180.0);
    //     t_origin.transform.rotation.x = q.x();
    //     t_origin.transform.rotation.y = q.y();
    //     t_origin.transform.rotation.z = q.z();
    //     t_origin.transform.rotation.w = q.w();
        
    //     tf_broadcaster_->sendTransform(t_origin);
        
    //     for (const auto& pair : detected_markers_) {
    //         const auto& m = pair.second;
    //         geometry_msgs::msg::TransformStamped t_marker;
    //         t_marker.header.stamp = now;
    //         t_marker.header.frame_id = "aruco_origin";
    //         t_marker.child_frame_id = "marker_" + std::to_string(m.id);
            
    //         t_marker.transform.translation.x = m.x - landing_center_.x;
    //         t_marker.transform.translation.y = m.y - landing_center_.y;
    //         t_marker.transform.translation.z = m.z - landing_center_.z;
            
    //         t_marker.transform.rotation.x = m.qx;
    //         t_marker.transform.rotation.y = m.qy;
    //         t_marker.transform.rotation.z = m.qz;
    //         t_marker.transform.rotation.w = m.qw;
            
    //         tf_broadcaster_->sendTransform(t_marker);
    //     }
    // }

    bool check_alignment(const CenterData& center) {
        double xy_offset = std::sqrt(center.x * center.x + center.y * center.y);
        double yaw_offset = std::abs(center.yaw);
        
        double current_time = this->now().seconds();
        
        if (xy_offset < ALIGNMENT_THRESHOLD_XY && yaw_offset < ALIGNMENT_THRESHOLD_YAW) {
            if (aligned_time_ < 0) {
                aligned_time_ = current_time;
                RCLCPP_INFO(this->get_logger(), "開始對齊 (偏差: xy=%.3fm, yaw=%.1f°)", xy_offset, yaw_offset);
            }
            if (current_time - aligned_time_ >= ALIGNMENT_HOLD_TIME) {
                aligned_done_ = true;
                return true;
            }
        } else {
            aligned_time_ = -1.0;
        }
        return false;
    }

    void control_loop() {
        double dt_lost = (this->now() - last_valid_time_).seconds();
        geometry_msgs::msg::Twist vel_cmd;
        
        if (dt_lost > 1.0) {
            if (!last_no_center_) {
                RCLCPP_WARN(this->get_logger(), "🚨 標記訊號遺失 (%.2fs)！垂直降落", dt_lost);
                last_no_center_ = true;
            }
            vel_cmd.linear.z = DESCENT_SPEED;
            vel_pub_->publish(vel_cmd);
            marker_buffer_.clear();
            integral_x_ = integral_y_ = integral_yaw_ = 0.0;
            return;
        }

        CenterData center;
        if (!get_smoothed_center(center)) {
            if (!last_no_center_) {
                RCLCPP_WARN(this->get_logger(), "No valid landing center, Slow landing...");
                last_no_center_ = true;
                vel_cmd.linear.z = DESCENT_SPEED;
                vel_pub_->publish(vel_cmd);
            }
            integral_x_ = integral_y_ = integral_yaw_ = 0.0;
            return;
        }
        last_no_center_ = false;
        
        check_alignment(center);
        bool xy_aligned = std::sqrt(center.x * center.x + center.y * center.y) < ALIGNMENT_THRESHOLD_XY;

        if (aligned_done_) vel_cmd.linear.z = DESCENT_SPEED;

        if (std::abs(center.x) < INTEGRAL_BAND_XY && std::abs(center.y) < INTEGRAL_BAND_XY) {
            integral_x_ += center.x * CONTROL_DT;
            integral_y_ += center.y * CONTROL_DT;
        } else {
            integral_x_ = integral_y_ = 0.0;
        }
        double i_term_x = std::clamp(Ki_xy * integral_x_, -I_MAX_XY, I_MAX_XY);
        double i_term_y = std::clamp(Ki_xy * integral_y_, -I_MAX_XY, I_MAX_XY);

        vel_cmd.linear.x = -(Kp_xy * center.y + i_term_y);
        vel_cmd.linear.y = -(Kp_xy * center.x + i_term_x);

        if (xy_aligned) {
            double yaw_err_rad = center.yaw * M_PI / 180.0;
            if (std::abs(center.yaw) < INTEGRAL_BAND_YAW) {
                integral_yaw_ += yaw_err_rad * CONTROL_DT;
            } else {
                integral_yaw_ = 0.0;
            }
            double i_term_yaw = std::clamp(Ki_yaw * integral_yaw_, -I_MAX_YAW, I_MAX_YAW);
            vel_cmd.angular.z = -(Kp_yaw * yaw_err_rad + i_term_yaw);
        } else {
            vel_cmd.angular.z = 0.0;
            integral_yaw_ = 0.0;
        }
        
        // 限制速度
        vel_cmd.linear.x = std::clamp(vel_cmd.linear.x, -MAXIMUM_XY_SPEED, MAXIMUM_XY_SPEED);
        vel_cmd.linear.y = std::clamp(vel_cmd.linear.y, -MAXIMUM_XY_SPEED, MAXIMUM_XY_SPEED);
        vel_cmd.linear.z = std::clamp(vel_cmd.linear.z, -MAXIMUM_XY_SPEED, MAXIMUM_XY_SPEED);
        vel_cmd.angular.z = std::clamp(vel_cmd.angular.z, -MAXIMUM_ANG_SPEED, MAXIMUM_ANG_SPEED);
        
        // 確保沒有 NaN
        if (!std::isfinite(vel_cmd.linear.x)) vel_cmd.linear.x = 0.0;
        if (!std::isfinite(vel_cmd.linear.y)) vel_cmd.linear.y = 0.0;
        if (!std::isfinite(vel_cmd.linear.z)) vel_cmd.linear.z = 0.0;
        if (!std::isfinite(vel_cmd.angular.z)) vel_cmd.angular.z = 0.0;
        
        vel_pub_->publish(vel_cmd);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiMarkerLanding>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}