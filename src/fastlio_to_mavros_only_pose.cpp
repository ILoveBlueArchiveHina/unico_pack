#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <array>
#include <cmath>

class FastLioVisionBridge : public rclcpp::Node
{
public:
  FastLioVisionBridge() : Node("fastlio_vision_bridge"), msg_count_(0)
  {
    declare_parameter("fastlio_odom_topic", "/Odometry");
    declare_parameter("mavros_vision_pose_topic", "/mavros/vision_pose/pose_cov");
    declare_parameter("output_frame_id", "odom");
    declare_parameter("position_cov_scale", 1000.0);
    declare_parameter("orientation_cov_scale", 1000.0);
    declare_parameter("min_position_cov", 0.01);
    declare_parameter("min_orientation_cov", 0.01);

    const auto fastlio_topic  = get_parameter("fastlio_odom_topic").as_string();
    const auto mavros_topic   = get_parameter("mavros_vision_pose_topic").as_string();
    output_frame_   = get_parameter("output_frame_id").as_string();
    pos_scale_      = get_parameter("position_cov_scale").as_double();
    ori_scale_      = get_parameter("orientation_cov_scale").as_double();
    min_pos_cov_    = get_parameter("min_position_cov").as_double();
    min_ori_cov_    = get_parameter("min_orientation_cov").as_double();

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      fastlio_topic, 10,
      std::bind(&FastLioVisionBridge::odomCallback, this, std::placeholders::_1));

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(mavros_topic, qos);

    RCLCPP_INFO(get_logger(), "==================================================");
    RCLCPP_INFO(get_logger(), "FAST-LIO2 Vision Bridge Started (Pose Only)");
    RCLCPP_INFO(get_logger(), "  Subscribe: %s", fastlio_topic.c_str());
    RCLCPP_INFO(get_logger(), "  Publish:   %s", mavros_topic.c_str());
    RCLCPP_INFO(get_logger(), "  Frame ID:  %s", output_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  Position Cov Scale: %.1f", pos_scale_);
    RCLCPP_INFO(get_logger(), "  Orientation Cov Scale: %.1f", ori_scale_);
    RCLCPP_INFO(get_logger(), "  Min Position Cov: %.4f", min_pos_cov_);
    RCLCPP_INFO(get_logger(), "  Min Orientation Cov: %.4f", min_ori_cov_);
    RCLCPP_INFO(get_logger(), "==================================================");
  }

private:
  using CovArray = std::array<double, 36>;

  CovArray scaleCovariance(const CovArray & orig) const
  {
    CovArray scaled = orig;

    // diagonal: position (0,7,14), orientation (21,28,35)
    for (int i : {0, 7, 14}) {
      scaled[i] = std::max(orig[i] * pos_scale_, min_pos_cov_);
    }
    for (int i : {21, 28, 35}) {
      scaled[i] = std::max(orig[i] * ori_scale_, min_ori_cov_);
    }

    const double cross_scale = std::sqrt(pos_scale_ * ori_scale_);

    for (int i = 0; i < 36; ++i) {
      if (i == 0 || i == 7 || i == 14 || i == 21 || i == 28 || i == 35) continue;
      const int row = i / 6;
      const int col = i % 6;
      if (row < 3 && col < 3) {
        scaled[i] = orig[i] * pos_scale_;
      } else if (row >= 3 && col >= 3) {
        scaled[i] = orig[i] * ori_scale_;
      } else {
        scaled[i] = orig[i] * cross_scale;
      }
    }

    return scaled;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped vision_msg;
    vision_msg.header.stamp    = msg->header.stamp;
    vision_msg.header.frame_id = output_frame_;
    vision_msg.pose.pose       = msg->pose.pose;
    vision_msg.pose.covariance = scaleCovariance(msg->pose.covariance);

    pub_->publish(vision_msg);

    if (++msg_count_ % 10 == 0) {
      const auto & pos   = msg->pose.pose.position;
      const auto & stamp = msg->header.stamp;
      RCLCPP_INFO(
        get_logger(),
        "[%d] Time: %d.%03d | Pos: (%.3f, %.3f, %.3f) | Cov: %.6f -> %.4f",
        msg_count_,
        stamp.sec, stamp.nanosec / 1000000,
        pos.x, pos.y, pos.z,
        msg->pose.covariance[0],
        vision_msg.pose.covariance[0]);
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_;

  std::string output_frame_;
  double pos_scale_;
  double ori_scale_;
  double min_pos_cov_;
  double min_ori_cov_;
  int msg_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastLioVisionBridge>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
  }
  RCLCPP_INFO(node->get_logger(), "Bridge stopped.");
  rclcpp::shutdown();
  return 0;
}
