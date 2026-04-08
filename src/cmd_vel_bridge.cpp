#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>

#include <chrono>

using namespace std::chrono_literals;

class CopterVelocityControlFixed : public rclcpp::Node {
public:
    CopterVelocityControlFixed() : Node("cmd_vel_bridge") {
        
        // 發布速度命令
        pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
            "/mavros/setpoint_raw/local", 1);
            
        // 訂閱 cmd_vel
        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 1,
            std::bind(&CopterVelocityControlFixed::cmd_callback, this, std::placeholders::_1));
            
        // 訂閱狀態
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 1,
            std::bind(&CopterVelocityControlFixed::state_callback, this, std::placeholders::_1));
            
        // 關鍵1：必須持續發送（20Hz = 50ms）
        timer_ = this->create_wall_timer(
            50ms, std::bind(&CopterVelocityControlFixed::send_velocity, this));
            
        RCLCPP_INFO(this->get_logger(), "速度控制節點已啟動（C++ 修正版）");
        RCLCPP_INFO(this->get_logger(), "節點運行中...");
        RCLCPP_INFO(this->get_logger(), "請先：1) 切換GUIDED模式  2) 解鎖  3) 起飛");
        RCLCPP_INFO(this->get_logger(), "然後發送cmd_vel命令");
    }

private:
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
    }
    
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 接收 cmd_vel 並儲存（不直接發送）
        target_velocity_ = *msg;
    }
    
    void send_velocity() {
        // 只在已連接且在 GUIDED 模式時發送
        if (!current_state_.connected) {
            return;
        }
        if (current_state_.mode != "GUIDED") {
            return;
        }
        
        mavros_msgs::msg::PositionTarget cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        
        // 選擇座標系：機體座標系（機頭方向）
        cmd.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_NED;
        
        // 關鍵2：修正 type_mask，允許 yaw_rate 控制
        // 僅忽略：位置(PX,PY,PZ) + 加速度(AFX,AFY,AFZ) + 偏航角(YAW)
        // 不忽略：速度(VX,VY,VZ) + 偏航率(YAW_RATE)
        cmd.type_mask = 
            mavros_msgs::msg::PositionTarget::IGNORE_PX |
            mavros_msgs::msg::PositionTarget::IGNORE_PY |
            mavros_msgs::msg::PositionTarget::IGNORE_PZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW;
            
        // 設置速度
        cmd.velocity.x = target_velocity_.linear.x;
        cmd.velocity.y = target_velocity_.linear.y;
        cmd.velocity.z = target_velocity_.linear.z;
        cmd.yaw_rate = target_velocity_.angular.z;
        
        pub_->publish(cmd);
    }

    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    mavros_msgs::msg::State current_state_;
    geometry_msgs::msg::Twist target_velocity_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CopterVelocityControlFixed>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}