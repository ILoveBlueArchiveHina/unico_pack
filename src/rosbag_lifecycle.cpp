#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <thread>
#include <chrono>

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

class RosbagLifecycle : public rclcpp_lifecycle::LifecycleNode {
public:
    RosbagLifecycle() : rclcpp_lifecycle::LifecycleNode("rosbag_node"), child_pid_(-1) {
        declare_parameter<std::string>("output_path", "/tmp/rosbag");
        declare_parameter<std::vector<std::string>>("topics", {
            "/zed/zed_node/rgb/color/rect/camera_info",
            "/zed/zed_node/rgb/color/rect/image/compressed",
            "/zed/zed_node/depth/depth_registered",
            "/zed/zed_node/depth/camera_info",
            "/zed/zed_node/imu/data",
            "/tf",
            "/tf_static"
        });
        RCLCPP_INFO(get_logger(), "rosbag_node instantiated.");
        configure_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { configure_timer_.reset(); this->configure(); });
    }

    LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
        output_path_ = get_parameter("output_path").as_string();
        topics_ = get_parameter("topics").as_string_array();
        RCLCPP_INFO(get_logger(), "Configured. Output: %s, Topics: %zu", output_path_.c_str(), topics_.size());
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
        // Re-read parameters so set_parameters before activate takes effect
        output_path_ = get_parameter("output_path").as_string();
        topics_ = get_parameter("topics").as_string_array();

        // Build argv for execlp: ros2 bag record -o <path> <topics...>
        std::vector<std::string> args = {"ros2", "bag", "record", "-o", output_path_};
        for (const auto & t : topics_) args.push_back(t);

        child_pid_ = fork();
        if (child_pid_ < 0) {
            RCLCPP_ERROR(get_logger(), "Fork failed!");
            return LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        if (child_pid_ == 0) {
            setsid();
            std::vector<char *> argv;
            for (auto & s : args) argv.push_back(s.data());
            argv.push_back(nullptr);
            execvp("ros2", argv.data());
            exit(1);
        }

        RCLCPP_INFO(get_logger(), "ros2 bag record started (PID %d) -> %s", child_pid_, output_path_.c_str());

        monitor_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RosbagLifecycle::monitor_child, this));

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
        monitor_timer_.reset();
        stop_process();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) {
        monitor_timer_.reset();
        stop_process();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) {
        monitor_timer_.reset();
        stop_process();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    pid_t child_pid_;
    rclcpp::TimerBase::SharedPtr configure_timer_;
    std::string output_path_;
    std::vector<std::string> topics_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;

    void monitor_child() {
        if (child_pid_ <= 0) return;
        int status;
        if (waitpid(child_pid_, &status, WNOHANG) == child_pid_) {
            RCLCPP_ERROR(get_logger(), "ros2 bag record died unexpectedly (PID %d)!", child_pid_);
            child_pid_ = -1;
            monitor_timer_.reset();
            this->deactivate();
        }
    }

    void stop_process() {
        if (child_pid_ <= 0) return;

        // SIGINT lets rosbag flush and write index before exit
        RCLCPP_INFO(get_logger(), "Sending SIGINT to rosbag process group %d", child_pid_);
        kill(-child_pid_, SIGINT);

        for (int i = 0; i < 50; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            int status;
            if (waitpid(child_pid_, &status, WNOHANG) == child_pid_) {
                RCLCPP_INFO(get_logger(), "Rosbag stopped cleanly. File saved to %s", output_path_.c_str());
                child_pid_ = -1;
                return;
            }
        }

        RCLCPP_WARN(get_logger(), "Timeout waiting for rosbag, sending SIGKILL.");
        kill(-child_pid_, SIGKILL);
        waitpid(child_pid_, nullptr, 0);
        child_pid_ = -1;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosbagLifecycle>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
