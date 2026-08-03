#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

class LivoxLifecycleWrapper : public rclcpp_lifecycle::LifecycleNode {
public:
    LivoxLifecycleWrapper() : rclcpp_lifecycle::LifecycleNode("livox_lifecycle"), child_pid_(-1) {
        RCLCPP_INFO(get_logger(), "livox_lifecycle instantiated.");
        configure_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { configure_timer_.reset(); this->configure(); });
    }

    LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(get_logger(), "Configuring livox_lifecycle...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) {
        RCLCPP_INFO(get_logger(), "Activating... Starting Livox MID360 driver process.");

        child_pid_ = fork();
        if (child_pid_ < 0) {
            RCLCPP_ERROR(get_logger(), "Fork failed!");
            return LifecycleNodeInterface::CallbackReturn::FAILURE;
        }
        else if (child_pid_ == 0) {
            setsid();
            execlp("taskset", "taskset", "-c", "4,5",
                   "ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py", nullptr);
            exit(1);
        }

        RCLCPP_INFO(get_logger(), "Livox MID360 driver started with PID: %d", child_pid_);

        // 監控子行程是否崩潰，每 500ms 檢查一次
        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&LivoxLifecycleWrapper::monitor_child, this));

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) {
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

public:
    // Ctrl+C 時 lifecycle 的 shutdown transition 不會被觸發，
    // 由 main() 在 spin 結束後主動呼叫，避免 setsid 的子行程變孤兒。
    void shutdown_child() { stop_process(); }

private:
    pid_t child_pid_;
    rclcpp::TimerBase::SharedPtr configure_timer_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;

    void monitor_child() {
        if (child_pid_ <= 0) return;

        int status;
        pid_t result = waitpid(child_pid_, &status, WNOHANG);
        if (result == child_pid_) {
            // 子行程已死，但我們沒有要求它停
            RCLCPP_ERROR(get_logger(), "Livox MID360 driver process died unexpectedly (PID %d)! Triggering deactivate.", child_pid_);
            child_pid_ = -1;
            monitor_timer_.reset();
            // 觸發 lifecycle error，讓上層 manager 知道
            this->deactivate();
        }
    }

    void stop_process() {
        if (child_pid_ <= 0) return;

        RCLCPP_INFO(get_logger(), "Sending SIGINT to process group %d", child_pid_);
        kill(-child_pid_, SIGINT);

        // 最多等 3 秒讓節點自己關閉
        for (int i = 0; i < 30; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            int status;
            if (waitpid(child_pid_, &status, WNOHANG) == child_pid_) {
                RCLCPP_INFO(get_logger(), "Livox MID360 driver stopped cleanly.");
                child_pid_ = -1;
                return;
            }
        }

        // 超時就強制殺
        RCLCPP_WARN(get_logger(), "Timeout waiting for process, sending SIGKILL.");
        kill(-child_pid_, SIGKILL);
        waitpid(child_pid_, nullptr, 0);
        child_pid_ = -1;
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LivoxLifecycleWrapper>();
    rclcpp::spin(node->get_node_base_interface());
    node->shutdown_child();   // spin 因 SIGINT 返回後，主動收掉 setsid 的子行程，避免孤兒
    rclcpp::shutdown();
    return 0;
}
