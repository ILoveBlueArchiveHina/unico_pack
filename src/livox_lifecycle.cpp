#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

class LivoxLifecycleWrapper : public rclcpp_lifecycle::LifecycleNode {
public:
    LivoxLifecycleWrapper()
    : rclcpp_lifecycle::LifecycleNode("livox_lifecycle"), child_pid_(-1), gpio_pid_(-1) {
        RCLCPP_INFO(get_logger(), "livox_lifecycle instantiated.");
        // 繼電器供電後到 MID360 開機完成的等待秒數，太短的話 driver 會連不上光達
        this->declare_parameter("lidar_power_on_delay", 15.0);
        configure_timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { configure_timer_.reset(); this->configure(); });
    }

    LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(get_logger(), "Configuring livox_lifecycle...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(get_logger(), "Activating... Powering on LiDAR through GPIO relay.");

        // 1. 先讓繼電器給光達供電。lidar_gpio.py 會常駐把 GPIO 拉在 HIGH，
        //    收到 SIGINT 才斷電結束，所以「它還活著」就代表光達還有電。
        gpio_pid_ = spawn_process("lidar_gpio", {"ros2", "run", "unico_pack", "lidar_gpio.py"});
        if (gpio_pid_ < 0) {
            return LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        // 2. 等光達開機，剛通電就開 driver 會抓不到裝置
        const double power_on_delay = this->get_parameter("lidar_power_on_delay").as_double();
        RCLCPP_INFO(get_logger(), "Waiting %.1f s for MID360 to boot up...", power_on_delay);
        std::this_thread::sleep_for(std::chrono::duration<double>(power_on_delay));

        // python 沒撐過開機等待就代表 GPIO 設定失敗（例如權限不足），此時光達根本沒電
        if (waitpid(gpio_pid_, nullptr, WNOHANG) == gpio_pid_) {
            RCLCPP_ERROR(get_logger(), "lidar_gpio.py exited during power-up! LiDAR has no power.");
            gpio_pid_ = -1;
            return LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        // 3. 光達已上電，開 driver
        RCLCPP_INFO(get_logger(), "Starting Livox MID360 driver process.");
        child_pid_ = spawn_process("Livox MID360 driver",
            {"taskset", "-c", "4,5", "ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py"});
        if (child_pid_ < 0) {
            stop_process(gpio_pid_, "lidar_gpio");   // driver 起不來就順手把電斷掉
            return LifecycleNodeInterface::CallbackReturn::FAILURE;
        }

        // 監控子行程是否崩潰，每 500ms 檢查一次
        monitor_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&LivoxLifecycleWrapper::monitor_child, this));

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) {
        monitor_timer_.reset();
        stop_all();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) {
        monitor_timer_.reset();
        stop_all();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) {
        monitor_timer_.reset();
        stop_all();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

public:
    // Ctrl+C 時 lifecycle 的 shutdown transition 不會被觸發，
    // 由 main() 在 spin 結束後主動呼叫，避免 setsid 的子行程變孤兒（也確保光達會斷電）。
    void shutdown_child() { stop_all(); }

private:
    pid_t child_pid_;   // livox_ros_driver2
    pid_t gpio_pid_;    // lidar_gpio.py（持有繼電器 HIGH）
    rclcpp::TimerBase::SharedPtr configure_timer_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;

    // fork + setsid + execvp，回傳子行程 PID，失敗回傳 -1。
    // argv 在 fork 之前就組好，child 端只呼叫 execvp/_exit，不做任何配置記憶體的動作。
    pid_t spawn_process(const std::string & name, const std::vector<std::string> & args) {
        std::vector<char *> argv;
        for (const auto & arg : args) {
            argv.push_back(const_cast<char *>(arg.c_str()));
        }
        argv.push_back(nullptr);

        pid_t pid = fork();
        if (pid < 0) {
            RCLCPP_ERROR(get_logger(), "Fork failed for %s!", name.c_str());
            return -1;
        }
        else if (pid == 0) {
            setsid();
            execvp(argv[0], argv.data());
            _exit(1);
        }

        RCLCPP_INFO(get_logger(), "%s started with PID: %d", name.c_str(), pid);
        return pid;
    }

    void monitor_child() {
        // driver 掛掉：交給 deactivate 收尾（順便斷電），讓上層 manager 知道
        if (child_pid_ > 0 && waitpid(child_pid_, nullptr, WNOHANG) == child_pid_) {
            RCLCPP_ERROR(get_logger(), "Livox MID360 driver process died unexpectedly (PID %d)! Triggering deactivate.", child_pid_);
            child_pid_ = -1;
            monitor_timer_.reset();
            this->deactivate();
            return;
        }

        // GPIO 腳本掛掉：繼電器不再被持有，光達等同隨時可能斷電，一樣要 deactivate
        if (gpio_pid_ > 0 && waitpid(gpio_pid_, nullptr, WNOHANG) == gpio_pid_) {
            RCLCPP_ERROR(get_logger(), "lidar_gpio.py died unexpectedly (PID %d)! LiDAR power is no longer held. Triggering deactivate.", gpio_pid_);
            gpio_pid_ = -1;
            monitor_timer_.reset();
            this->deactivate();
            return;
        }
    }

    // 關閉順序固定：先關 driver 再斷電，避免 driver 還在讀一台已經沒電的光達
    void stop_all() {
        stop_process(child_pid_, "Livox MID360 driver");
        stop_process(gpio_pid_, "lidar_gpio");
    }

    void stop_process(pid_t & pid, const std::string & name) {
        if (pid <= 0) return;

        RCLCPP_INFO(get_logger(), "Sending SIGINT to %s process group %d", name.c_str(), pid);
        kill(-pid, SIGINT);

        // 最多等 3 秒讓節點自己關閉
        for (int i = 0; i < 30; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            int status;
            if (waitpid(pid, &status, WNOHANG) == pid) {
                RCLCPP_INFO(get_logger(), "%s stopped cleanly.", name.c_str());
                pid = -1;
                return;
            }
        }

        // 超時就強制殺
        RCLCPP_WARN(get_logger(), "Timeout waiting for %s, sending SIGKILL.", name.c_str());
        kill(-pid, SIGKILL);
        waitpid(pid, nullptr, 0);
        pid = -1;
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
