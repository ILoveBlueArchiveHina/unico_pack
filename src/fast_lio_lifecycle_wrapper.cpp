#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/select.h>
#include <signal.h>
#include <fcntl.h>
#include <string>
#include <vector>
#include <chrono>

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using namespace std::chrono_literals;

class FastLioLifecycleWrapper : public rclcpp_lifecycle::LifecycleNode {
public:
    FastLioLifecycleWrapper() : rclcpp_lifecycle::LifecycleNode("fastlio_wrapper"), child_pid_(-1) {
        RCLCPP_INFO(get_logger(), "C++ Fast-LIO Wrapper instantiated. State: Unconfigured");
    }

    // 1. 配置階段
    LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) {
        RCLCPP_INFO(get_logger(), "Configuring Fast-LIO Wrapper...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // 2. 啟動階段 (帶有 Pipe 攔截與重試機制)
    LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) {
        RCLCPP_INFO(get_logger(), "Activating Fast-LIO...");
        
        for (int attempt = 1; attempt <= max_retries_; ++attempt) {
            RCLCPP_INFO(get_logger(), "--- Fast-LIO Start Attempt %d/%d ---", attempt, max_retries_);
            
            // 建立 Pipe 用於攔截子行程的標準輸出 (stdout & stderr)
            int pipefd[2];
            if (pipe(pipefd) == -1) {
                RCLCPP_ERROR(get_logger(), "Failed to create pipe.");
                return LifecycleNodeInterface::CallbackReturn::FAILURE;
            }

            child_pid_ = fork();

            if (child_pid_ < 0) {
                RCLCPP_ERROR(get_logger(), "Fork failed!");
                close(pipefd[0]); close(pipefd[1]);
                return LifecycleNodeInterface::CallbackReturn::FAILURE;
            } 
            else if (child_pid_ == 0) {
                // --- 子行程 ---
                setsid(); // 建立新的 Process Group，防殭屍

                // 將子行程的 stdout 和 stderr 導向 Pipe 的寫入端
                dup2(pipefd[1], STDOUT_FILENO);
                dup2(pipefd[1], STDERR_FILENO);
                close(pipefd[0]); // 子行程不需要讀取端
                close(pipefd[1]); 

                // 執行 launch 檔
                execlp("taskset", "taskset", "-c", "4,5",
                        "ros2", "launch", "fast_lio", "mapping.launch.py", nullptr);
                exit(1); 
            }

            // --- 父行程 ---
            close(pipefd[1]); // 父行程不需要寫入端
            int read_fd = pipefd[0];

            // 將讀取端設為非阻塞模式 (Non-blocking)
            int flags = fcntl(read_fd, F_GETFL, 0);
            fcntl(read_fd, F_SETFL, flags | O_NONBLOCK);

            bool startup_success = false;
            bool has_error = false;
            std::string buffer = "";
            char read_buf[256];

            auto start_time = std::chrono::steady_clock::now();

            // 進入 Log 監控迴圈
            while (true) {
                auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count() > init_timeout_sec_) {
                    break; // 檢查超時
                }

                fd_set read_fds;
                FD_ZERO(&read_fds);
                FD_SET(read_fd, &read_fds);

                struct timeval timeout;
                timeout.tv_sec = 0;
                timeout.tv_usec = 200000; // 0.2 秒超時

                // 使用 select 進行非阻塞等待
                int ret = select(read_fd + 1, &read_fds, NULL, NULL, &timeout);

                if (ret > 0 && FD_ISSET(read_fd, &read_fds)) {
                    ssize_t bytes_read = read(read_fd, read_buf, sizeof(read_buf) - 1);
                    if (bytes_read > 0) {
                        read_buf[bytes_read] = '\0';
                        buffer += read_buf;

                        // 逐行解析 Log
                        size_t pos = 0;
                        while ((pos = buffer.find('\n')) != std::string::npos) {
                            std::string line = buffer.substr(0, pos);
                            buffer.erase(0, pos + 1);

                            // 判斷 1: 捕捉到失敗關鍵字
                            if (line.find("time diff is") != std::string::npos || 
                                line.find("lidar loop back") != std::string::npos) {
                                RCLCPP_ERROR(get_logger(), "Detected Time Sync Error! Log: %s", line.c_str());
                                has_error = true;
                                break;
                            }
                            
                            // 判斷 2: 捕捉到成功關鍵字
                            if (line.find("Initialize the map kdtree") != std::string::npos) {
                                RCLCPP_INFO(get_logger(), "Fast-LIO initialized successfully!");
                                startup_success = true;
                                break;
                            }
                        }
                    } else if (bytes_read == 0) {
                        // EOF, subprocess 已經結束
                        break; 
                    }
                }
                
                if (startup_success || has_error) {
                    break;
                }
            }

            close(read_fd);

            // 判斷結果
            if (startup_success) {
                return LifecycleNodeInterface::CallbackReturn::SUCCESS;
            } else {
                if (!has_error) {
                    RCLCPP_WARN(get_logger(), "Timeout waiting for Fast-LIO to initialize.");
                }
                // 清理殘留的 Process，準備下一次重試
                stop_process();
                rclcpp::sleep_for(1s);
            }
        }

        RCLCPP_FATAL(get_logger(), "Fast-LIO failed to start after all retries!");
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // 3. 休眠階段
    LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) {
        RCLCPP_INFO(get_logger(), "Deactivating... Stopping Fast-LIO.");
        stop_process();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) {
        stop_process();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) {
        stop_process();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    pid_t child_pid_;
    const int max_retries_ = 5;
    const int init_timeout_sec_ = 20;

    void stop_process() {
        if (child_pid_ > 0) {
            // 發送 SIGINT 給整個 Process Group
            kill(-child_pid_, SIGINT); 
            
            // 給予最多 3 秒的時間優雅關閉
            int status;
            int wait_count = 0;
            pid_t wpid = 0;
            
            while (wait_count < 30) { // 30 * 100ms = 3s
                wpid = waitpid(child_pid_, &status, WNOHANG);
                if (wpid > 0 || wpid == -1) {
                    break; // 行程已死或不存在
                }
                rclcpp::sleep_for(100ms);
                wait_count++;
            }

            // 如果 3 秒後還是沒死，強制拔管 (SIGKILL)
            if (wpid == 0) {
                RCLCPP_WARN(get_logger(), "Fast-LIO taking too long. Forcing SIGKILL...");
                kill(-child_pid_, SIGKILL);
                waitpid(child_pid_, &status, 0); // 確保資源回收
            } else {
                RCLCPP_INFO(get_logger(), "Fast-LIO stopped gracefully.");
            }
            
            child_pid_ = -1;
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLioLifecycleWrapper>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}