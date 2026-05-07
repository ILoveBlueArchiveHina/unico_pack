#!/bin/bash
# ==============================================================================
# 室內無人機系統啟動腳本 (安全修復版)
# ==============================================================================

# 配置參數
MAX_FASTLIO_RETRIES=10
FASTLIO_ERROR_WINDOW=5
CLOCKS_BACKUP="/tmp/jetson_clocks.conf"

# ==============================================================================
# 清理函數 (安全版)
# ==============================================================================
cleanup_routine() {
    # 【關鍵修正 1】：立刻解除 trap！防止腳本收到自己的訊號而引發無限迴圈
    trap - SIGINT SIGTERM EXIT
    
    echo ""
    echo "[EXIT] Stopping script's child processes gracefully..."
    
    # 恢復時脈
    if [ -f "$CLOCKS_BACKUP" ]; then
        echo "[EXIT] Restoring clock settings..."
        sudo /usr/bin/jetson_clocks --restore "$CLOCKS_BACKUP"
        sudo rm -f "$CLOCKS_BACKUP"
    fi
    
    # 【關鍵修正 2】：只對這個腳本放到背景執行的任務 (jobs) 發送 SIGINT
    # 這樣就不會炸到腳本自己，也不會干擾其他終端機的程序
    kill -INT $(jobs -p) 2>/dev/null
    
    # 給予 3 秒鐘的優雅關機時間讓 ROS 2 處理
    sleep 3
    
    # 如果還有沒死透的，進行精準狙擊
    echo "[EXIT] Force killing remaining specific nodes..."
    killall -9 mavros_node fastlio_mapping 2>/dev/null
    pkill -9 -f "msg_MID360_launch" 2>/dev/null
    pkill -9 -f "mapping.launch.py" 2>/dev/null
    pkill -9 -f "livox_ros_driver2" 2>/dev/null
    
    echo "[EXIT] Done."
    exit 0
}
# 綁定中斷訊號
trap cleanup_routine SIGINT SIGTERM EXIT

# ==============================================================================
# 啟動前清理 (安全版)
# ==============================================================================
echo "[Init] Cleanup specific previous runs..."
# 只清除自己管轄的執行檔，不要清空共享記憶體，不要停用 daemon
killall -9 mavros_node fastlio_mapping 2>/dev/null
pkill -9 -f "livox" 2>/dev/null
pkill -9 -f "fast_lio" 2>/dev/null
sleep 1

# ==============================================================================
# 鎖定最高時脈
# ==============================================================================
echo "[Init] Locking max clock..."
if command -v jetson_clocks &> /dev/null; then
    # 用 sudo 刪除舊的備份檔
    sudo rm -f "$CLOCKS_BACKUP"
    # 備份當前設定
    sudo /usr/bin/jetson_clocks --store "$CLOCKS_BACKUP"
    # 鎖定最高頻率
    sudo /usr/bin/jetson_clocks
    echo "   -> Jetson Clocks: ENABLED"
fi

# ==============================================================================
# 載入 ROS2
# ==============================================================================
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# ==============================================================================
# Step 1: MAVROS
# ==============================================================================
echo ""
echo "[Step 1] Starting MAVROS..."
taskset -c 4,5 ros2 launch kuo_pack mavros.launch.py &
sleep 5

# ==============================================================================
# Step 2: LiDAR
# ==============================================================================
echo ""
echo "[Step 2] Starting LiDAR..."
taskset -c 4,5 ros2 launch livox_ros_driver2 msg_MID360_launch.py &
sleep 5
echo "   -> LiDAR started."

# ==============================================================================
# Step 3: Fast-LIO
# ==============================================================================
echo ""
echo "[Step 3] Starting Fast-LIO..."

FASTLIO_LOG=$(mktemp)
FASTLIO_OK=false

for ((retry=1; retry<=MAX_FASTLIO_RETRIES; retry++)); do
    echo "   -> Attempt $retry/$MAX_FASTLIO_RETRIES"
    
    > "$FASTLIO_LOG"
    taskset -c 4,5 ros2 launch fast_lio mapping.launch.py 2>&1 | tee "$FASTLIO_LOG" &
    FASTLIO_PID=$!
    
    # 監測錯誤（5 秒）
    ERROR=false
    for ((t=1; t<=FASTLIO_ERROR_WINDOW; t++)); do
        sleep 1
        if grep -q "time diff is\|lidar loop back" "$FASTLIO_LOG" 2>/dev/null; then
            echo "   -> [ERROR] Time sync error!"
            ERROR=true
            break
        fi
        echo -n "."
    done
    echo ""
    
    if [ "$ERROR" = true ]; then
        kill -9 $FASTLIO_PID 2>/dev/null
        pkill -9 -f "fast_lio" 2>/dev/null
        sleep 1
        continue
    fi
    
    # 等待初始化（20 秒）
    echo "   -> Waiting for init..."
    for ((t=1; t<=20; t++)); do
        sleep 1
        if grep -q "time diff is\|lidar loop back" "$FASTLIO_LOG" 2>/dev/null; then
            ERROR=true
            break
        fi
        if grep -q "Initialize the map kdtree" "$FASTLIO_LOG" 2>/dev/null; then
            echo "   -> [OK] Fast-LIO initialized!"
            FASTLIO_OK=true
            break
        fi
    done
    
    if [ "$FASTLIO_OK" = true ]; then
        break
    fi
    
    kill -9 $FASTLIO_PID 2>/dev/null
    pkill -9 -f "fast_lio" 2>/dev/null
    sleep 1
done

rm -f "$FASTLIO_LOG"

if [ "$FASTLIO_OK" = false ]; then
    echo "[ERROR] Fast-LIO failed!"
    exit 1
fi

# ==============================================================================
# Step 4: Vision Bridge
# ==============================================================================
echo ""
echo "[Step 4] Starting Vision Bridge..."
taskset -c 3 ros2 launch kuo_pack fastlio_vision.launch.py &
sleep 2

# ==============================================================================
# 完成
# ==============================================================================
echo ""
echo "============================================================"
echo "[SUCCESS] All Systems Started!"
echo "============================================================"
echo ""
echo "Press Ctrl+C to stop."
echo ""

wait