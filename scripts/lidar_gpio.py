#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import time

class LidarPower:
    def __init__(self, pin=7):
        """初始化光達電源控制 (預設 Pin 7)"""
        self.pin = pin
        self.is_on = False
        
        # 設定 GPIO 模式
        # 注意：如果在主程式已經設定過 setmode，這行其實是重複的，但為了保險起見保留
        try:
            GPIO.setmode(GPIO.BOARD)
        except:
            pass
            
        GPIO.setwarnings(False)
        
        # 初始化為 LOW (斷電狀態 - 配合常開繼電器 NO)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        print(f"[LiDAR GPIO] Initialized on Pin {self.pin} (State: OFF)")

    def turn_on(self):
        """開啟光達電源 (持續發送 HIGH)"""
        if not self.is_on:
            GPIO.output(self.pin, GPIO.HIGH)
            self.is_on = True
            print("[LiDAR GPIO] Power ON (Signal: HIGH)")
        else:
            print("[LiDAR GPIO] Already ON")

    def turn_off(self):
        """關閉光達電源 (發送 LOW)"""
        if self.is_on:
            GPIO.output(self.pin, GPIO.LOW)
            self.is_on = False
            print("[LiDAR GPIO] Power OFF (Signal: LOW)")
        else:
            print("[LiDAR GPIO] Already OFF")

# 以下是執行區塊，只有直接執行此檔案時才會運作
if __name__ == "__main__":
    import signal
    import sys

    # 互動測試模式（手動測繼電器用）：按 Enter 才斷電
    if "--interactive" in sys.argv:
        lidar = LidarPower()
        try:
            print("測試開始：正在開啟光達電源...")
            lidar.turn_on()
            # 這行會讓程式「卡住」，直到按下 Enter，期間繼電器都保持吸合（通電）
            input(">>> 繼電器現在應該是【開啟】狀態。確認無誤後，請按 [Enter] 鍵來關閉...")
            print("測試結束：正在關閉...")
        except (KeyboardInterrupt, EOFError):
            print("")
        finally:
            lidar.turn_off()
            GPIO.cleanup()
        sys.exit(0)

    # 常駐模式（livox_lifecycle 會用這個模式把它 fork 出來）：
    # 開機後一直持有 GPIO 為 HIGH，收到 SIGINT / SIGTERM 才斷電結束。
    # 因為「這支程式活著」就等於「光達有電」，lifecycle 才能用它的存活狀態判斷供電是否正常。
    running = True

    def _on_signal(_signum, _frame):
        global running
        running = False

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    lidar = LidarPower()
    try:
        lidar.turn_on()
        print("[LiDAR GPIO] Holding power ON, waiting for SIGINT/SIGTERM...", flush=True)
        while running:
            time.sleep(0.5)
    finally:
        lidar.turn_off()
        GPIO.cleanup()
