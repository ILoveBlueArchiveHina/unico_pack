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

# 以下是測試區塊，只有直接執行此檔案時才會運作
#if __name__ == "__main__":
#    lidar = LidarPower()
#    try:
#        print("測試模式：開啟光達 5 秒...")
#        lidar.turn_on()
#        time.sleep(5)
#        print("測試結束：關閉光達")
#        lidar.turn_off()
#        GPIO.cleanup()
#    except KeyboardInterrupt:
#        lidar.turn_off()
#        GPIO.cleanup()
if __name__ == "__main__":
    lidar = LidarPower()
    try:
        print("測試開始：正在開啟光達電源...")
        lidar.turn_on()
        
        # 【關鍵修改】這行會讓程式「卡住」，直到你按下 Enter
        # 在你按下 Enter 之前，繼電器都必須保持吸合 (通電)
        input(">>> 繼電器現在應該是【開啟】狀態。確認無誤後，請按 [Enter] 鍵來關閉...")

        print("測試結束：正在關閉...")
        lidar.turn_off()
        GPIO.cleanup()
        
    except KeyboardInterrupt:
        lidar.turn_off()
        GPIO.cleanup()