"""
测试串口IMU接收器
================
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.serial_imu_receiver import SerialIMUReceiver
import time

def main():
    print("=" * 60)
    print("串口IMU测试")
    print("=" * 60)
    
    receiver = SerialIMUReceiver()
    
    if receiver.connect():
        print("\n等待数据... (10秒)")
        print("(如果设备在发送数据，应该会立即看到)")
        print("-" * 60)
        
        for i in range(10):
            time.sleep(1)
            if receiver._data_received:
                print(f"\n✓ 已收到数据!")
                receiver.calibrate()
                
                print("\n显示实时四元数数据...")
                for _ in range(50):
                    euler = receiver.get_euler_degrees()
                    if euler:
                        print(f"Roll: {euler[0]:6.1f}°, Pitch: {euler[1]:6.1f}°, Yaw: {euler[2]:6.1f}°", end='\r')
                    time.sleep(0.1)
                print()
                break
            else:
                print(f"[{i+1}/10] 等待中...", end='\r')
        
        if not receiver._data_received:
            print("\n⚠ 未收到数据")
            print("提示: 设备可能:")
            print("  1. 需要特定命令才能开始发送数据")
            print("  2. 数据格式与预期不同")
            print("  3. 波特率不正确（当前: 115200）")
        
        receiver.disconnect()
    else:
        print("\n连接失败!")
        print("\n权限问题解决方案:")
        print("  运行: sudo chmod 666 /dev/ttyACM0")
        print("  或者: sudo usermod -a -G dialout $USER")
        print("        (然后重新登录)")

if __name__ == "__main__":
    main()
