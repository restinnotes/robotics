"""
测试Nordic UART IMU接收器
==========================
使用NordicUARTIMUReceiver类连接设备并获取四元数数据
"""

import asyncio
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.ble_imu_receiver import NordicUARTIMUReceiver

# 设备地址
DEVICE_ADDRESS = "CB:D2:FB:A7:17:66"

async def test_receiver(start_command=None):
    """测试接收器"""
    print("=" * 60)
    print("Nordic UART IMU 接收器测试")
    print("=" * 60)
    
    # 创建接收器
    receiver = NordicUARTIMUReceiver(
        address=DEVICE_ADDRESS,
        start_command=start_command
    )
    
    try:
        # 连接
        print(f"\n正在连接设备: {DEVICE_ADDRESS}")
        if not await receiver.connect():
            print("连接失败!")
            return
        
        # 等待数据
        print("\n等待数据... (5秒)")
        await asyncio.sleep(5)
        
        # 检查是否收到数据
        if not receiver._data_received:
            print("\n⚠ 未收到数据")
            print("提示: 设备可能需要特定的启动命令")
            print("可以尝试修改 start_command 参数")
            
            # 尝试发送一些常见命令
            print("\n尝试发送常见命令...")
            commands = [b'start', b'1', b'q', b'quat', b'stream', b'\x01']
            for cmd in commands:
                print(f"  发送: {cmd}")
                await receiver.send_command(cmd)
                await asyncio.sleep(2)
                
                if receiver._data_received:
                    print(f"  ✓ 命令 '{cmd}' 触发了数据流!")
                    break
        else:
            print("\n✓ 已收到数据!")
        
        # 如果收到数据，进行校准并显示
        if receiver._data_received:
            print("\n进行校准...")
            receiver.calibrate()
            
            print("\n显示实时四元数数据 (10秒)...")
            for i in range(100):
                euler = receiver.get_euler_degrees()
                if euler:
                    print(f"  Roll: {euler[0]:6.1f}°, Pitch: {euler[1]:6.1f}°, Yaw: {euler[2]:6.1f}°", end='\r')
                await asyncio.sleep(0.1)
            print()  # 换行
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await receiver.disconnect()

async def main():
    # 测试1: 不发送启动命令（设备可能自动发送数据）
    print("\n[测试1] 不发送启动命令")
    await test_receiver(start_command=None)
    
    await asyncio.sleep(2)
    
    # 测试2: 发送启动命令
    print("\n[测试2] 发送 'start' 命令")
    await test_receiver(start_command=b'start')
    
    await asyncio.sleep(2)
    
    # 测试3: 发送其他命令
    print("\n[测试3] 发送 '1' 命令")
    await test_receiver(start_command=b'1')

if __name__ == "__main__":
    asyncio.run(main())
