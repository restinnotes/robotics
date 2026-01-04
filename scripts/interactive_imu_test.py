"""
交互式IMU测试工具
=================
连接设备，实时显示四元数数据，支持手动发送命令
"""

import asyncio
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.ble_imu_receiver import NordicUARTIMUReceiver

# 设备地址
DEVICE_ADDRESS = "CB:D2:FB:A7:17:66"

async def interactive_test():
    """交互式测试"""
    print("=" * 60)
    print("交互式 IMU 测试工具")
    print("=" * 60)
    print(f"设备地址: {DEVICE_ADDRESS}")
    print("\n提示:")
    print("  - 按 Enter 进行校准")
    print("  - 输入命令发送到设备 (如: start, 1, q 等)")
    print("  - 输入 'quit' 退出")
    print("=" * 60)
    
    # 创建接收器（不自动发送命令）
    receiver = NordicUARTIMUReceiver(
        address=DEVICE_ADDRESS,
        start_command=None
    )
    
    try:
        # 连接
        print(f"\n正在连接设备...")
        if not await receiver.connect():
            print("连接失败!")
            return
        
        print("已连接! 等待数据...\n")
        
        # 创建任务来显示实时数据
        display_task = asyncio.create_task(display_data(receiver))
        
        # 主循环：处理用户输入
        loop = asyncio.get_event_loop()
        
        while True:
            # 使用 asyncio 等待用户输入（非阻塞）
            user_input = await loop.run_in_executor(None, input, "> ")
            
            if user_input.strip() == "":
                # 空输入 = 校准
                receiver.calibrate()
            elif user_input.strip().lower() == "quit":
                break
            else:
                # 发送命令
                cmd = user_input.strip()
                print(f"发送命令: {cmd}")
                await receiver.send_command(cmd)
        
        # 取消显示任务
        display_task.cancel()
        try:
            await display_task
        except asyncio.CancelledError:
            pass
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await receiver.disconnect()
        print("\n已断开连接")

async def display_data(receiver):
    """后台任务：显示实时数据"""
    last_count = 0
    while True:
        await asyncio.sleep(0.5)
        
        if receiver._data_received:
            euler = receiver.get_euler_degrees()
            if euler:
                # 只在数据更新时显示
                current_count = receiver._data_received
                if current_count != last_count:
                    print(f"\r[数据] Roll: {euler[0]:6.1f}°, Pitch: {euler[1]:6.1f}°, Yaw: {euler[2]:6.1f}°", end='', flush=True)
                    last_count = current_count
        else:
            # 显示等待状态
            print(f"\r[等待数据...]", end='', flush=True)

if __name__ == "__main__":
    try:
        asyncio.run(interactive_test())
    except KeyboardInterrupt:
        print("\n\n退出")
