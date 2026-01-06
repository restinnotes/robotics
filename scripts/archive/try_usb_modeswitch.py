"""
尝试USB模式切换
===============
某些设备需要切换到Mass Storage模式才能访问文件系统
"""

import subprocess
import sys

def try_modeswitch():
    print("=" * 60)
    print("尝试USB模式切换")
    print("=" * 60)
    
    # 查找设备
    result = subprocess.run(['lsusb'], capture_output=True, text=True)
    lines = result.stdout.split('\n')
    
    device_line = None
    for line in lines:
        if '108c:ab3c' in line or 'APP' in line:
            device_line = line
            break
    
    if not device_line:
        print("未找到设备")
        return
    
    print(f"找到设备: {device_line}")
    
    # 提取Bus和Device号
    parts = device_line.split()
    bus = parts[1]
    device = parts[3].rstrip(':')
    
    print(f"Bus: {bus}, Device: {device}")
    
    # 检查usb_modeswitch
    if subprocess.run(['which', 'usb_modeswitch'], capture_output=True).returncode != 0:
        print("\n未安装usb_modeswitch")
        print("安装方法: sudo apt-get install usb-modeswitch")
        return
    
    print("\n尝试模式切换...")
    print("(这可能需要root权限)")
    
    # 尝试切换
    cmd = ['sudo', 'usb_modeswitch', '-v', '0x108c', '-p', '0xab3c', '-M', '55534243123456780000000000000011062000000000000000000000000000']
    
    print(f"运行命令: {' '.join(cmd)}")
    print("(如果失败，可能需要查看设备特定的切换命令)")
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        print(f"输出: {result.stdout}")
        if result.stderr:
            print(f"错误: {result.stderr}")
    except Exception as e:
        print(f"执行失败: {e}")
        print("\n提示: 需要root权限，或者设备不支持模式切换")

if __name__ == "__main__":
    try_modeswitch()
