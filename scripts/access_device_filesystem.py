"""
访问设备文件系统
================
尝试通过串口发送命令来访问设备文件系统
很多嵌入式设备支持通过串口访问文件系统
"""

import serial
import serial.tools.list_ports
import time
import sys

def find_device():
    """查找设备"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "APP" in port.description or "Board" in port.description or "108c:ab3c" in port.hwid:
            return port.device
    return "/dev/ttyACM0"

def send_command(ser, command, wait_time=0.5):
    """发送命令并等待响应"""
    print(f"\n发送命令: {command}")
    ser.write((command + '\r\n').encode('utf-8'))
    time.sleep(wait_time)
    
    response = b""
    start_time = time.time()
    while time.time() - start_time < 2.0:
        if ser.in_waiting > 0:
            response += ser.read(ser.in_waiting)
            time.sleep(0.1)
        else:
            break
    
    if response:
        try:
            text = response.decode('utf-8', errors='ignore')
            print(f"响应: {repr(text)}")
            return text
        except:
            print(f"响应 (hex): {response.hex()}")
            return response
    else:
        print("无响应")
        return None

def main():
    print("=" * 60)
    print("设备文件系统访问工具")
    print("=" * 60)
    
    port = find_device()
    print(f"找到设备: {port}")
    
    # 尝试不同的波特率
    baud_rates = [115200, 9600, 57600, 230400, 460800]
    
    for baud in baud_rates:
        print(f"\n尝试波特率: {baud}")
        try:
            ser = serial.Serial(port, baud, timeout=1)
            time.sleep(0.5)
            ser.reset_input_buffer()
            
            print("\n尝试常见文件系统命令...")
            
            # 常见的文件系统命令
            commands = [
                # 基本命令
                "ls",
                "dir",
                "pwd",
                "cd /",
                "cat /",
                
                # 文件系统相关
                "fs",
                "file",
                "read",
                "list",
                
                # 设备特定命令
                "help",
                "?",
                "info",
                "status",
                
                # AT命令风格
                "AT",
                "AT+FS?",
                "AT+FILE?",
                
                # 其他
                "\r",
                "\n",
            ]
            
            for cmd in commands:
                response = send_command(ser, cmd, wait_time=0.3)
                if response and len(response) > 10:  # 有意义的响应
                    print(f"\n✓ 命令 '{cmd}' 有响应!")
                    break
                time.sleep(0.2)
            
            ser.close()
            print(f"\n波特率 {baud} 测试完成")
            
        except Exception as e:
            print(f"波特率 {baud} 失败: {e}")
            continue
    
    print("\n" + "=" * 60)
    print("提示:")
    print("1. 如果设备有响应，请查看响应内容")
    print("2. 可能需要查看设备文档了解正确的命令格式")
    print("3. 某些设备可能需要特定的初始化序列")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n已停止")
