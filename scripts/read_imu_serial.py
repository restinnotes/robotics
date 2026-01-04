"""
通过串口读取IMU数据
==================
设备在Linux上显示为 /dev/ttyACM0，可以通过串口直接读取数据
"""

import serial
import serial.tools.list_ports
import time
import struct
import json
from scipy.spatial.transform import Rotation as R

# 串口设备路径
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200  # 常见波特率，可能需要调整

def find_imu_port():
    """查找IMU设备的串口"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "APP" in port.description or "Board" in port.description or "108c:ab3c" in port.hwid:
            return port.device
    return None

def parse_quaternion(data):
    """尝试解析四元数数据"""
    # 格式1: JSON
    try:
        text = data.decode('utf-8', errors='ignore').strip()
        if text.startswith('{') or text.startswith('['):
            json_data = json.loads(text)
            if isinstance(json_data, dict):
                # 查找四元数字段
                for key in ['quaternion', 'quat', 'q', 'orientation']:
                    if key in json_data:
                        val = json_data[key]
                        if isinstance(val, (list, tuple)) and len(val) >= 4:
                            return [float(val[0]), float(val[1]), float(val[2]), float(val[3])]
                
                # 查找单独字段
                if all(k in json_data for k in ['qx', 'qy', 'qz', 'qw']):
                    return [float(json_data['qx']), float(json_data['qy']), 
                           float(json_data['qz']), float(json_data['qw'])]
    except:
        pass
    
    # 格式2: 二进制 int16 (8字节)
    if len(data) >= 8:
        try:
            w, x, y, z = struct.unpack('<4h', data[:8])
            scale = 1.0 / 32768.0
            return [x * scale, y * scale, z * scale, w * scale]
        except:
            pass
    
    # 格式3: 二进制 float32 (16字节)
    if len(data) >= 16:
        try:
            x, y, z, w = struct.unpack('<4f', data[:16])
            return [x, y, z, w]
        except:
            pass
    
    return None

def main():
    print("=" * 60)
    print("串口IMU数据读取工具")
    print("=" * 60)
    
    # 查找设备
    port = find_imu_port()
    if not port:
        port = SERIAL_PORT
        print(f"使用默认串口: {port}")
    else:
        print(f"找到设备: {port}")
    
    try:
        # 打开串口
        print(f"\n正在打开串口 {port} (波特率: {BAUD_RATE})...")
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print("✓ 串口已打开\n")
        
        # 清空缓冲区
        ser.reset_input_buffer()
        time.sleep(0.1)
        
        print("开始读取数据... (按 Ctrl+C 停止)\n")
        print("-" * 60)
        
        buffer = b""
        count = 0
        last_quat = None
        
        while True:
            # 读取数据
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                buffer += data
                
                # 尝试按行解析（如果数据是文本格式）
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    if len(line) > 0:
                        count += 1
                        print(f"\n[数据 #{count}] (长度: {len(line)} 字节)")
                        print(f"  Hex: {line.hex()[:64]}...")
                        
                        # 尝试解析为文本
                        try:
                            text = line.decode('utf-8', errors='ignore').strip()
                            if text:
                                print(f"  文本: {repr(text)}")
                        except:
                            pass
                        
                        # 尝试解析四元数
                        quat = parse_quaternion(line)
                        if quat:
                            try:
                                r = R.from_quat(quat)
                                euler = r.as_euler('xyz', degrees=True)
                                print(f"  ✓ 四元数: w={quat[3]:.4f}, x={quat[0]:.4f}, y={quat[1]:.4f}, z={quat[2]:.4f}")
                                print(f"    欧拉角: Roll={euler[0]:.1f}°, Pitch={euler[1]:.1f}°, Yaw={euler[2]:.1f}°")
                                last_quat = quat
                            except Exception as e:
                                print(f"  ✗ 解析失败: {e}")
                else:
                    # 如果没有换行符，尝试按固定长度解析（二进制格式）
                    if len(buffer) >= 16:  # 假设至少16字节
                        quat = parse_quaternion(buffer[:16])
                        if quat:
                            count += 1
                            try:
                                r = R.from_quat(quat)
                                euler = r.as_euler('xyz', degrees=True)
                                print(f"\n[数据 #{count}] 四元数: w={quat[3]:.4f}, x={quat[0]:.4f}, y={quat[1]:.4f}, z={quat[2]:.4f}")
                                print(f"  欧拉角: Roll={euler[0]:.1f}°, Pitch={euler[1]:.1f}°, Yaw={euler[2]:.1f}°")
                                buffer = buffer[16:]  # 移除已处理的数据
                            except:
                                pass
            else:
                time.sleep(0.01)
    
    except serial.SerialException as e:
        print(f"\n❌ 串口错误: {e}")
        print("\n提示:")
        print("1. 检查设备是否已连接")
        print("2. 检查是否有其他程序占用串口")
        print("3. 尝试运行: sudo chmod 666 /dev/ttyACM0")
    except KeyboardInterrupt:
        print(f"\n\n已停止 (总共读取 {count} 条数据)")
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("✓ 串口已关闭")

if __name__ == "__main__":
    main()
