"""
监听IMU数据流
=============
简单监听设备发送的所有数据，不做任何命令发送
"""

import asyncio
from bleak import BleakClient
from scipy.spatial.transform import Rotation as R
import struct
import json
from datetime import datetime

# 设备地址
DEVICE_ADDRESS = "CB:D2:FB:A7:17:66"

# Nordic UART Service UUIDs
UART_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # NOTIFY - 接收数据

class DataListener:
    def __init__(self):
        self.client = None
        self.count = 0
        self.last_quat = None
        
    def notification_handler(self, sender, data):
        """处理接收到的数据"""
        self.count += 1
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        print(f"\n[{timestamp}] 数据 #{self.count} (长度: {len(data)} 字节)")
        print(f"  Hex: {data.hex()}")
        
        # 尝试解析为文本
        try:
            text = data.decode('utf-8', errors='ignore').strip()
            if text and len(text) > 0:
                print(f"  文本: {text}")
                # 尝试解析JSON
                try:
                    if text.startswith('{') or text.startswith('['):
                        json_data = json.loads(text)
                        print(f"  JSON: {json_data}")
                        # 查找四元数
                        self._extract_quaternion_from_dict(json_data)
                except:
                    pass
        except:
            pass
        
        # 尝试解析为二进制四元数
        self._try_binary_quaternion(data)
    
    def _extract_quaternion_from_dict(self, data_dict):
        """从字典中提取四元数"""
        # 常见的四元数字段名
        quat_keys = ['quaternion', 'quat', 'q', 'orientation', 'rot', 'rotation']
        
        for key in quat_keys:
            if key in data_dict:
                val = data_dict[key]
                if isinstance(val, (list, tuple)) and len(val) >= 4:
                    try:
                        q = [float(val[0]), float(val[1]), float(val[2]), float(val[3])]
                        self._print_quaternion(q, f"JSON字段 '{key}'")
                        return
                    except:
                        pass
        
        # 查找单独的字段
        for key in ['qx', 'qy', 'qz', 'qw', 'x', 'y', 'z', 'w']:
            if key in data_dict:
                # 尝试找到其他三个分量
                if key == 'qx' or key == 'x':
                    if all(k in data_dict for k in ['qy', 'qz', 'qw']):
                        q = [float(data_dict['qx']), float(data_dict['qy']), 
                             float(data_dict['qz']), float(data_dict['qw'])]
                        self._print_quaternion(q, "JSON字段 (qx,qy,qz,qw)")
                        return
                    elif all(k in data_dict for k in ['y', 'z', 'w']):
                        q = [float(data_dict['x']), float(data_dict['y']), 
                             float(data_dict['z']), float(data_dict['w'])]
                        self._print_quaternion(q, "JSON字段 (x,y,z,w)")
                        return
    
    def _try_binary_quaternion(self, data):
        """尝试解析二进制格式的四元数"""
        formats = [
            # (格式名, 解析函数, 字节数)
            ("int16 (w,x,y,z)", lambda d: struct.unpack('<4h', d[:8]), 8, 32768.0),
            ("int16 (x,y,z,w)", lambda d: struct.unpack('<4h', d[:8]), 8, 32768.0),
            ("float32 (x,y,z,w)", lambda d: struct.unpack('<4f', d[:16]), 16, 1.0),
            ("float32 (w,x,y,z)", lambda d: struct.unpack('<4f', d[:16]), 16, 1.0),
        ]
        
        for fmt_name, unpack_func, min_len, scale in formats:
            if len(data) >= min_len:
                try:
                    values = unpack_func(data)
                    if fmt_name.startswith("int16"):
                        if "w,x,y,z" in fmt_name:
                            w, x, y, z = values
                            q = [x/scale, y/scale, z/scale, w/scale]
                        else:
                            x, y, z, w = values
                            q = [x/scale, y/scale, z/scale, w/scale]
                    else:  # float32
                        if "x,y,z,w" in fmt_name:
                            x, y, z, w = values
                            q = [x, y, z, w]
                        else:
                            w, x, y, z = values
                            q = [x, y, z, w]
                    
                    self._print_quaternion(q, fmt_name)
                    return
                except:
                    continue
    
    def _print_quaternion(self, q, format_name):
        """打印四元数信息"""
        try:
            r = R.from_quat(q)
            euler = r.as_euler('xyz', degrees=True)
            
            # 检查是否与上次不同（避免重复打印）
            if self.last_quat is None or not r.equals(self.last_quat, atol=0.01):
                print(f"  ✓ {format_name} 解析成功:")
                print(f"    四元数: w={q[3]:.4f}, x={q[0]:.4f}, y={q[1]:.4f}, z={q[2]:.4f}")
                print(f"    欧拉角: Roll={euler[0]:.1f}°, Pitch={euler[1]:.1f}°, Yaw={euler[2]:.1f}°")
                self.last_quat = r
        except Exception as e:
            print(f"  ✗ {format_name} 解析失败: {e}")
    
    async def connect(self):
        """连接设备"""
        self.client = BleakClient(DEVICE_ADDRESS)
        await self.client.connect()
        
        if not self.client.is_connected:
            print("连接失败!")
            return False
        
        print(f"已连接到设备: {DEVICE_ADDRESS}")
        
        # 订阅UART TX (接收数据)
        await self.client.start_notify(UART_TX_UUID, self.notification_handler)
        print("已订阅数据流，开始监听...")
        print("(按 Ctrl+C 停止)\n")
        
        return True
    
    async def disconnect(self):
        """断开连接"""
        if self.client and self.client.is_connected:
            await self.client.stop_notify(UART_TX_UUID)
            await self.client.disconnect()
        print(f"\n已断开连接 (总共接收 {self.count} 条数据)")

async def main():
    print("=" * 60)
    print("IMU数据监听器")
    print("=" * 60)
    
    listener = DataListener()
    
    try:
        # 连接设备
        if not await listener.connect():
            return
        
        # 持续监听
        while True:
            await asyncio.sleep(1)
            if listener.count > 0 and listener.count % 10 == 0:
                print(f"\n[统计] 已接收 {listener.count} 条数据...")
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await listener.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
