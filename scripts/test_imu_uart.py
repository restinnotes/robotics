"""
测试IMU固件通过Nordic UART Service获取四元数
===========================================
Nordic UART Service通常用于传输串口数据，需要：
1. 通过RX特征发送命令启动数据流
2. 通过TX特征接收数据
"""

import asyncio
from bleak import BleakClient
from scipy.spatial.transform import Rotation as R
import struct
import json

# 设备地址
DEVICE_ADDRESS = "CB:D2:FB:A7:17:66"

# Nordic UART Service UUIDs
UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # NOTIFY - 接收数据
UART_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # WRITE - 发送命令

class IMUReceiver:
    def __init__(self):
        self.client = None
        self.raw_quat = R.identity()
        self.data_received = False
        self.received_data = []
        
    def notification_handler(self, sender, data):
        """处理接收到的数据"""
        self.data_received = True
        self.received_data.append(data)
        
        print(f"\n收到数据 (长度: {len(data)} 字节):")
        print(f"  Hex: {data.hex()}")
        print(f"  Raw: {data}")
        
        # 尝试多种数据格式解析
        self._try_parse_data(data)
    
    def _try_parse_data(self, data):
        """尝试解析不同格式的数据"""
        # 格式1: JSON字符串
        try:
            text = data.decode('utf-8', errors='ignore')
            if text.strip().startswith('{') or text.strip().startswith('['):
                try:
                    json_data = json.loads(text)
                    print(f"  JSON解析: {json_data}")
                    if isinstance(json_data, dict):
                        # 查找四元数字段
                        for key in ['quaternion', 'quat', 'q', 'orientation', 'rot']:
                            if key in json_data:
                                q_data = json_data[key]
                                if isinstance(q_data, (list, tuple)) and len(q_data) >= 4:
                                    q = [float(q_data[0]), float(q_data[1]), float(q_data[2]), float(q_data[3])]
                                    self._print_quaternion(q, "JSON")
                                    return
                except:
                    pass
        except:
            pass
        
        # 格式2: 4个int16 (w, x, y, z) 小端序 - 8字节
        if len(data) >= 8:
            try:
                w, x, y, z = struct.unpack('<4h', data[:8])
                scale = 1.0 / 32768.0
                q = [x * scale, y * scale, z * scale, w * scale]
                self._print_quaternion(q, "int16 (w,x,y,z)")
            except:
                pass
        
        # 格式3: 4个int16 (x, y, z, w) 小端序 - 8字节
        if len(data) >= 8:
            try:
                x, y, z, w = struct.unpack('<4h', data[:8])
                scale = 1.0 / 32768.0
                q = [x * scale, y * scale, z * scale, w * scale]
                self._print_quaternion(q, "int16 (x,y,z,w)")
            except:
                pass
        
        # 格式4: 4个float32 (x, y, z, w) 小端序 - 16字节
        if len(data) >= 16:
            try:
                x, y, z, w = struct.unpack('<4f', data[:16])
                q = [x, y, z, w]
                self._print_quaternion(q, "float32 (x,y,z,w)")
            except:
                pass
        
        # 格式5: 4个float32 (w, x, y, z) 小端序 - 16字节
        if len(data) >= 16:
            try:
                w, x, y, z = struct.unpack('<4f', data[:16])
                q = [x, y, z, w]
                self._print_quaternion(q, "float32 (w,x,y,z)")
            except:
                pass
        
        # 格式6: 文本格式 "q: x, y, z, w"
        try:
            text = data.decode('utf-8', errors='ignore')
            if 'q:' in text.lower() or 'quat' in text.lower():
                print(f"  文本格式: {text}")
        except:
            pass
    
    def _print_quaternion(self, q, format_name):
        """打印四元数信息"""
        try:
            r = R.from_quat(q)
            euler = r.as_euler('xyz', degrees=True)
            print(f"  ✓ {format_name} 格式解析成功:")
            print(f"    四元数: w={q[3]:.4f}, x={q[0]:.4f}, y={q[1]:.4f}, z={q[2]:.4f}")
            print(f"    欧拉角: Roll={euler[0]:.1f}°, Pitch={euler[1]:.1f}°, Yaw={euler[2]:.1f}°")
            self.raw_quat = r
        except Exception as e:
            print(f"  ✗ {format_name} 格式解析失败: {e}")
    
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
        print("已订阅UART TX数据流")
        
        return True
    
    async def send_command(self, command):
        """通过UART RX发送命令"""
        if not self.client or not self.client.is_connected:
            print("设备未连接!")
            return
        
        # 命令可以是字符串或字节
        if isinstance(command, str):
            command = command.encode('utf-8')
        
        print(f"发送命令: {command.hex() if isinstance(command, bytes) else command}")
        await self.client.write_gatt_char(UART_RX_UUID, command)
    
    async def disconnect(self):
        """断开连接"""
        if self.client and self.client.is_connected:
            await self.client.stop_notify(UART_TX_UUID)
            await self.client.disconnect()
        print("已断开连接")

async def main():
    print("=" * 60)
    print("IMU固件 UART 测试")
    print("=" * 60)
    
    receiver = IMUReceiver()
    
    try:
        # 连接设备
        if not await receiver.connect():
            return
        
        # 尝试发送一些常见命令来启动数据流
        print("\n尝试发送命令启动数据流...")
        
        # 常见命令（根据固件不同可能需要调整）
        commands = [
            b'start',           # 启动
            b'1',               # 简单命令
            b'q',               # 查询四元数
            b'quat',            # 四元数命令
            b'stream',          # 流式传输
            b'\x01',            # 二进制命令
            b'{"cmd":"start"}', # JSON命令
        ]
        
        for cmd in commands:
            await receiver.send_command(cmd)
            await asyncio.sleep(0.5)  # 等待响应
            
            if receiver.data_received:
                print(f"\n✓ 命令 '{cmd}' 触发了数据流!")
                break
        
        # 如果没有收到数据，继续等待
        if not receiver.data_received:
            print("\n等待数据... (15秒)")
            print("提示: 如果设备需要特定命令，请查看固件文档")
        
        # 持续监听数据
        for i in range(30):  # 30秒
            await asyncio.sleep(1)
            if receiver.data_received and len(receiver.received_data) > 0:
                # 每5秒显示一次统计
                if i % 5 == 0:
                    print(f"\n已接收 {len(receiver.received_data)} 条数据")
        
        # 显示统计
        print(f"\n总共接收 {len(receiver.received_data)} 条数据")
        
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await receiver.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
