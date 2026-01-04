"""
调试IMU原始数据
===============
显示所有收到的原始数据，不进行解析，用于查看实际数据格式
"""

import asyncio
from bleak import BleakClient
from datetime import datetime

# 设备地址
DEVICE_ADDRESS = "CB:D2:FB:A7:17:66"

# Nordic UART Service UUIDs
UART_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

class RawDataLogger:
    def __init__(self):
        self.client = None
        self.count = 0
        self.last_data = None
        
    def notification_handler(self, sender, data):
        """记录所有收到的数据"""
        self.count += 1
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # 检查是否与上次相同
        is_duplicate = (data == self.last_data)
        self.last_data = data
        
        print(f"\n[{timestamp}] 数据 #{self.count} (长度: {len(data)} 字节)")
        print(f"  Hex: {data.hex()}")
        print(f"  Hex (空格分隔): {' '.join([data[i:i+1].hex() for i in range(len(data))])}")
        
        # 尝试显示为文本
        try:
            text = data.decode('utf-8', errors='ignore')
            if text and text.isprintable():
                print(f"  文本: {repr(text)}")
        except:
            pass
        
        # 显示字节值
        byte_values = [f"{b:3d}" for b in data]
        print(f"  字节值: {' '.join(byte_values)}")
        
        # 尝试解析为整数
        if len(data) >= 2:
            try:
                int16_values = []
                for i in range(0, len(data) - 1, 2):
                    val = int.from_bytes(data[i:i+2], byteorder='little', signed=True)
                    int16_values.append(val)
                if int16_values:
                    print(f"  int16 (小端): {int16_values}")
            except:
                pass
        
        if len(data) >= 4:
            try:
                int32_values = []
                for i in range(0, len(data) - 3, 4):
                    val = int.from_bytes(data[i:i+4], byteorder='little', signed=True)
                    int32_values.append(val)
                if int32_values:
                    print(f"  int32 (小端): {int32_values}")
            except:
                pass
        
        if len(data) >= 4:
            try:
                import struct
                float_values = []
                for i in range(0, len(data) - 3, 4):
                    val = struct.unpack('<f', data[i:i+4])[0]
                    float_values.append(val)
                if float_values:
                    print(f"  float32 (小端): {[f'{v:.6f}' for v in float_values]}")
            except:
                pass
        
        if is_duplicate:
            print("  ⚠ 与上次数据相同")
    
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
    print("IMU原始数据调试工具")
    print("=" * 60)
    print("此工具将显示所有收到的原始数据，用于分析数据格式")
    print("=" * 60)
    
    logger = RawDataLogger()
    
    try:
        # 连接设备
        if not await logger.connect():
            return
        
        # 持续监听
        print("等待数据中...")
        while True:
            await asyncio.sleep(1)
            if logger.count > 0:
                if logger.count % 10 == 0:
                    print(f"\n[统计] 已接收 {logger.count} 条数据...")
        
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await logger.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
