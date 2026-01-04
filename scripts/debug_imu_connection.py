"""
调试IMU连接和数据接收
=====================
详细检查连接状态、服务和特征，并监听数据
"""

import asyncio
from bleak import BleakClient
from datetime import datetime

# 设备地址
DEVICE_ADDRESS = "CB:D2:FB:A7:17:66"

# Nordic UART Service UUIDs
UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
UART_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

async def debug_connection():
    """详细调试连接和数据"""
    print("=" * 60)
    print("IMU连接调试工具")
    print("=" * 60)
    
    client = None
    data_count = 0
    
    def notification_handler(sender, data):
        nonlocal data_count
        data_count += 1
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"\n[{timestamp}] ✓ 收到数据 #{data_count} (长度: {len(data)} 字节)")
        print(f"  Hex: {data.hex()}")
        print(f"  原始字节: {list(data)}")
        
        # 尝试文本
        try:
            text = data.decode('utf-8', errors='ignore')
            if text.strip():
                print(f"  文本: {repr(text)}")
        except:
            pass
    
    try:
        print(f"\n1. 正在连接设备: {DEVICE_ADDRESS}")
        client = BleakClient(DEVICE_ADDRESS)
        await client.connect()
        
        if not client.is_connected:
            print("❌ 连接失败!")
            return
        
        print("✓ 连接成功!")
        
        # 检查服务
        print(f"\n2. 检查服务...")
        services = list(client.services)
        print(f"   发现 {len(services)} 个服务")
        
        uart_service_found = False
        for service in services:
            if service.uuid.lower() == UART_SERVICE_UUID.lower():
                uart_service_found = True
                print(f"   ✓ 找到 Nordic UART Service: {service.uuid}")
                print(f"     特征数量: {len(service.characteristics)}")
                for char in service.characteristics:
                    print(f"      - {char.uuid}: {char.properties}")
                break
        
        if not uart_service_found:
            print(f"   ⚠ 未找到 Nordic UART Service")
            print(f"   可用服务:")
            for service in services:
                print(f"      - {service.uuid}")
        
        # 订阅通知
        print(f"\n3. 订阅UART TX通知...")
        try:
            await client.start_notify(UART_TX_UUID, notification_handler)
            print(f"   ✓ 已订阅: {UART_TX_UUID}")
        except Exception as e:
            print(f"   ❌ 订阅失败: {e}")
            return
        
        # 检查连接状态
        print(f"\n4. 连接状态检查...")
        print(f"   is_connected: {client.is_connected}")
        
        # 等待数据
        print(f"\n5. 开始监听数据 (30秒)...")
        print("   (如果设备在发送数据，应该会立即看到)")
        print("   " + "-" * 50)
        
        for i in range(30):
            await asyncio.sleep(1)
            if data_count > 0:
                print(f"\n   [第{i+1}秒] 已收到 {data_count} 条数据")
            else:
                print(f"   [第{i+1}秒] 等待中...", end='\r')
        
        print(f"\n\n6. 结果统计:")
        print(f"   总共收到: {data_count} 条数据")
        
        if data_count == 0:
            print("\n   ⚠ 未收到任何数据!")
            print("   可能的原因:")
            print("   1. 设备需要特定命令才能开始发送数据")
            print("   2. 数据发送频率很低")
            print("   3. 特征UUID不正确")
            print("   4. 设备固件需要配置")
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if client and client.is_connected:
            try:
                await client.stop_notify(UART_TX_UUID)
            except:
                pass
            await client.disconnect()
            print("\n✓ 已断开连接")

if __name__ == "__main__":
    asyncio.run(debug_connection())
