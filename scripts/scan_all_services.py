"""
扫描设备的所有服务和特征
========================
详细列出所有可用的服务和特征，查找文件系统相关的服务
"""

import asyncio
from bleak import BleakClient

# 设备地址
DEVICE_ADDRESS = "CB:D2:FB:A7:17:66"

# 常见的文件系统相关UUID
FILE_SYSTEM_SERVICE_UUID = "0000fe59-0000-1000-8000-00805f9b34fb"  # Nordic DFU Service
OBEX_SERVICE_UUID = "00001105-0000-1000-8000-00805f9b34fb"  # OBEX Object Push

async def scan_all_services():
    """扫描所有服务和特征"""
    print("=" * 60)
    print("扫描设备所有服务和特征")
    print("=" * 60)
    
    client = None
    try:
        print(f"\n正在连接设备: {DEVICE_ADDRESS}")
        client = BleakClient(DEVICE_ADDRESS)
        await client.connect()
        
        if not client.is_connected:
            print("❌ 连接失败!")
            return
        
        print("✓ 连接成功!\n")
        
        # 获取所有服务
        services = list(client.services)
        print(f"发现 {len(services)} 个服务:\n")
        
        file_services = []
        
        for i, service in enumerate(services, 1):
            print(f"[服务 {i}] {service.uuid}")
            if service.description:
                print(f"  描述: {service.description}")
            
            # 检查是否是文件系统相关
            if "dfu" in service.uuid.lower() or "file" in service.uuid.lower() or "obex" in service.uuid.lower():
                file_services.append(service)
                print(f"  ⭐ 可能是文件系统服务!")
            
            print(f"  特征数量: {len(service.characteristics)}\n")
            
            for j, char in enumerate(service.characteristics, 1):
                props = []
                if "read" in char.properties:
                    props.append("READ")
                if "write" in char.properties:
                    props.append("WRITE")
                if "notify" in char.properties:
                    props.append("NOTIFY")
                if "indicate" in char.properties:
                    props.append("INDICATE")
                
                print(f"    [特征 {j}] {char.uuid}")
                print(f"      属性: {', '.join(props) if props else '无'}")
                if char.description:
                    print(f"      描述: {char.description}")
                
                # 尝试读取可读的特征
                if "read" in char.properties:
                    try:
                        data = await client.read_gatt_char(char.uuid)
                        if len(data) > 0:
                            print(f"      数据 (hex): {data.hex()[:64]}...")
                            print(f"      数据长度: {len(data)} 字节")
                            
                            # 尝试解析为文本
                            try:
                                text = data.decode('utf-8', errors='ignore')
                                if text.isprintable() and len(text) > 0:
                                    print(f"      数据 (文本): {repr(text[:50])}")
                            except:
                                pass
                    except Exception as e:
                        print(f"      读取失败: {e}")
                
                print()
            
            print("-" * 60)
        
        # 总结文件系统服务
        if file_services:
            print(f"\n⭐ 发现 {len(file_services)} 个可能的文件系统服务:")
            for fs in file_services:
                print(f"  - {fs.uuid}: {fs.description}")
        else:
            print("\n⚠ 未发现明显的文件系统服务")
            print("提示: 某些设备可能需要特定的应用或工具来访问文件系统")
        
    except Exception as e:
        print(f"\n❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if client and client.is_connected:
            await client.disconnect()
            print("\n✓ 已断开连接")

if __name__ == "__main__":
    asyncio.run(scan_all_services())
