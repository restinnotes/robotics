"""
测试IMU固件连接并获取四元数数据
================================
连接BLE设备，查看所有服务和特征，然后尝试读取四元数数据
"""

import asyncio
from bleak import BleakClient, BleakScanner
from scipy.spatial.transform import Rotation as R
import struct

# 设备地址（从扫描结果获取）
DEVICE_ADDRESS = "CB:D2:FB:A7:17:66"

async def discover_services_and_characteristics(address):
    """发现设备的所有服务和特征"""
    print(f"正在连接设备: {address}")
    async with BleakClient(address) as client:
        print(f"已连接! 设备名称: {client.is_connected}")
        
        # 获取所有服务
        services = list(client.services)
        print(f"\n发现 {len(services)} 个服务:\n")
        
        for service in services:
            print(f"服务 UUID: {service.uuid}")
            print(f"  描述: {service.description}")
            print(f"  特征数量: {len(service.characteristics)}")
            
            for char in service.characteristics:
                props = []
                if "read" in char.properties:
                    props.append("READ")
                if "write" in char.properties:
                    props.append("WRITE")
                if "notify" in char.properties:
                    props.append("NOTIFY")
                if "indicate" in char.properties:
                    props.append("INDICATE")
                
                print(f"    特征 UUID: {char.uuid}")
                print(f"      属性: {', '.join(props)}")
                print(f"      描述: {char.description}")
                
                # 尝试读取可读的特征
                if "read" in char.properties:
                    try:
                        data = await client.read_gatt_char(char.uuid)
                        if len(data) > 0:
                            print(f"      数据 (hex): {data.hex()}")
                            print(f"      数据长度: {len(data)} 字节")
                    except Exception as e:
                        print(f"      读取失败: {e}")
                print()
            
            print("-" * 60)

async def test_quaternion_reading(address, quaternion_uuid=None):
    """测试读取四元数数据"""
    print(f"\n正在测试四元数读取...")
    
    quaternion_data = None
    last_quat = None
    
    def notification_handler(sender, data):
        nonlocal quaternion_data, last_quat
        quaternion_data = data
        print(f"收到数据: {data.hex()} (长度: {len(data)} 字节)")
        
        # 尝试解析为四元数（常见格式）
        if len(data) >= 8:
            # 格式1: 4个int16 (w, x, y, z) 小端序
            try:
                w, x, y, z = struct.unpack('<4h', data[:8])
                scale = 1.0 / 32768.0
                q = [x * scale, y * scale, z * scale, w * scale]
                r = R.from_quat(q)
                euler = r.as_euler('xyz', degrees=True)
                print(f"  解析为四元数 (int16): w={q[3]:.4f}, x={q[0]:.4f}, y={q[1]:.4f}, z={q[2]:.4f}")
                print(f"  欧拉角: Roll={euler[0]:.1f}°, Pitch={euler[1]:.1f}°, Yaw={euler[2]:.1f}°")
            except:
                pass
            
            # 格式2: 4个float32 (x, y, z, w) 小端序
            try:
                x, y, z, w = struct.unpack('<4f', data[:16])
                q = [x, y, z, w]
                r = R.from_quat(q)
                euler = r.as_euler('xyz', degrees=True)
                print(f"  解析为四元数 (float32): w={w:.4f}, x={x:.4f}, y={y:.4f}, z={z:.4f}")
                print(f"  欧拉角: Roll={euler[0]:.1f}°, Pitch={euler[1]:.1f}°, Yaw={euler[2]:.1f}°")
            except:
                pass
    
    async with BleakClient(address) as client:
        if quaternion_uuid:
            # 如果提供了UUID，直接订阅
            print(f"订阅特征: {quaternion_uuid}")
            await client.start_notify(quaternion_uuid, notification_handler)
        else:
            # 否则查找所有支持notify的特征并订阅
            services = list(client.services)
            notify_chars = []
            for service in services:
                for char in service.characteristics:
                    if "notify" in char.properties:
                        notify_chars.append((service.uuid, char.uuid))
            
            print(f"发现 {len(notify_chars)} 个支持通知的特征，将全部订阅...")
            for service_uuid, char_uuid in notify_chars:
                print(f"  订阅: {char_uuid}")
                await client.start_notify(char_uuid, notification_handler)
        
        print("\n等待数据... (10秒)")
        await asyncio.sleep(10)
        
        # 停止通知
        if quaternion_uuid:
            await client.stop_notify(quaternion_uuid)
        else:
            for service_uuid, char_uuid in notify_chars:
                await client.stop_notify(char_uuid)

async def main():
    print("=" * 60)
    print("IMU固件测试工具")
    print("=" * 60)
    
    # 步骤1: 发现服务和特征
    print("\n[步骤1] 发现设备服务和特征")
    await discover_services_and_characteristics(DEVICE_ADDRESS)
    
    # 步骤2: 测试读取四元数（需要手动指定UUID或自动订阅所有notify特征）
    print("\n[步骤2] 测试四元数数据读取")
    print("提示: 如果知道四元数特征的UUID，可以修改脚本中的 quaternion_uuid 参数")
    await test_quaternion_reading(DEVICE_ADDRESS)

if __name__ == "__main__":
    asyncio.run(main())
