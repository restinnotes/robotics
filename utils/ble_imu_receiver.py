"""
BLE IMU 接收器 (BHI3xx)
=======================
通过 BLE 从 Bosch BHI360/BHI260 传感器接收四元数数据。

数据格式 (Bosch 标准):
- 8 字节: 4 个 int16 (w, x, y, z)
- 归一化: 除以 32768 得到 [-1, 1] 范围

使用方法:
    receiver = BLEIMUReceiver("AA:BB:CC:DD:EE:FF")
    await receiver.connect()
    receiver.calibrate()
    while True:
        orientation = receiver.get_orientation()
"""

import asyncio
import struct
import json
from typing import Optional
from scipy.spatial.transform import Rotation as R

try:
    from bleak import BleakClient, BleakScanner
    BLEAK_AVAILABLE = True
except ImportError:
    BLEAK_AVAILABLE = False
    print("警告: bleak 库未安装，BLE 功能不可用。请运行: pip install bleak")

from utils.imu_data_source import IMUDataSource


# Bosch BHI3xx 通用 GATT UUIDs (可能需要根据实际固件调整)
# 如果您的固件使用不同的 UUID，请在此修改
DEFAULT_SERVICE_UUID = "0000180a-0000-1000-8000-00805f9b34fb"  # Device Information Service (示例)
DEFAULT_QUATERNION_CHAR_UUID = "00002a5d-0000-1000-8000-00805f9b34fb"  # Sensor Location (示例，需替换)

# Nordic UART Service UUIDs
NORDIC_UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NORDIC_UART_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # NOTIFY - 接收数据
NORDIC_UART_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # WRITE - 发送命令


class BLEIMUReceiver(IMUDataSource):
    """
    BLE IMU 接收器，用于接收 BHI3xx 传感器的四元数数据
    """

    def __init__(
        self,
        address: str,
        quaternion_uuid: str = DEFAULT_QUATERNION_CHAR_UUID,
        auto_reconnect: bool = True
    ):
        """
        Args:
            address: BLE 设备地址 (如 "AA:BB:CC:DD:EE:FF")
            quaternion_uuid: 四元数 Characteristic 的 UUID
            auto_reconnect: 是否自动重连
        """
        if not BLEAK_AVAILABLE:
            raise RuntimeError("bleak 库未安装")

        self.address = address
        self.quaternion_uuid = quaternion_uuid
        self.auto_reconnect = auto_reconnect

        # 状态
        self.client: Optional[BleakClient] = None
        self._connected = False
        self._raw_quat = R.identity()  # 原始四元数
        self._calib_inv = R.identity()  # 校准逆
        self._last_update_time = 0

    async def connect(self) -> bool:
        """
        连接到 BLE 设备并订阅四元数通知

        Returns:
            bool: 连接是否成功
        """
        try:
            self.client = BleakClient(self.address)
            await self.client.connect()

            if not self.client.is_connected:
                print(f"无法连接到设备: {self.address}")
                return False

            print(f"已连接到 BLE 设备: {self.address}")

            # 订阅四元数通知
            await self.client.start_notify(self.quaternion_uuid, self._on_quaternion_data)
            self._connected = True
            print(f"已订阅四元数数据 (UUID: {self.quaternion_uuid})")
            return True

        except Exception as e:
            print(f"BLE 连接错误: {e}")
            self._connected = False
            return False

    async def disconnect(self):
        """断开 BLE 连接"""
        if self.client and self.client.is_connected:
            await self.client.disconnect()
        self._connected = False
        print("BLE 连接已断开")

    def _on_quaternion_data(self, sender, data: bytearray):
        """
        处理接收到的四元数数据

        Bosch 格式: 4 个 int16 (w, x, y, z), 小端序
        归一化: 除以 32768
        """
        if len(data) < 8:
            return

        # 解析 4 个 int16
        w, x, y, z = struct.unpack('<4h', data[:8])
        scale = 1.0 / 32768.0

        # 转换为 Scipy Rotation (注意: from_quat 顺序是 [x, y, z, w])
        self._raw_quat = R.from_quat([x * scale, y * scale, z * scale, w * scale])
        self._last_update_time = asyncio.get_event_loop().time()

    def get_orientation(self) -> Optional[R]:
        """获取校准后的相对姿态"""
        if not self._connected:
            return None
        return self._calib_inv * self._raw_quat

    def calibrate(self):
        """将当前姿态设为零位"""
        self._calib_inv = self._raw_quat.inv()
        print(">>> BLE IMU 校准完成!")

    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._connected and self.client is not None and self.client.is_connected


class NordicUARTIMUReceiver(IMUDataSource):
    """
    Nordic UART Service IMU 接收器
    适用于通过Nordic UART Service传输数据的IMU固件
    支持多种数据格式：JSON、二进制int16、二进制float32等
    """
    
    def __init__(
        self,
        address: str,
        start_command: Optional[bytes] = None,
        auto_reconnect: bool = True
    ):
        """
        Args:
            address: BLE 设备地址 (如 "AA:BB:CC:DD:EE:FF")
            start_command: 启动数据流的命令（字节或字符串），如果为None则不发送命令
            auto_reconnect: 是否自动重连
        """
        if not BLEAK_AVAILABLE:
            raise RuntimeError("bleak 库未安装")
        
        self.address = address
        self.start_command = start_command.encode('utf-8') if isinstance(start_command, str) else start_command
        self.auto_reconnect = auto_reconnect
        
        # 状态
        self.client: Optional[BleakClient] = None
        self._connected = False
        self._raw_quat = R.identity()  # 原始四元数
        self._calib_inv = R.identity()  # 校准逆
        self._last_update_time = 0
        self._data_received = False
    
    async def connect(self) -> bool:
        """
        连接到 BLE 设备并订阅UART数据流
        
        Returns:
            bool: 连接是否成功
        """
        try:
            self.client = BleakClient(self.address)
            await self.client.connect()
            
            if not self.client.is_connected:
                print(f"无法连接到设备: {self.address}")
                return False
            
            print(f"已连接到 BLE 设备: {self.address}")
            
            # 订阅UART TX (接收数据)
            await self.client.start_notify(NORDIC_UART_TX_UUID, self._on_data)
            self._connected = True
            print(f"已订阅UART数据流")
            
            # 如果提供了启动命令，发送它
            if self.start_command:
                await self.client.write_gatt_char(NORDIC_UART_RX_UUID, self.start_command)
                print(f"已发送启动命令: {self.start_command}")
            
            return True
            
        except Exception as e:
            print(f"BLE 连接错误: {e}")
            self._connected = False
            return False
    
    async def disconnect(self):
        """断开 BLE 连接"""
        if self.client and self.client.is_connected:
            await self.client.stop_notify(NORDIC_UART_TX_UUID)
            await self.client.disconnect()
        self._connected = False
        print("BLE 连接已断开")
    
    def _on_data(self, sender, data: bytearray):
        """
        处理接收到的数据，尝试解析为四元数
        支持多种格式：JSON、二进制int16、二进制float32等
        """
        self._data_received = True
        self._last_update_time = asyncio.get_event_loop().time()
        
        # 尝试解析JSON格式
        try:
            text = data.decode('utf-8', errors='ignore').strip()
            if text and (text.startswith('{') or text.startswith('[')):
                json_data = json.loads(text)
                if isinstance(json_data, dict):
                    # 查找四元数字段
                    quat = self._extract_quat_from_dict(json_data)
                    if quat:
                        self._raw_quat = R.from_quat(quat)
                        return
        except:
            pass
        
        # 尝试解析二进制格式
        quat = self._parse_binary_quaternion(data)
        if quat:
            self._raw_quat = R.from_quat(quat)
    
    def _extract_quat_from_dict(self, data_dict):
        """从字典中提取四元数"""
        # 常见的四元数字段名
        quat_keys = ['quaternion', 'quat', 'q', 'orientation', 'rot', 'rotation']
        
        for key in quat_keys:
            if key in data_dict:
                val = data_dict[key]
                if isinstance(val, (list, tuple)) and len(val) >= 4:
                    try:
                        return [float(val[0]), float(val[1]), float(val[2]), float(val[3])]
                    except:
                        pass
        
        # 查找单独的字段 (qx, qy, qz, qw)
        if all(k in data_dict for k in ['qx', 'qy', 'qz', 'qw']):
            try:
                return [float(data_dict['qx']), float(data_dict['qy']), 
                       float(data_dict['qz']), float(data_dict['qw'])]
            except:
                pass
        
        # 查找单独的字段 (x, y, z, w)
        if all(k in data_dict for k in ['x', 'y', 'z', 'w']):
            try:
                return [float(data_dict['x']), float(data_dict['y']), 
                       float(data_dict['z']), float(data_dict['w'])]
            except:
                pass
        
        return None
    
    def _parse_binary_quaternion(self, data):
        """解析二进制格式的四元数"""
        # 格式1: 4个int16 (w, x, y, z) 小端序 - 8字节
        if len(data) >= 8:
            try:
                w, x, y, z = struct.unpack('<4h', data[:8])
                scale = 1.0 / 32768.0
                return [x * scale, y * scale, z * scale, w * scale]
            except:
                pass
        
        # 格式2: 4个int16 (x, y, z, w) 小端序 - 8字节
        if len(data) >= 8:
            try:
                x, y, z, w = struct.unpack('<4h', data[:8])
                scale = 1.0 / 32768.0
                return [x * scale, y * scale, z * scale, w * scale]
            except:
                pass
        
        # 格式3: 4个float32 (x, y, z, w) 小端序 - 16字节
        if len(data) >= 16:
            try:
                x, y, z, w = struct.unpack('<4f', data[:16])
                return [x, y, z, w]
            except:
                pass
        
        # 格式4: 4个float32 (w, x, y, z) 小端序 - 16字节
        if len(data) >= 16:
            try:
                w, x, y, z = struct.unpack('<4f', data[:16])
                return [x, y, z, w]
            except:
                pass
        
        return None
    
    def get_orientation(self) -> Optional[R]:
        """获取校准后的相对姿态"""
        if not self._connected or not self._data_received:
            return None
        return self._calib_inv * self._raw_quat
    
    def calibrate(self):
        """将当前姿态设为零位"""
        if not self._data_received:
            print("警告: 尚未收到数据，无法校准")
            return
        self._calib_inv = self._raw_quat.inv()
        print(">>> Nordic UART IMU 校准完成!")
    
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._connected and self.client is not None and self.client.is_connected
    
    async def send_command(self, command):
        """发送命令到设备"""
        if not self.client or not self.client.is_connected:
            print("设备未连接!")
            return False
        
        if isinstance(command, str):
            command = command.encode('utf-8')
        
        try:
            await self.client.write_gatt_char(NORDIC_UART_RX_UUID, command)
            return True
        except Exception as e:
            print(f"发送命令失败: {e}")
            return False


async def scan_ble_devices(timeout: float = 5.0) -> list:
    """
    扫描附近的 BLE 设备

    Returns:
        list: 设备列表 [(address, name), ...]
    """
    if not BLEAK_AVAILABLE:
        print("bleak 库未安装")
        return []

    print(f"正在扫描 BLE 设备 ({timeout}秒)...")
    devices = await BleakScanner.discover(timeout=timeout)

    result = []
    for d in devices:
        if d.name:
            result.append((d.address, d.name))
            print(f"  [{d.address}] {d.name}")

    return result


# 测试入口
if __name__ == "__main__":
    async def test():
        # 先扫描设备
        devices = await scan_ble_devices()

        if not devices:
            print("未发现任何 BLE 设备")
            return

        # 使用第一个设备测试
        address = devices[0][0]
        print(f"\n尝试连接: {address}")

        receiver = BLEIMUReceiver(address)
        connected = await receiver.connect()

        if connected:
            receiver.calibrate()
            for _ in range(100):
                euler = receiver.get_euler_degrees()
                if euler:
                    print(f"Roll: {euler[0]:.1f}, Pitch: {euler[1]:.1f}, Yaw: {euler[2]:.1f}")
                await asyncio.sleep(0.02)

            await receiver.disconnect()

    asyncio.run(test())
