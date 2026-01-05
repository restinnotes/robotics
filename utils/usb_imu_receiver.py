"""
USB IMU 接收器 (BHI3xx)
=========================
通过 USB 串口从 Bosch BHI360/BHI260 开发板接收四元数数据。
适用于 Linux Ubuntu 系统。

硬件连接:
- 使用 USB 线直接连接 BHI3xx 开发板到电脑
- 设备通常出现在 /dev/ttyUSB0 或 /dev/ttyACM0

数据格式 (Bosch 标准):
- 8 字节: 4 个 int16 (w, x, y, z)
- 归一化: 除以 32768 得到 [-1, 1] 范围

使用方法:
    receiver = USBIMUReceiver("/dev/ttyUSB0")
    receiver.connect()
    receiver.calibrate()
    while True:
        orientation = receiver.get_orientation()
"""

import struct
import threading
import time
from typing import Optional
from scipy.spatial.transform import Rotation as R

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("警告: pyserial 库未安装，USB 功能不可用。请运行: pip install pyserial")

from utils.imu_data_source import IMUDataSource


# 默认串口配置
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 1.0

# 数据帧格式常量
FRAME_HEADER = 0xAA  # 帧头标识 (根据实际固件调整)
QUAT_FRAME_SIZE = 8  # 4 个 int16 = 8 字节


class USBIMUReceiver(IMUDataSource):
    """
    USB 串口 IMU 接收器，用于接收 BHI3xx 传感器的四元数数据

    支持两种数据格式:
    1. 原始二进制格式: 8 字节 (4 x int16)
    2. 带帧头的格式: [0xAA][8字节数据] (根据固件调整)
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = DEFAULT_BAUDRATE,
        data_format: str = "raw",  # "raw" 或 "framed"
        timeout: float = DEFAULT_TIMEOUT
    ):
        """
        Args:
            port: 串口设备路径 (如 "/dev/ttyUSB0", "/dev/ttyACM0")
            baudrate: 波特率 (默认 115200)
            data_format: 数据格式 - "raw" (纯数据) 或 "framed" (带帧头)
            timeout: 读取超时时间
        """
        if not SERIAL_AVAILABLE:
            raise RuntimeError("pyserial 库未安装，请运行: pip install pyserial")

        self.port = port
        self.baudrate = baudrate
        self.data_format = data_format
        self.timeout = timeout

        # 状态
        self.serial: Optional[serial.Serial] = None
        self._connected = False
        self._raw_quat = R.identity()  # 原始四元数
        self._calib_inv = R.identity()  # 校准逆
        self._last_update_time = 0.0

        # 接收线程
        self._read_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()

    def connect(self) -> bool:
        """
        连接到 USB 串口设备并启动数据接收线程

        Returns:
            bool: 连接是否成功
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )

            if not self.serial.is_open:
                print(f"无法打开串口: {self.port}")
                return False

            print(f"已连接到 USB 串口: {self.port} @ {self.baudrate} bps")

            # 清空输入缓冲区
            self.serial.reset_input_buffer()

            # 启动接收线程
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()

            self._connected = True
            print("USB IMU 接收线程已启动")
            return True

        except serial.SerialException as e:
            print(f"USB 串口错误: {e}")
            print("提示: 确保你有访问串口的权限，可能需要运行:")
            print(f"  sudo chmod 666 {self.port}")
            print("  或将用户添加到 dialout 组: sudo usermod -a -G dialout $USER")
            self._connected = False
            return False

    def disconnect(self):
        """断开 USB 连接"""
        self._running = False

        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)

        if self.serial and self.serial.is_open:
            self.serial.close()

        self._connected = False
        print("USB 连接已断开")

    def _read_loop(self):
        """
        数据接收线程的主循环
        持续从串口读取数据并解析四元数
        """
        buffer = bytearray()

        while self._running and self.serial and self.serial.is_open:
            try:
                # 读取可用数据
                available = self.serial.in_waiting
                if available > 0:
                    data = self.serial.read(available)
                    buffer.extend(data)

                    # 处理缓冲区中的数据
                    self._process_buffer(buffer)
                else:
                    time.sleep(0.001)  # 短暂休眠避免 CPU 占用过高

            except serial.SerialException as e:
                print(f"串口读取错误: {e}")
                break

    def _process_buffer(self, buffer: bytearray):
        """
        处理接收缓冲区中的数据
        支持原始格式和带帧头格式
        """
        if self.data_format == "raw":
            # 原始格式: 直接读取 8 字节
            while len(buffer) >= QUAT_FRAME_SIZE:
                data = bytes(buffer[:QUAT_FRAME_SIZE])
                del buffer[:QUAT_FRAME_SIZE]
                self._parse_quaternion(data)

        elif self.data_format == "framed":
            # 带帧头格式: 寻找帧头后读取数据
            while len(buffer) >= QUAT_FRAME_SIZE + 1:
                # 寻找帧头
                try:
                    idx = buffer.index(FRAME_HEADER)
                    if idx > 0:
                        # 丢弃帧头之前的数据
                        del buffer[:idx]
                except ValueError:
                    # 没有找到帧头，清空缓冲区
                    buffer.clear()
                    break

                # 检查是否有完整帧
                if len(buffer) >= QUAT_FRAME_SIZE + 1:
                    # 跳过帧头，读取数据
                    data = bytes(buffer[1:QUAT_FRAME_SIZE + 1])
                    del buffer[:QUAT_FRAME_SIZE + 1]
                    self._parse_quaternion(data)
                else:
                    break

    def _parse_quaternion(self, data: bytes):
        """
        解析四元数数据

        Bosch 格式: 4 个 int16 (w, x, y, z), 小端序
        归一化: 除以 32768
        """
        if len(data) < 8:
            return

        # 解析 4 个 int16
        w, x, y, z = struct.unpack('<4h', data[:8])
        scale = 1.0 / 32768.0

        # 转换为 Scipy Rotation (注意: from_quat 顺序是 [x, y, z, w])
        with self._lock:
            self._raw_quat = R.from_quat([x * scale, y * scale, z * scale, w * scale])
            self._last_update_time = time.time()

    def get_orientation(self) -> Optional[R]:
        """获取校准后的相对姿态"""
        if not self._connected:
            return None
        with self._lock:
            return self._calib_inv * self._raw_quat

    def calibrate(self):
        """将当前姿态设为零位"""
        with self._lock:
            self._calib_inv = self._raw_quat.inv()
        print(">>> USB IMU 校准完成!")

    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._connected and self.serial is not None and self.serial.is_open

    def get_data_rate(self) -> float:
        """
        获取数据更新率 (用于调试)

        Returns:
            float: 距离上次数据更新的时间 (秒)
        """
        if self._last_update_time == 0:
            return -1.0
        return time.time() - self._last_update_time


def list_serial_ports() -> list:
    """
    列出所有可用的串口设备

    Returns:
        list: 串口设备列表 [(port, description), ...]
    """
    if not SERIAL_AVAILABLE:
        print("pyserial 库未安装")
        return []

    print("正在扫描串口设备...")
    ports = serial.tools.list_ports.comports()

    result = []
    for port in ports:
        result.append((port.device, port.description))
        print(f"  [{port.device}] {port.description}")

    if not result:
        print("  未发现任何串口设备")
        print("  提示: 确保 USB 线已连接，并尝试:")
        print("    ls /dev/ttyUSB* /dev/ttyACM*")

    return result


# 测试入口
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="USB IMU 接收器测试")
    parser.add_argument("--port", "-p", default="/dev/ttyUSB0", help="串口设备路径")
    parser.add_argument("--baudrate", "-b", type=int, default=115200, help="波特率")
    parser.add_argument("--format", "-f", choices=["raw", "framed"], default="raw", help="数据格式")
    parser.add_argument("--scan", "-s", action="store_true", help="扫描可用串口")
    args = parser.parse_args()

    if args.scan:
        list_serial_ports()
    else:
        print(f"\n尝试连接: {args.port}")

        receiver = USBIMUReceiver(
            port=args.port,
            baudrate=args.baudrate,
            data_format=args.format
        )

        if receiver.connect():
            print("\n按 Ctrl+C 退出\n")
            time.sleep(0.5)  # 等待首批数据
            receiver.calibrate()

            try:
                while True:
                    euler = receiver.get_euler_degrees()
                    if euler:
                        print(f"\rRoll: {euler[0]:7.2f}°, Pitch: {euler[1]:7.2f}°, Yaw: {euler[2]:7.2f}°  ", end="")
                    time.sleep(0.02)
            except KeyboardInterrupt:
                print("\n")

            receiver.disconnect()
        else:
            print("连接失败!")
