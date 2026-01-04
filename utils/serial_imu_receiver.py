"""
串口IMU接收器
=============
通过串口（/dev/ttyACM0）读取IMU四元数数据
"""

import serial
import serial.tools.list_ports
import time
import struct
import json
import threading
from typing import Optional
from scipy.spatial.transform import Rotation as R

from utils.imu_data_source import IMUDataSource


class SerialIMUReceiver(IMUDataSource):
    """
    串口IMU接收器，用于从串口读取四元数数据
    设备在Linux上通常显示为 /dev/ttyACM0
    """
    
    def __init__(
        self,
        port: Optional[str] = None,
        baud_rate: int = 115200,
        timeout: float = 1.0
    ):
        """
        Args:
            port: 串口路径，如果为None则自动查找
            baud_rate: 波特率
            timeout: 超时时间（秒）
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        
        # 状态
        self.ser: Optional[serial.Serial] = None
        self._connected = False
        self._raw_quat = R.identity()
        self._calib_inv = R.identity()
        self._last_update_time = 0
        self._data_received = False
        
        # 线程
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._buffer = b""
    
    def _find_port(self) -> Optional[str]:
        """自动查找IMU设备串口"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "APP" in port.description or "Board" in port.description or "108c:ab3c" in port.hwid:
                return port.device
        # 默认尝试
        return "/dev/ttyACM0"
    
    def connect(self) -> bool:
        """
        连接串口设备
        
        Returns:
            bool: 连接是否成功
        """
        try:
            if self.port is None:
                self.port = self._find_port()
            
            print(f"正在打开串口: {self.port} (波特率: {self.baud_rate})...")
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            
            # 清空缓冲区
            self.ser.reset_input_buffer()
            time.sleep(0.1)
            
            self._connected = True
            self._running = True
            
            # 启动读取线程
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            
            print(f"✓ 串口已打开: {self.port}")
            return True
            
        except serial.SerialException as e:
            print(f"❌ 串口错误: {e}")
            if "Permission denied" in str(e):
                print("\n权限问题解决方案:")
                print("  方法1: sudo chmod 666 /dev/ttyACM0")
                print("  方法2: sudo usermod -a -G dialout $USER")
                print("         (然后重新登录)")
            self._connected = False
            return False
        except Exception as e:
            print(f"连接错误: {e}")
            self._connected = False
            return False
    
    def disconnect(self):
        """断开串口连接"""
        self._running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self._connected = False
        print("串口已关闭")
    
    def _parse_quaternion(self, data: bytes) -> Optional[list]:
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
        
        # 格式2: 二进制 int16 (8字节) - w, x, y, z
        if len(data) >= 8:
            try:
                w, x, y, z = struct.unpack('<4h', data[:8])
                scale = 1.0 / 32768.0
                return [x * scale, y * scale, z * scale, w * scale]
            except:
                pass
        
        # 格式3: 二进制 float32 (16字节) - x, y, z, w
        if len(data) >= 16:
            try:
                x, y, z, w = struct.unpack('<4f', data[:16])
                return [x, y, z, w]
            except:
                pass
        
        return None
    
    def _read_loop(self):
        """后台读取循环"""
        while self._running and self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    self._buffer += data
                    
                    # 尝试按行解析（文本格式）
                    while b'\n' in self._buffer:
                        line, self._buffer = self._buffer.split(b'\n', 1)
                        if len(line) > 0:
                            quat = self._parse_quaternion(line)
                            if quat:
                                try:
                                    self._raw_quat = R.from_quat(quat)
                                    self._last_update_time = time.time()
                                    self._data_received = True
                                except:
                                    pass
                    
                    # 如果没有换行符，尝试按固定长度解析（二进制格式）
                    if len(self._buffer) >= 16:
                        quat = self._parse_quaternion(self._buffer[:16])
                        if quat:
                            try:
                                self._raw_quat = R.from_quat(quat)
                                self._last_update_time = time.time()
                                self._data_received = True
                                self._buffer = self._buffer[16:]
                            except:
                                # 如果解析失败，移除一个字节继续尝试
                                self._buffer = self._buffer[1:]
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self._running:
                    print(f"读取错误: {e}")
                break
    
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
        print(">>> 串口IMU校准完成!")
    
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._connected and self.ser is not None and self.ser.is_open


# 测试入口
if __name__ == "__main__":
    receiver = SerialIMUReceiver()
    
    if receiver.connect():
        print("\n等待数据... (5秒)")
        time.sleep(5)
        
        if receiver._data_received:
            print("✓ 已收到数据!")
            receiver.calibrate()
            
            print("\n显示实时数据 (10秒)...")
            for _ in range(100):
                euler = receiver.get_euler_degrees()
                if euler:
                    print(f"Roll: {euler[0]:6.1f}°, Pitch: {euler[1]:6.1f}°, Yaw: {euler[2]:6.1f}°", end='\r')
                time.sleep(0.1)
            print()
        else:
            print("⚠ 未收到数据")
        
        receiver.disconnect()
    else:
        print("连接失败")
