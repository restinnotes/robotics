"""
WiFi IMU 接收器 (phyphox)
=========================
通过 HTTP API 从 phyphox App 接收姿态数据。

使用方法:
    receiver = PhyphoxIMUReceiver("http://192.168.1.31:8080")
    receiver.calibrate()
    while True:
        orientation = receiver.get_orientation()
"""

import requests
from typing import Optional
from scipy.spatial.transform import Rotation as R
import numpy as np

from utils.imu_data_source import IMUDataSource


class PhyphoxIMUReceiver(IMUDataSource):
    """
    WiFi IMU 接收器，通过 phyphox App 的 HTTP API 获取姿态数据
    支持 "斜面 (Inclination)" 实验
    """

    def __init__(self, base_url: str, timeout: float = 0.3):
        """
        Args:
            base_url: phyphox 显示的网址 (如 "http://192.168.1.31:8080")
            timeout: HTTP 请求超时时间 (秒)
        """
        self.base_url = base_url.rstrip('/')
        self.api_url = f"{self.base_url}/get?"
        self.timeout = timeout

        # 状态
        self._raw_orientation = R.identity()
        self._calib_inv = R.identity()
        self._connected = False

    def _fetch_data(self) -> Optional[R]:
        """从 phyphox API 获取原始姿态数据"""
        try:
            # 使用 Inclination 实验的 buffer 名
            response = requests.get(
                f"{self.api_url}angle&anglePlane",
                timeout=self.timeout,
                proxies={'http': None, 'https': None}  # 绕过 VPN
            )

            if response.status_code != 200:
                return None

            data = response.json()['buffer']

            # 解析 angle (前后倾斜) 和 anglePlane (左右倾斜)
            if 'angle' in data and len(data['angle']['buffer']) > 0:
                angle = data['angle']['buffer'][-1]
                anglePlane = 0
                if 'anglePlane' in data and len(data['anglePlane']['buffer']) > 0:
                    anglePlane = data['anglePlane']['buffer'][-1]

                self._connected = True
                return R.from_euler('xyz', [np.radians(angle), 0, np.radians(-anglePlane)], degrees=False)

            return None

        except requests.exceptions.RequestException:
            self._connected = False
            return None
        except Exception:
            return None

    def get_orientation(self) -> Optional[R]:
        """获取校准后的相对姿态"""
        raw = self._fetch_data()
        if raw is None:
            return None
        self._raw_orientation = raw
        return self._calib_inv * self._raw_orientation

    def calibrate(self):
        """将当前姿态设为零位"""
        raw = self._fetch_data()
        if raw is not None:
            self._calib_inv = raw.inv()
            print(">>> Phyphox IMU 校准完成!")
        else:
            print("警告: 无法获取数据进行校准，请检查连接")

    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._connected

    def wait_for_connection(self, max_attempts: int = 30) -> bool:
        """
        等待连接建立

        Args:
            max_attempts: 最大尝试次数

        Returns:
            bool: 是否成功连接
        """
        import time
        print(f"正在连接 {self.base_url}...")

        for i in range(max_attempts):
            if self._fetch_data() is not None:
                print("连接成功!")
                return True
            print(f"等待连接... ({i+1}/{max_attempts})")
            time.sleep(1.0)

        print("连接超时")
        return False


# 测试入口
if __name__ == "__main__":
    import time

    receiver = PhyphoxIMUReceiver("http://192.168.1.31:8080")

    if receiver.wait_for_connection():
        print("\n请将手机保持静止，按 Enter 校准...")
        input()
        receiver.calibrate()

        print("\n开始读取数据 (Ctrl+C 退出):")
        try:
            while True:
                euler = receiver.get_euler_degrees()
                if euler:
                    print(f"Roll: {euler[0]:6.1f}°  Pitch: {euler[1]:6.1f}°  Yaw: {euler[2]:6.1f}°", end='\r')
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n退出")
