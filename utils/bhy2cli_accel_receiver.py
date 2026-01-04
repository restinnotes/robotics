"""
BHy2CLI 加速度计倾斜接收器
=========================
通过加速度计直接计算 Roll 和 Pitch，完全避免陀螺仪漂移问题。

原理：
    加速度计测量重力方向（当静止或匀速运动时）。
    从重力向量可以直接计算 Roll 和 Pitch，但无法获取 Yaw。

数据格式 (Accelerometer, Sensor ID 1):
    [D]SID: 1; T: 106.136375000; x: -0.513672, y: 0.746338, z: -0.972168; acc: 0

使用方法:
    receiver = BHy2CLIAccelReceiver()
    receiver.connect()
    receiver.calibrate()
    while True:
        roll, pitch = receiver.get_tilt_degrees()
"""

import subprocess
import threading
import re
import time
import os
import numpy as np
from typing import Optional, Tuple

from utils.imu_data_source import IMUDataSource


class BHy2CLIAccelReceiver(IMUDataSource):
    """
    BHy2CLI 加速度计接收器

    使用 Sensor ID 1 (Accelerometer) 计算倾斜角 (Roll/Pitch)。
    优点: 永远不会漂移
    缺点: 无法获取 Yaw 角度
    """

    SENSOR_ID = 1  # Accelerometer
    DEFAULT_SAMPLE_RATE = 50

    def __init__(
        self,
        exe_path: str = None,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        firmware_path: str = "release/BHI3-firmwares/BHI360/Bosch_Shuttle3_BHI360.fw"
    ):
        self.sensor_id = self.SENSOR_ID
        self.sample_rate = sample_rate
        self.firmware_path = firmware_path

        if exe_path is None:
            self.exe_path = self._find_exe()
        else:
            self.exe_path = exe_path

        # 状态
        self.process: Optional[subprocess.Popen] = None
        self._running = False
        self._connected = False
        self._thread: Optional[threading.Thread] = None
        self._last_line = ""

        # 加速度数据 (单位: g)
        self._accel = np.array([0.0, 0.0, -1.0])  # 默认朝下
        self._calib_roll = 0.0
        self._calib_pitch = 0.0
        self._last_update_time = 0
        self._data_count = 0

        # 正则表达式 (3轴格式)
        self._accel_pattern = re.compile(
            r'x:\s*([-\d.]+),\s*y:\s*([-\d.]+),\s*z:\s*([-\d.]+);'
        )

    def _find_exe(self) -> str:
        """查找 bhy2cli.exe 或 spi_bhy2cli.exe"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(current_dir)
        bhy_root = os.path.join(project_root, "BHy2CLI")

        exe_names = ["spi_bhy2cli.exe", "bhy2cli.exe", "i2c_bhy2cli.exe"]
        for name in exe_names:
            path = os.path.join(bhy_root, name)
            if os.path.isfile(path):
                return os.path.abspath(path)
        raise FileNotFoundError("找不到 bhy2cli 可执行文件")

    def _boot_firmware(self):
        """尝试上传并启动固件"""
        bhy_root = os.path.dirname(self.exe_path)
        fw_abs_path = os.path.join(bhy_root, self.firmware_path)

        if not os.path.isfile(fw_abs_path):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(current_dir)
            fw_abs_path = os.path.join(project_root, "BHy2CLI", self.firmware_path)

        if not os.path.isfile(fw_abs_path):
            return False

        print(f"正在加载固件: {os.path.basename(fw_abs_path)} ...")
        cmd = [self.exe_path, "-b", fw_abs_path]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, cwd=bhy_root, timeout=15)
            return "Booting from RAM successful" in result.stdout
        except:
            return False

    def connect(self) -> bool:
        if self._running:
            return True

        success = self._start_process()
        if not success:
            print("尝试重新加载固件...")
            if self._boot_firmware():
                success = self._start_process()
        return success

    def _start_process(self) -> bool:
        try:
            cmd = [self.exe_path, "-c", f"{self.sensor_id}:{self.sample_rate}"]
            print(f"启动 BHy2CLI: {' '.join(cmd)}")

            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                cwd=os.path.dirname(self.exe_path)
            )

            self._running = True
            self._data_count = 0
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()

            start_wait = time.time()
            while time.time() - start_wait < 3.0:
                if self._data_count > 5:
                    self._connected = True
                    print(f"加速度计连接成功 (已记录 {self._data_count} 帧)")
                    return True
                time.sleep(0.1)

            self.disconnect()
            return False

        except Exception as e:
            print(f"进程启动失败: {e}")
            return False

    def disconnect(self):
        self._running = False
        if self.process:
            try:
                self.process.terminate()
                self.process.wait(timeout=2)
            except:
                self.process.kill()
            self.process = None
        self._connected = False
        print("BHy2CLI 已断开")

    def _read_loop(self):
        while self._running and self.process:
            try:
                line = self.process.stdout.readline()
                if not line:
                    if self.process.poll() is not None:
                        break
                    continue

                self._last_line = line.strip()
                self._parse_line(self._last_line)
            except:
                break
        self._connected = False

    def _parse_line(self, line: str):
        if not line.startswith("[D]SID:"):
            if line and not line.startswith("[D][META"):
                print(f"[BHy2CLI] {line}")
            return

        if f"SID: {self.sensor_id}" not in line:
            return

        match = self._accel_pattern.search(line)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            z = float(match.group(3))
            self._accel = np.array([x, y, z])
            self._last_update_time = time.time()
            self._data_count += 1

    def get_tilt_degrees(self) -> Optional[Tuple[float, float]]:
        """
        从加速度计计算倾斜角 (Roll, Pitch)

        Returns:
            (roll, pitch) 度数, 或 None 如果未连接
        """
        if not self._connected:
            return None

        ax, ay, az = self._accel

        # 从重力计算倾斜
        # Roll: 绕 X 轴旋转 (左右倾斜)
        # Pitch: 绕 Y 轴旋转 (前后倾斜)
        roll = np.arctan2(ay, np.sqrt(ax**2 + az**2))
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        roll_deg = np.degrees(roll) - self._calib_roll
        pitch_deg = np.degrees(pitch) - self._calib_pitch

        return (roll_deg, pitch_deg)

    def calibrate(self):
        """将当前倾斜设为零位"""
        if not self._connected:
            return

        print(">>> 正在校准 (请保持静止)...")
        time.sleep(1.0)

        ax, ay, az = self._accel
        roll = np.arctan2(ay, np.sqrt(ax**2 + az**2))
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        self._calib_roll = np.degrees(roll)
        self._calib_pitch = np.degrees(pitch)

        print(f">>> 校准完成! 零点: Roll={self._calib_roll:.1f}°, Pitch={self._calib_pitch:.1f}°")

    def is_connected(self) -> bool:
        return self._connected and self._running

    # IMUDataSource 接口兼容
    def get_orientation(self):
        """兼容接口，返回一个模拟的 Rotation 对象"""
        tilt = self.get_tilt_degrees()
        if tilt is None:
            return None
        roll, pitch = tilt
        # 创建一个只有 Roll/Pitch 的 Rotation (Yaw=0)
        from scipy.spatial.transform import Rotation as R
        return R.from_euler('xyz', [np.radians(roll), np.radians(pitch), 0])
