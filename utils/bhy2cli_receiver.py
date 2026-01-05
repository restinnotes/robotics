"""
BHy2CLI IMU 接收器
==================
通过运行 bhy2cli.exe 子进程从 Bosch BHI360/BHI260 传感器接收四元数数据。

数据格式 (Game Rotation Vector, Sensor ID 37):
    [D]SID: 37; T: 886.524359375; x: 0.623108, y: 0.249878, z: 0.002319, w: 0.741089; acc: 0.000000

使用方法:
    receiver = BHy2CLIReceiver(sensor_id=37, sample_rate=50)
    receiver.connect()
    receiver.calibrate()
    while True:
        orientation = receiver.get_orientation()
"""

import subprocess
import threading
import re
import time
import os
import platform
from typing import Optional
import numpy as np
from scipy.spatial.transform import Rotation as R

from utils.imu_data_source import IMUDataSource


# 默认配置
DEFAULT_SENSOR_ID = 37  # Game Rotation Vector (四元数, 仅 IMU)
DEFAULT_SAMPLE_RATE = 50  # Hz


class BHy2CLIReceiver(IMUDataSource):
    """
    BHy2CLI 接收器，通过子进程运行 bhy2cli 可执行文件获取传感器数据
    支持 Windows 和 Linux 平台

    支持的 Sensor ID:
        - 34: Rotation Vector (含磁力计校正)
        - 37: Game Rotation Vector (仅 IMU, 推荐)
        - 40: GeoMag Rotation Vector
        - 121: Head Orientation Quaternion
    """

    def __init__(
        self,
        exe_path: str = None,
        sensor_id: int = DEFAULT_SENSOR_ID,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
        firmware_path: str = "release/BHI3-firmwares/BHI360/Bosch_Shuttle3_BHI360.fw",
        enable_drift_compensation: bool = True  # 默认启用，与 Windows 版本保持一致
    ):
        """
        Args:
            exe_path: bhy2cli 可执行文件的路径 (默认自动查找，支持 Windows 和 Linux)
            sensor_id: 传感器 ID (默认 37 = Game Rotation Vector)
            sample_rate: 采样率 Hz (默认 50)
            firmware_path: 固件文件路径 (用于自动修复固件丢失)
            enable_drift_compensation: 是否启用自适应漂移补偿
        """
        self.sensor_id = sensor_id
        self.sample_rate = sample_rate
        self.firmware_path = firmware_path
        self.enable_drift_compensation = enable_drift_compensation

        # 自动查找 exe 路径
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

        # 四元数数据
        self._raw_quat = R.identity()  # 原始四元数
        self._calib_inv = R.identity()  # 校准逆
        self._last_update_time = 0
        self._data_count = 0

        # 正则表达式解析
        # 格式: x: 0.623108, y: 0.249878, z: 0.002319, w: 0.741089
        self._quat_pattern = re.compile(
            r'x:\s*([-\d.]+),\s*y:\s*([-\d.]+),\s*z:\s*([-\d.]+),\s*w:\s*([-\d.]+)'
        )

    def _find_exe(self) -> str:
        """查找 bhy2cli 可执行文件路径（支持 Windows 和 Linux）"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(current_dir)
        bhy_root = os.path.join(project_root, "BHy2CLI")

        # 根据操作系统确定文件扩展名
        is_windows = platform.system() == "Windows"
        exe_ext = ".exe" if is_windows else ""

        # 候选文件名 (按优先级)
        base_names = ["spi_bhy2cli", "bhy2cli", "i2c_bhy2cli"]
        exe_names = [name + exe_ext for name in base_names]

        for name in exe_names:
            path = os.path.join(bhy_root, name)
            if os.path.isfile(path) and os.access(path, os.X_OK):
                return os.path.abspath(path)

            # 同时也检查子目录
            for root, dirs, files in os.walk(bhy_root):
                if name in files:
                    full_path = os.path.abspath(os.path.join(root, name))
                    if os.access(full_path, os.X_OK):
                        return full_path

        raise FileNotFoundError(
            f"找不到 bhy2cli 可执行文件。请确保 BHy2CLI 目录下有以下文件之一: {', '.join(exe_names)}"
        )

    def _boot_firmware(self):
        """尝试上传并启动固件"""
        if not self.firmware_path:
            return False

        # 确定固件绝对路径
        bhy_root = os.path.dirname(self.exe_path)
        fw_abs_path = os.path.join(bhy_root, self.firmware_path)

        if not os.path.isfile(fw_abs_path):
            # 尝试相对于根目录
            current_dir = os.path.dirname(os.path.abspath(__file__))
            project_root = os.path.dirname(current_dir)
            fw_abs_path = os.path.join(project_root, "BHy2CLI", self.firmware_path)

            if not os.path.isfile(fw_abs_path):
                print(f"找不到固件文件: {self.firmware_path}")
                return False

        print(f"检测到固件丢失，正在自动尝试重载: {os.path.basename(fw_abs_path)} ...")
        cmd = [self.exe_path, "-b", fw_abs_path]

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, cwd=bhy_root, timeout=15)
            if "Booting from RAM successful" in result.stdout:
                print("固件加载成功!")
                return True
            else:
                print(f"固件加载失败: {result.stdout}")
                return False
        except Exception as e:
            print(f"固件加载出错: {e}")
            return False

    def connect(self, perform_gyro_foc: bool = False) -> bool:
        """
        启动子进程并开始接收数据
        Args:
            perform_gyro_foc: 是否在启动时执行陀螺仪校准 (foc 3)
        """
        if self._running:
            return True

        # 尝试连接
        success = self._start_process(perform_gyro_foc=perform_gyro_foc)

        # 如果是因为传感器没固件失败，尝试加载固件后再试一次
        if not success:
            print("连接初次尝试失败，尝试修复固件...")
            if self._boot_firmware():
                success = self._start_process(perform_gyro_foc=perform_gyro_foc)

        return success

    def _start_process(self, perform_gyro_foc: bool = False) -> bool:
        """启动 bhy2cli 进程"""
        try:
            cmd = [self.exe_path]

            # 如果需要 Gyro FOC (且 bhy2cli 支持链式命令)，则插在前面
            if perform_gyro_foc:
                print("启用启动时陀螺仪校准 (foc 3)...")
                cmd.extend(["foc", "3"])

            cmd.extend(["-c", f"{self.sensor_id}:{self.sample_rate}"])
            print(f"启动 BHy2CLI: {' '.join(cmd)}")

            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                cwd=os.path.dirname(self.exe_path)
            )

            # 监听数据
            self._running = True
            self._data_count = 0
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()

            # 等待确认是否有有效数据输出
            start_wait = time.time()
            while time.time() - start_wait < 3.0:
                if "not present in the loaded firmware" in self._last_line:
                    print("错误: 当前固件不支持此传感器。")
                    self.disconnect()
                    return False
                if self._data_count > 5:
                    self._connected = True
                    print(f"数据接收正常 (已记录 {self._data_count} 帧)")
                    return True
                time.sleep(0.1)

            self.disconnect()
            return False

        except Exception as e:
            print(f"进程启动失败: {e}")
            self._running = False
            return False

    def disconnect(self):
        """停止子进程"""
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
        """读取子进程 stdout 的线程"""
        while self._running and self.process:
            try:
                line = self.process.stdout.readline()
                if not line:
                    if self.process.poll() is not None:
                        break
                    continue

                self._last_line = line.strip()
                # 解析数据
                self._parse_line(self._last_line)

            except Exception as e:
                if self._running:
                    print(f"读取错误: {e}")
                break

        self._connected = False

    def _parse_line(self, line: str):
        """解析一行输出"""
        # 跳过非数据行
        if not line.startswith("[D]SID:"):
            # 打印信息行 (用于调试)
            if line and not line.startswith("[D][META"):
                print(f"[BHy2CLI] {line}")
            return

        # 检查是否是我们请求的 Sensor ID
        if f"SID: {self.sensor_id}" not in line:
            return

        # 解析四元数
        match = self._quat_pattern.search(line)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            z = float(match.group(3))
            w = float(match.group(4))

            # 转换为 Scipy Rotation (注意: from_quat 顺序是 [x, y, z, w])
            try:
                self._raw_quat = R.from_quat([x, y, z, w])
                self._last_update_time = time.time()
                self._data_count += 1

                # 运行自适应漂移补偿
                if self.enable_drift_compensation:
                    self._detect_stillness_and_compensate()
            except Exception as e:
                print(f"四元数解析错误: {e}")

    def get_orientation(self) -> Optional[R]:
        """获取校准后的相对姿态 (带自适应漂移补偿)"""
        if not self._connected:
            return None

        # 基础校准
        calibrated = self._calib_inv * self._raw_quat

        # 应用自适应零点偏移（由静止检测持续更新）
        if hasattr(self, '_zero_offset'):
            calibrated = self._zero_offset.inv() * calibrated

        return calibrated

    def _detect_stillness_and_compensate(self):
        """
        检测静止状态并自适应补偿漂移
        在 _parse_line 中调用
        """
        if not hasattr(self, '_prev_raw_quat'):
            self._prev_raw_quat = self._raw_quat
            self._prev_quat_time = time.time()
            self._zero_offset = R.identity()
            self._stillness_counter = 0
            return

        # 计算两帧之间的角度变化
        delta_rot = self._prev_raw_quat.inv() * self._raw_quat
        delta_angle = np.linalg.norm(delta_rot.as_rotvec()) * 180 / np.pi  # 度
        delta_time = time.time() - self._prev_quat_time

        if delta_time > 0:
            angular_rate = delta_angle / delta_time  # 度/秒
        else:
            angular_rate = 0

        # 判断是否静止 (角速度 < 阈值)
        STILLNESS_THRESHOLD = 5.0  # 度/秒

        if angular_rate < STILLNESS_THRESHOLD:
            self._stillness_counter += 1

            # 连续静止超过一定帧数后，开始校正漂移
            if self._stillness_counter > 10:  # 大约 0.2 秒静止
                # 获取当前校准后的姿态（不含 zero_offset）
                calibrated_no_offset = self._calib_inv * self._raw_quat

                # 计算需要补偿的偏移量（让它回到零）
                # 使用指数平滑逐渐调整
                alpha = 0.02  # 调整速度，越小越平滑

                if hasattr(self, '_zero_offset'):
                    # 逐渐将当前姿态拉回零点
                    current_offset = calibrated_no_offset.as_rotvec()
                    new_offset_rotvec = self._zero_offset.as_rotvec() + alpha * current_offset
                    self._zero_offset = R.from_rotvec(new_offset_rotvec)
        else:
            self._stillness_counter = 0

        self._prev_raw_quat = self._raw_quat
        self._prev_quat_time = time.time()

    def calibrate(self, measure_drift_seconds: float = 2.0):
        """
        将当前姿态设为零位，并初始化自适应漂移补偿

        Args:
            measure_drift_seconds: 等待时间 (秒)，让用户准备好
        """
        print(f">>> 正在初始化 (请保持静止 {measure_drift_seconds} 秒)...")
        time.sleep(measure_drift_seconds)

        # 设置校准零点
        self._calib_inv = self._raw_quat.inv()

        # 初始化自适应补偿变量
        self._zero_offset = R.identity()
        self._prev_raw_quat = self._raw_quat
        self._prev_quat_time = time.time()
        self._stillness_counter = 0

        print(">>> BHy2CLI 校准完成 (已启用自适应漂移补偿)!")
        print(">>> 提示: 保持静止时系统会自动消除漂移")



    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._connected and self._running

    def get_data_rate(self) -> float:
        """
        获取数据更新延迟 (用于调试)

        Returns:
            float: 距离上次数据更新的时间 (秒)
        """
        if self._last_update_time == 0:
            return -1
        return time.time() - self._last_update_time

    def get_raw_quaternion(self) -> tuple:
        """
        获取原始四元数 (未校准)

        Returns:
            tuple: (x, y, z, w)
        """
        quat = self._raw_quat.as_quat()  # [x, y, z, w]
        return tuple(quat)


# 测试入口
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="BHy2CLI IMU 接收器测试")
    parser.add_argument("--sensor-id", "-s", type=int, default=37,
                        help="Sensor ID (默认: 37 = Game Rotation Vector)")
    parser.add_argument("--rate", "-r", type=int, default=50,
                        help="采样率 Hz (默认: 50)")
    args = parser.parse_args()

    print(f"测试 BHy2CLI 接收器 (Sensor ID: {args.sensor_id}, Rate: {args.rate} Hz)")

    receiver = BHy2CLIReceiver(sensor_id=args.sensor_id, sample_rate=args.rate)

    if receiver.connect():
        print("\n等待数据稳定后按 Enter 校准...")
        input()
        receiver.calibrate()

        print("\n开始输出欧拉角 (Ctrl+C 退出):")
        try:
            while True:
                euler = receiver.get_euler_degrees()
                if euler:
                    print(f"Roll: {euler[0]:7.2f}, Pitch: {euler[1]:7.2f}, Yaw: {euler[2]:7.2f}")
                time.sleep(0.02)
        except KeyboardInterrupt:
            print("\n")

        receiver.disconnect()
    else:
        print("连接失败!")
