"""
BHy2CLI IMU 接收器
==================
通过运行 bhy2cli 可执行文件（i2c_bhy2cli 或 spi_bhy2cli）子进程从 Bosch BHI360/BHI260 传感器接收四元数数据。
支持 Windows (.exe) 和 Linux 平台。

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
        firmware_path: str = "release/BHI3-firmwares/BHI360/Bosch_Shuttle3_BHI360.fw"
    ):
        """
        Args:
            exe_path: bhy2cli 可执行文件的路径 (默认自动查找，支持 Windows 和 Linux)
            sensor_id: 传感器 ID (默认 37 = Game Rotation Vector)
            sample_rate: 采样率 Hz (默认 50)
            firmware_path: 固件文件路径 (用于自动修复固件丢失)
        """
        self.sensor_id = sensor_id
        self.sample_rate = sample_rate
        self.firmware_path = firmware_path

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
        
        # 注意：重力校准已通过 FOC (foc 3) 在传感器层面完成
        # FOC 校准数据直接写入传感器内存，不需要文件操作

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

    def connect(self, perform_gyro_foc: bool = False, load_bsx_calibration: bool = True) -> bool:
        """
        启动子进程并开始接收数据
        Args:
            perform_gyro_foc: 是否在启动时执行陀螺仪校准 (foc 3)
            load_bsx_calibration: 是否自动加载 BSX 校准数据（从 bin 文件，默认 True）
        """
        if self._running:
            return True

        # 在启动数据流之前，先加载 BSX 校准 bin 文件（如果存在）
        # 因为一旦启动数据流，传感器就被占用，无法再执行 setbsxparam
        if load_bsx_calibration:
            self._load_bsx_calibration_before_connect()

        # 尝试连接
        success = self._start_process(perform_gyro_foc=perform_gyro_foc)

        # 如果是因为传感器没固件失败，尝试加载固件后再试一次
        if not success:
            print("连接初次尝试失败，尝试修复固件...")
            if self._boot_firmware():
                # 固件加载后，再次尝试加载 BSX 校准
                if load_bsx_calibration:
                    self._load_bsx_calibration_before_connect()
                success = self._start_process(perform_gyro_foc=perform_gyro_foc)

        return success
    
    def _load_bsx_calibration_before_connect(self):
        """在连接之前加载 BSX 校准 bin 文件（此时传感器未被占用）"""
        calibration_folder = "calibration"
        acc_bin = os.path.join(calibration_folder, "acc_calib.bin")
        gyro_bin = os.path.join(calibration_folder, "gyro_calib.bin")
        
        if not os.path.exists(acc_bin) or not os.path.exists(gyro_bin):
            return False
        
        print(">>> 检测到 BSX 校准 bin 文件，正在加载...")
        
        params = {
            "acc": ("0x201", acc_bin),
            "gyro": ("0x203", gyro_bin)
        }
        
        success_count = 0
        for name, (pid, path) in params.items():
            abs_path = os.path.abspath(path)
            # 确保使用绝对路径
            exe_abs_path = os.path.abspath(self.exe_path)
            cmd = [exe_abs_path, "setbsxparam", pid, abs_path]
            
            print(f"   执行命令: {' '.join(cmd)}")
            
            try:
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=5,
                    cwd=os.path.dirname(exe_abs_path)
                )
                
                # 检查成功的关键信息：包含 "Calibration profile ... is read from the file ... and calibrated"
                has_calibrated = "calibrated" in result.stdout.lower()
                has_calibration_profile = "calibration profile" in result.stdout.lower()
                # SPI 配置错误可以忽略（因为后续连接时会重新配置）
                has_critical_error = "error" in result.stdout.lower() and "spi configuration failed" not in result.stdout.lower() and not has_calibrated
                
                if (has_calibrated or has_calibration_profile) and result.returncode == 0:
                    print(f"    ✅ {name} 加载成功")
                    success_count += 1
                elif has_critical_error:
                    print(f"    ❌ {name} 加载失败")
                    if result.stdout:
                        # 显示关键错误信息
                        lines = result.stdout.strip().split('\n')
                        for line in lines:
                            if "error" in line.lower() and "spi configuration" not in line.lower():
                                print(f"       错误: {line.strip()[:100]}")
                else:
                    # 有 SPI 配置错误但可能仍然成功
                    if has_calibrated:
                        print(f"    ✅ {name} 加载成功（忽略 SPI 配置警告）")
                        success_count += 1
                    else:
                        print(f"    ⚠️  {name} 加载状态未知")
                        if result.stdout:
                            lines = result.stdout.strip().split('\n')
                            for line in lines[-3:]:  # 只显示最后几行
                                if line.strip():
                                    print(f"       输出: {line.strip()[:100]}")
            except subprocess.TimeoutExpired:
                print(f"    ❌ {name} 加载超时")
            except Exception as e:
                print(f"    ❌ {name} 加载出错: {e}")
        
        if success_count > 0:
            print(f">>> BSX 校准数据已加载 ({success_count}/{len(params)} 个文件)")
            return True
        else:
            print(">>> 警告: BSX 校准数据加载失败，将使用默认校准")
            return False

    def _start_process(self, perform_gyro_foc: bool = False) -> bool:
        """启动 bhy2cli 进程"""
        try:
            cmd = [self.exe_path]

            # 如果需要 Gyro FOC (链式命令：foc 3 -c 37:50)
            # 这样校准数据直接存在传感器内存里，不需要文件
            if perform_gyro_foc:
                print("启用启动时陀螺仪校准 (foc 3，直接写入传感器内存)...")
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

    def calibrate(self, measure_drift_seconds: float = 2.0, save_to_file: bool = True, calibration_folder: str = "calibration"):
        """
        将当前姿态设为零位，并初始化自适应漂移补偿

        Args:
            measure_drift_seconds: 等待时间 (秒)，让用户准备好
            save_to_file: 是否保存校准数据到 bin 文件（默认 True）
            calibration_folder: 校准文件保存目录（默认 "calibration"）
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
        
        # 保存校准数据到文件（注意：这个可能会超时，但不影响使用）
        if save_to_file:
            self.save_calibration_profile(calibration_folder)
    
    def calibrate_gravity(self, num_faces: int = None, wait_seconds: float = 2.0, save_to_file: bool = True, calibration_folder: str = "calibration", interactive: bool = True):
        """
        [已废弃] 多面重力校准方法
        
        注意：重力校准已通过 FOC (foc 3) 在传感器层面完成。
        FOC 校准数据直接写入传感器内存，不需要文件操作。
        此方法保留仅为兼容性，实际不需要调用。
        """
        print(">>> 注意: 重力校准已通过 FOC (foc 3) 在启动时自动完成")
        print(">>> FOC 校准数据直接写入传感器内存，不需要额外校准")
        return True
    
    def _calibrate_gravity_old(self, num_faces: int = None, wait_seconds: float = 2.0, save_to_file: bool = True, calibration_folder: str = "calibration", interactive: bool = True):
        """
        多面重力校准：通过校准多个面来确定传感器的完整方向
        
        原理：
            只校准一面不够，因为传感器可以任意旋转。
            需要校准至少3个正交面（或更多面）来确定传感器的完整方向。
        
        使用方法：
            1. 将传感器依次放在不同面上（比如：正面朝下、侧面朝下、背面朝下）
            2. 每次保持静止，按 Enter 记录
            3. 至少需要3个面，建议6个面（所有可能的方向）
        
        Args:
            num_faces: 要校准的面数（None 时交互式选择，默认3，建议6）
            wait_seconds: 每个面的等待时间 (秒)
            save_to_file: 是否保存到本地文件（默认 True）
            calibration_folder: 校准文件保存目录（默认 "calibration"）
            interactive: 是否交互式选择（默认 True）
        """
        if not self._connected:
            print(">>> 错误: 传感器未连接，无法进行重力校准")
            return False
        
        # 交互式选择：读取历史数据还是新校准
        if interactive:
            print("="*60)
            print(">>> 重力校准选项")
            print("="*60)
            print("1. 读取历史校准数据（如果之前已校准过）")
            print("2. 进行新校准（需要校准多个面）")
            print("="*60)
            
            choice = input(">>> 请选择 (1/2，默认2): ").strip()
            if choice == "1":
                # 尝试加载历史数据
                if self._load_gravity_calibration(calibration_folder):
                    print(">>> 已成功加载历史校准数据！")
                    return True
                else:
                    print(">>> 未找到历史校准数据，将进行新校准")
                    choice = "2"
            
            if choice == "2" or choice == "":
                # 新校准：选择面数
                print("\n>>> 请选择校准面数：")
                print("   1. 3个面（最少，快速）")
                print("   2. 6个面（推荐，更准确）")
                print("   3. 自定义面数")
                
                face_choice = input(">>> 请选择 (1/2/3，默认2): ").strip()
                if face_choice == "1":
                    num_faces = 3
                elif face_choice == "3":
                    try:
                        num_faces = int(input(">>> 请输入面数（至少3）: ").strip())
                        if num_faces < 3:
                            print(">>> 警告: 面数至少需要3个，使用3个")
                            num_faces = 3
                    except ValueError:
                        print(">>> 输入无效，使用默认值6")
                        num_faces = 6
                else:
                    num_faces = 6  # 默认6个面
        
        # 如果 num_faces 仍然为 None，使用默认值
        if num_faces is None:
            num_faces = 6
        
        print("="*60)
        print(">>> 多面重力方向校准")
        print("="*60)
        print(f">>> 需要校准 {num_faces} 个面来确定传感器的完整方向")
        print(">>> 请依次将传感器放在不同面上（比如：")
        print(">>>   1. 正面朝下")
        print(">>>   2. 背面朝下")
        print(">>>   3. 左侧面朝下")
        print(">>>   4. 右侧面朝下")
        print(">>>   5. 顶面朝下")
        print(">>>   6. 底面朝下")
        print(f">>>   或任意 {num_faces} 个不同的面）")
        print("="*60)
        
        self._gravity_samples = []
        
        for i in range(num_faces):
            print(f"\n>>> [第 {i+1}/{num_faces} 面] 请将传感器放在第 {i+1} 个面上（朝下）")
            print(">>> 保持静止，按 Enter 开始记录...")
            input()
            
            print(f">>> 正在记录（等待 {wait_seconds} 秒，请保持静止）...")
            time.sleep(wait_seconds)
            
            # 从当前四元数计算重力方向（在传感器坐标系中）
            # 重力在世界坐标系中是 [0, 0, -1]（向下）
            gravity_world = np.array([0, 0, -1])
            
            # 使用当前原始四元数将重力从世界坐标系转换到传感器坐标系
            gravity_sensor = self._raw_quat.apply(gravity_world)
            
            # 归一化
            gravity_norm = np.linalg.norm(gravity_sensor)
            if gravity_norm > 0:
                gravity_sensor = gravity_sensor / gravity_norm
            
            self._gravity_samples.append({
                'gravity_sensor': gravity_sensor.copy(),
                'quat': R.from_quat(self._raw_quat.as_quat())  # 创建新的 Rotation 对象
            })
            
            print(f">>> 已记录: 重力方向 [{gravity_sensor[0]:.3f}, {gravity_sensor[1]:.3f}, {gravity_sensor[2]:.3f}]")
        
        # 从多个样本计算传感器到世界坐标系的旋转
        if len(self._gravity_samples) >= 3:
            self._compute_sensor_to_world_rotation()
            self._gravity_calibrated = True
            print("\n>>> 多面重力校准完成！")
            print(">>> 传感器现在可以正确理解所有方向的重力了")
        else:
            print("\n>>> 警告: 样本数量不足，无法完成校准")
            return False
        
        # 保存到本地文件
        if save_to_file:
            self._save_gravity_calibration(calibration_folder)
        
        return True
    
    def _compute_sensor_to_world_rotation(self):
        """
        从多个重力样本计算传感器坐标系到世界坐标系的旋转
        使用最小二乘法或SVD来找到最佳旋转矩阵
        """
        if len(self._gravity_samples) < 3:
            return
        
        # 构建矩阵：传感器坐标系中的重力向量 -> 世界坐标系中的重力向量
        # 世界坐标系中的重力总是 [0, 0, -1]
        gravity_world = np.array([0, 0, -1])
        
        # 收集所有传感器坐标系中的重力向量
        sensor_vectors = np.array([s['gravity_sensor'] for s in self._gravity_samples])
        
        # 使用 Kabsch 算法或 SVD 找到最佳旋转
        # 目标：找到旋转 R，使得 R * sensor_vector ≈ world_vector
        # 这里我们使用所有样本的平均方向作为参考
        
        # 方法1: 使用第一个样本的四元数（如果可用）
        if len(self._gravity_samples) > 0:
            # 使用第一个样本的四元数作为初始估计
            first_quat = self._gravity_samples[0]['quat']
            # 验证：检查这个旋转是否能让传感器坐标系的重力指向世界坐标系的重力
            test_gravity = first_quat.apply(self._gravity_samples[0]['gravity_sensor'])
            
            # 如果方向接近，使用这个四元数
            if np.dot(test_gravity, gravity_world) > 0.9:  # 接近对齐
                self._sensor_to_world = first_quat
            else:
                # 否则计算逆旋转
                self._sensor_to_world = first_quat.inv()
        
        # 方法2: 使用 SVD 计算最佳旋转（更精确，但需要更多样本）
        if len(self._gravity_samples) >= 3:
            # 构建点集：传感器坐标系中的重力向量
            P = sensor_vectors
            
            # 目标点集：世界坐标系中的重力向量（都是 [0, 0, -1]）
            Q = np.tile(gravity_world, (len(sensor_vectors), 1))
            
            # 计算质心
            P_centroid = np.mean(P, axis=0)
            Q_centroid = np.mean(Q, axis=0)
            
            # 去中心化
            P_centered = P - P_centroid
            Q_centered = Q - Q_centroid
            
            # 计算协方差矩阵
            H = P_centered.T @ Q_centered
            
            # SVD
            U, S, Vt = np.linalg.svd(H)
            
            # 计算旋转矩阵
            R = Vt.T @ U.T
            
            # 确保是右手坐标系（det(R) = 1）
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            # 转换为四元数
            try:
                self._sensor_to_world = R.from_matrix(R)
            except:
                # 如果转换失败，使用之前的方法
                pass
    
    def _save_gravity_calibration(self, folder: str = "calibration"):
        """保存重力校准数据到本地文件"""
        if not os.path.exists(folder):
            os.makedirs(folder)
        
        gravity_file = os.path.join(folder, "gravity_calibration.txt")
        try:
            with open(gravity_file, 'w') as f:
                f.write(f"# 多面重力方向校准数据\n")
                f.write(f"# 格式: 每行一个面的重力向量 (x y z)\n")
                f.write(f"# 样本数量: {len(self._gravity_samples)}\n")
                for i, sample in enumerate(self._gravity_samples):
                    g = sample['gravity_sensor']
                    f.write(f"{g[0]:.9f} {g[1]:.9f} {g[2]:.9f}\n")
                
                # 保存传感器到世界坐标系的旋转（四元数）
                if self._gravity_calibrated:
                    q = self._sensor_to_world.as_quat()
                    f.write(f"# 传感器到世界坐标系旋转 (四元数 x y z w):\n")
                    f.write(f"# {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
            
            print(f">>> 重力校准数据已保存到: {gravity_file} ({len(self._gravity_samples)} 个样本)")
            return True
        except Exception as e:
            print(f">>> 警告: 保存重力校准数据失败: {e}")
            return False
    
    def _load_gravity_calibration(self, folder: str = "calibration"):
        """从本地文件加载重力校准数据"""
        gravity_file = os.path.join(folder, "gravity_calibration.txt")
        if not os.path.exists(gravity_file):
            return False
        
        try:
            self._gravity_samples = []
            quat_line = None
            
            with open(gravity_file, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        parts = line.split()
                        if len(parts) >= 3:
                            # 重力向量
                            gravity_vec = np.array([float(parts[0]), float(parts[1]), float(parts[2])])
                            self._gravity_samples.append({
                                'gravity_sensor': gravity_vec,
                                'quat': None  # 加载时不需要四元数
                            })
                        elif len(parts) == 4 and line.startswith('#'):
                            # 可能是四元数注释行，跳过
                            pass
            
            if len(self._gravity_samples) >= 3:
                # 重新计算传感器到世界坐标系的旋转
                self._compute_sensor_to_world_rotation()
                self._gravity_calibrated = True
                print(f">>> 已加载重力校准数据: {len(self._gravity_samples)} 个样本")
                return True
            else:
                print(f">>> 警告: 加载的样本数量不足 ({len(self._gravity_samples)})")
                return False
        except Exception as e:
            print(f">>> 警告: 加载重力校准数据失败: {e}")
            return False



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

    def save_calibration_profile(self, folder: str = "calibration"):
        """
        保存传感器校准数据到文件（BSX 算法参数）
        
        Args:
            folder: 校准文件保存目录（默认 "calibration"）
        """
        if not self._connected or not self.process:
            print(">>> 警告: 传感器未连接，无法保存校准数据")
            return False

        if not os.path.exists(folder):
            os.makedirs(folder)
            print(f">>> 创建校准目录: {folder}")

        # BSX 算法参数 ID: 0x201 (Accelerometer), 0x203 (Gyroscope)
        params = {
            "acc": ("0x201", os.path.join(folder, "acc_calib.bin")),
            "gyro": ("0x203", os.path.join(folder, "gyro_calib.bin"))
        }

        print(">>> 正在保存校准数据到文件...")
        success_count = 0
        
        for name, (pid, path) in params.items():
            abs_path = os.path.abspath(path)
            cmd = [self.exe_path, "getbsxparam", pid, abs_path]
            
            try:
                result = subprocess.run(
                    cmd, 
                    capture_output=True, 
                    text=True, 
                    cwd=os.path.dirname(self.exe_path),
                    timeout=5
                )
                
                if os.path.exists(abs_path):
                    file_size = os.path.getsize(abs_path)
                    print(f"    ✅ {name} 保存成功: {abs_path} ({file_size} bytes)")
                    success_count += 1
                else:
                    print(f"    ❌ {name} 保存失败: 文件未创建")
                    if result.stdout:
                        print(f"       输出: {result.stdout.strip()}")
                    if result.stderr:
                        print(f"       错误: {result.stderr.strip()}")
            except subprocess.TimeoutExpired:
                print(f"    ❌ {name} 保存超时")
            except Exception as e:
                print(f"    ❌ {name} 保存出错: {e}")
        
        if success_count > 0:
            print(f">>> 校准数据已保存 ({success_count}/{len(params)} 个文件)")
            return True
        else:
            print(">>> 警告: 所有校准数据保存失败")
            return False

    def load_calibration_profile(self, folder: str = "calibration"):
        """
        从文件加载传感器校准数据（BSX 算法参数）
        
        注意：此方法需要在传感器未连接时调用，因为一旦启动数据流，传感器就被占用。
        如果传感器已连接，此方法会失败。建议在 connect() 之前调用，或使用 connect() 的自动加载功能。
        
        Args:
            folder: 校准文件目录（默认 "calibration"）
        """
        # 如果传感器已连接，无法加载（传感器被占用）
        if self._connected or self.process:
            print(">>> 警告: 传感器已连接，无法加载校准数据")
            print(">>> 提示: BSX 校准应在连接之前加载（connect() 会自动处理）")
            return False

        # 使用内部方法加载（传感器未连接时）
        return self._load_bsx_calibration_before_connect()


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
