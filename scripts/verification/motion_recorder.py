"""
运动数据记录器
用于保存 IMU/关节数据，供后续回放和训练
"""

import numpy as np
import os
import time
from datetime import datetime


class MotionRecorder:
    """运动数据记录器"""

    def __init__(self, output_dir="./recordings"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self.is_recording = False
        self.data = {
            "timestamps": [],
            "qpos": [],           # 关节角度
            "qvel": [],           # 关节速度
            "imu_upper": [],      # 大臂 IMU (四元数)
            "imu_fore": [],       # 小臂 IMU (四元数)
            "accel_upper": [],    # 大臂加速度
            "accel_fore": [],     # 小臂加速度
        }
        self.start_time = None
        self.frequency = 50.0  # 默认采样频率

    def start(self):
        """开始记录"""
        self.is_recording = True
        self.start_time = time.time()
        self._clear_data()
        print("▶ 开始记录...")

    def stop(self):
        """停止记录"""
        self.is_recording = False
        duration = time.time() - self.start_time if self.start_time else 0
        n_frames = len(self.data["timestamps"])
        print(f"⏹ 停止记录: {n_frames} 帧, {duration:.1f} 秒")
        return self.get_recording_stats()

    def get_recording_stats(self):
        """获取录制状态"""
        duration = time.time() - self.start_time if self.start_time and self.is_recording else 0
        if not self.is_recording and self.start_time:
             # 如果停止了，计算总时长
             duration = (self.data["timestamps"][-1] - self.data["timestamps"][0]) if self.data["timestamps"] else 0

        return {
            "is_recording": self.is_recording,
            "n_frames": len(self.data["timestamps"]),
            "duration": duration
        }

    def record_frame(self, qpos=None, qvel=None,
                     imu_upper=None, imu_fore=None,
                     accel_upper=None, accel_fore=None):
        """记录一帧数据"""
        if not self.is_recording:
            return

        t = time.time() - self.start_time
        self.data["timestamps"].append(t)

        if qpos is not None:
            self.data["qpos"].append(qpos.copy())
        if qvel is not None:
            self.data["qvel"].append(qvel.copy())
        if imu_upper is not None:
            self.data["imu_upper"].append(imu_upper.copy())
        if imu_fore is not None:
            self.data["imu_fore"].append(imu_fore.copy())
        if accel_upper is not None:
            self.data["accel_upper"].append(accel_upper.copy())
        if accel_fore is not None:
            self.data["accel_fore"].append(accel_fore.copy())

    def save(self, filename=None):
        """保存数据到文件"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"motion_{timestamp}.npz"

        filepath = os.path.join(self.output_dir, filename)

        # 转换为 numpy 数组
        save_data = {"frequency": self.frequency}
        for key, values in self.data.items():
            if len(values) > 0:
                save_data[key] = np.array(values)

        np.savez(filepath, **save_data)
        print(f"💾 数据已保存: {filepath}")
        return filepath

    def load(self, filepath):
        """加载数据"""
        data = np.load(filepath, allow_pickle=True)
        self.frequency = float(data.get("frequency", 50.0))

        for key in self.data.keys():
            if key in data:
                self.data[key] = data[key].tolist()

        print(f"📂 数据已加载: {filepath}")
        print(f"   帧数: {len(self.data['timestamps'])}")
        return self.data

    def _clear_data(self):
        """清空数据"""
        for key in self.data:
            self.data[key] = []


# 格式转换工具
def convert_walk_arm_to_recording(input_path, output_path):
    """
    将 walk_arm_direct.npz 转换为标准记录格式
    """
    # 加载原始数据
    raw = np.load(input_path, allow_pickle=True)
    qpos = raw["qpos"]
    freq = float(raw["frequency"])
    n_frames = qpos.shape[0]

    # 生成时间戳
    timestamps = np.arange(n_frames) / freq

    # 保存为标准格式
    np.savez(
        output_path,
        timestamps=timestamps,
        qpos=qpos,
        frequency=freq,
        joint_names=raw["joint_names"],
    )
    print(f"✅ 转换完成: {output_path}")


if __name__ == "__main__":
    # 测试记录器
    recorder = MotionRecorder()

    recorder.start()
    for i in range(50):
        recorder.record_frame(
            qpos=np.random.randn(6),
            qvel=np.random.randn(6),
        )
        time.sleep(0.02)
    recorder.stop()

    filepath = recorder.save("test_recording.npz")

    # 测试加载
    recorder2 = MotionRecorder()
    recorder2.load(filepath)
