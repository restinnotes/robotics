"""
合成噪声注入器 (Synthetic Noise Injector)
用于在干净的 IMU 数据上叠加真实传感器的各种噪声

支持的噪声类型:
1. 高斯白噪声 (Gaussian White Noise) - 传感器抖动
2. 随机偏移 (Bias) - 固定误差
3. 累积漂移 (Drift) - 随时间累积的误差
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


class NoiseInjector:
    """IMU 数据噪声注入器"""

    def __init__(
        self,
        gaussian_std: float = 0.02,      # 高斯噪声标准差 (弧度)
        bias_range: float = 0.05,         # Bias 范围 (弧度)
        drift_rate: float = 0.001,        # 漂移速率 (弧度/帧)
        seed: int = None
    ):
        self.gaussian_std = gaussian_std
        self.bias_range = bias_range
        self.drift_rate = drift_rate

        self.rng = np.random.default_rng(seed)

        # 每个 Episode 随机生成的固定 Bias
        self.bias = None
        # 累积漂移量
        self.accumulated_drift = None

    def reset(self):
        """每个 Episode 开始时重置噪声状态"""
        # 生成新的随机 Bias (每个episode固定)
        self.bias = self.rng.uniform(-self.bias_range, self.bias_range, size=3)
        # 重置漂移
        self.accumulated_drift = np.zeros(3)

    def inject_noise_to_quaternion(self, q: np.ndarray) -> np.ndarray:
        """
        对四元数施加噪声

        Args:
            q: 原始四元数 [x, y, z, w]

        Returns:
            带噪声的四元数 [x, y, z, w]
        """
        if self.bias is None:
            self.reset()

        # 1. 转为欧拉角 (方便添加角度噪声)
        r = R.from_quat(q)
        euler = r.as_euler('xyz', degrees=False)  # [roll, pitch, yaw]

        # 2. 添加高斯噪声
        gaussian_noise = self.rng.normal(0, self.gaussian_std, size=3)

        # 3. 添加固定 Bias
        bias_noise = self.bias

        # 4. 累积漂移 (每帧累加一点)
        drift_step = self.rng.uniform(-self.drift_rate, self.drift_rate, size=3)
        self.accumulated_drift += drift_step

        # 5. 叠加所有噪声
        noisy_euler = euler + gaussian_noise + bias_noise + self.accumulated_drift

        # 6. 转回四元数
        noisy_r = R.from_euler('xyz', noisy_euler, degrees=False)
        return noisy_r.as_quat().astype(np.float32)

    def inject_noise_to_imu_reading(
        self,
        q_upper: np.ndarray,
        q_fore: np.ndarray
    ) -> tuple:
        """
        对一组 IMU 读数 (大臂+前臂) 施加噪声

        Args:
            q_upper: 大臂四元数 [x, y, z, w]
            q_fore: 前臂四元数 [x, y, z, w]

        Returns:
            (noisy_q_upper, noisy_q_fore)
        """
        noisy_upper = self.inject_noise_to_quaternion(q_upper)
        noisy_fore = self.inject_noise_to_quaternion(q_fore)
        return noisy_upper, noisy_fore


class NoisyIMUWrapper:
    """
    IMU 数据包装器：在读取虚拟 IMU 时自动注入噪声
    可以直接替换环境中的 _get_imu_reading 方法
    """

    def __init__(self, injector: NoiseInjector = None):
        self.injector = injector or NoiseInjector()

    def wrap_reading(self, imu_reading: np.ndarray) -> np.ndarray:
        """
        包装 8 维 IMU 读数 [upper_quat(4), fore_quat(4)]
        """
        q_upper = imu_reading[:4]
        q_fore = imu_reading[4:]

        noisy_upper, noisy_fore = self.injector.inject_noise_to_imu_reading(
            q_upper, q_fore
        )

        return np.concatenate([noisy_upper, noisy_fore])


# 预设噪声配置
NOISE_PRESETS = {
    "clean": {"gaussian_std": 0, "bias_range": 0, "drift_rate": 0},
    "mild": {"gaussian_std": 0.01, "bias_range": 0.02, "drift_rate": 0.0005},
    "moderate": {"gaussian_std": 0.03, "bias_range": 0.05, "drift_rate": 0.001},
    "severe": {"gaussian_std": 0.05, "bias_range": 0.1, "drift_rate": 0.002},
    "extreme": {"gaussian_std": 0.1, "bias_range": 0.2, "drift_rate": 0.005},
}


def get_injector_by_preset(preset_name: str, seed: int = None) -> NoiseInjector:
    """根据预设名称创建噪声注入器"""
    if preset_name not in NOISE_PRESETS:
        raise ValueError(f"Unknown preset: {preset_name}. Available: {list(NOISE_PRESETS.keys())}")

    return NoiseInjector(**NOISE_PRESETS[preset_name], seed=seed)


if __name__ == "__main__":
    # 测试噪声注入
    print("Testing Noise Injector...")

    # 创建一个中等噪声的注入器
    injector = get_injector_by_preset("moderate", seed=42)
    injector.reset()

    # 模拟 10 帧 IMU 数据
    original_quat = np.array([0, 0, 0, 1], dtype=np.float32)  # 单位四元数

    print("Original Quaternion:", original_quat)
    print("\nNoisy Quaternions over 10 frames:")

    for i in range(10):
        noisy_q = injector.inject_noise_to_quaternion(original_quat.copy())
        # 计算与原始的角度差
        r_orig = R.from_quat(original_quat)
        r_noisy = R.from_quat(noisy_q)
        angle_diff = (r_orig.inv() * r_noisy).magnitude() * 180 / np.pi
        print(f"  Frame {i}: angle_diff = {angle_diff:.2f}°")

    print("\nDone!")
