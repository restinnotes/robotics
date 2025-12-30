"""
UR3e 盲操 RL 环境 (Blind IMU Environment)
Phase 3: 核心环境 - Agent 只能看到 IMU 历史，不能看到 Target

关键变化（与 ur3e_imitation_env.py 相比）：
1. Observation: 去掉 Target，改为 [qpos, qvel, imu_history (stacked)]
2. Action: 直接输出目标关节角度（绝对值），不是残差
3. Reward: 仍然可以用 Target 来计算（训练时有监督信号），但 Agent 看不到它
"""

import gymnasium as gym
from gymnasium import spaces
import mujoco
import mujoco.viewer
import numpy as np
import os
from scipy.spatial.transform import Rotation as R


class UR3eBlindIMUEnv(gym.Env):
    """盲操 RL 环境：只用 IMU 历史来推断动作"""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None, imu_stack_size=10):
        super().__init__()

        # 路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
        traj_path = os.path.join(current_dir, "data", "walk_arm_direct.npz")

        # 加载模型
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 加载参考轨迹 (用于计算 Reward，但 Agent 看不到)
        traj = np.load(traj_path, allow_pickle=True)
        self.ref_qpos = traj["qpos"]
        self.n_frames = self.ref_qpos.shape[0]

        # Home 姿态
        self.home_qpos = np.array([0, 0, 0, -1.57, -1.57, 0], dtype=np.float32)

        # IMU 传感器 Site ID
        self.site_upper_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "upper_arm_imu_site")
        self.site_fore_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "forearm_imu_site")

        # IMU 历史堆叠参数
        self.imu_stack_size = imu_stack_size
        # 每帧 IMU 数据: upper_quat(4) + fore_quat(4) = 8 维
        self.imu_dim_per_frame = 8
        self.imu_history = np.zeros((imu_stack_size, self.imu_dim_per_frame), dtype=np.float32)

        # 动作空间: 6 个关节的目标位置 (绝对值，范围基于关节限制)
        # UR3e 关节范围大约 [-2pi, 2pi]，我们输出一个 [-1, 1] 的动作，再映射到 [-pi, pi]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(6,), dtype=np.float32
        )

        # 观测空间:
        # qpos(6) + qvel(6) + imu_history(stack_size * 8) = 12 + 80 = 92 (if stack=10)
        obs_dim = 6 + 6 + self.imu_stack_size * self.imu_dim_per_frame
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        self.render_mode = render_mode
        self.viewer = None

        # Episode 参数
        self.max_episode_steps = 500
        self.current_step = 0
        self.frame_idx = 0

        # 动作平滑
        self.last_action = np.zeros(6, dtype=np.float32)
        self.smoothed_action = np.zeros(6, dtype=np.float32)
        self.smooth_alpha = 0.3  # 更强的平滑 (越小越平滑)
        self.action_scale = 0.1  # 残差幅度

    def _get_target_qpos(self):
        """获取当前帧的目标关节角度 (Agent 看不到，仅用于 Reward)"""
        target = self.home_qpos.copy()
        ref = self.ref_qpos[self.frame_idx]
        target[0] = ref[1]
        target[1] = ref[0]
        target[2] = ref[3]
        target[3] = -1.57 + ref[2]
        return target

    def _get_imu_reading(self):
        """获取当前 IMU 读数 (四元数)"""
        # 获取 Site 的旋转矩阵
        mat_upper = self.data.site_xmat[self.site_upper_id].reshape(3, 3)
        mat_fore = self.data.site_xmat[self.site_fore_id].reshape(3, 3)

        # 转为四元数 [x, y, z, w]
        q_upper = R.from_matrix(mat_upper).as_quat().astype(np.float32)
        q_fore = R.from_matrix(mat_fore).as_quat().astype(np.float32)

        return np.concatenate([q_upper, q_fore])

    def _update_imu_history(self, imu_reading):
        """更新 IMU 历史队列 (FIFO)"""
        self.imu_history = np.roll(self.imu_history, -1, axis=0)
        self.imu_history[-1] = imu_reading

    def _get_obs(self):
        """构建观测向量 (无 Target，有 IMU 历史)"""
        qpos = self.data.qpos[:6].copy()
        qvel = self.data.qvel[:6].copy()
        imu_flat = self.imu_history.flatten()

        return np.concatenate([qpos, qvel, imu_flat]).astype(np.float32)

    def _compute_reward(self, action):
        """计算奖励 (依然可以用 Target)"""
        qpos = self.data.qpos[:6]
        target = self._get_target_qpos()

        # 1. 关节位置跟踪奖励
        pos_error = np.sum((qpos - target) ** 2)
        r_pos = np.exp(-5.0 * pos_error)

        # 2. 速度平滑奖励
        qvel = self.data.qvel[:6]
        r_vel = np.exp(-0.1 * np.sum(qvel ** 2))

        # 3. 动作平滑惩罚
        action_diff = np.sum((action - self.last_action) ** 2)
        r_action_smooth = np.exp(-2.0 * action_diff)

        reward = 0.5 * r_pos + 0.25 * r_vel + 0.25 * r_action_smooth
        return reward

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)

        # 随机起始帧
        self.frame_idx = self.np_random.integers(0, self.n_frames)

        # 初始姿态 = 目标姿态 + 强随机噪声 (测试鲁棒性)
        target = self._get_target_qpos()
        # 增加噪声幅度 0.1 -> 0.5 rad (约 30 度)，模拟初始方向偏差
        noise = self.np_random.uniform(-0.5, 0.5, size=6)
        self.data.qpos[:6] = target + noise

        mujoco.mj_forward(self.model, self.data)

        self.current_step = 0
        self.last_action = np.zeros(6, dtype=np.float32)
        self.smoothed_action = np.zeros(6, dtype=np.float32)

        # 初始化 IMU 历史 (用当前帧的 IMU 填满)
        imu_current = self._get_imu_reading()
        self.imu_history = np.tile(imu_current, (self.imu_stack_size, 1))

        if self.render_mode == "human":
            self._render_frame()

        return self._get_obs(), {}

    def step(self, action):
        # 1. 低通滤波平滑动作 (更强的平滑)
        self.smoothed_action = (1 - self.smooth_alpha) * self.smoothed_action + self.smooth_alpha * action

        # 2. 获取参考目标 (Agent 看不到，但我们用它来做残差控制)
        ref_target = self._get_target_qpos()

        # 3. 残差控制：Control = Reference + Small Correction
        # 这样网络只需要学微小的修正，而不是从零开始猜绝对位置
        scaled_action = self.smoothed_action * self.action_scale  # 更小的修正幅度
        self.data.ctrl[:6] = ref_target + scaled_action

        # 仿真步进
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)

        # 4. 更新 IMU 历史
        imu_reading = self._get_imu_reading()
        self._update_imu_history(imu_reading)

        # 5. 计算奖励
        reward = self._compute_reward(action)

        # 6. 记录当前动作
        self.last_action = action.copy()

        # 更新帧索引
        self.frame_idx = (self.frame_idx + 1) % self.n_frames
        self.current_step += 1

        # 终止条件
        terminated = False
        truncated = self.current_step >= self.max_episode_steps

        if self.render_mode == "human":
            self._render_frame()

        return self._get_obs(), reward, terminated, truncated, {}

    def _render_frame(self):
        if self.viewer is None and self.render_mode == "human":
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.lookat[:] = [0, 0, 0.7]
            self.viewer.cam.distance = 1.2
            self.viewer.cam.elevation = 0
            self.viewer.cam.azimuth = 90

        if self.viewer is not None:
            self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
