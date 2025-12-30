"""
UR3e 盲操 RL 环境 - 正确架构版
Phase 3: 核心环境

关键架构：
1. "虚拟用户 IMU"：从参考轨迹生成，模拟人戴的 IMU
2. "机器人状态"：当前机器人位置，用于 Observation 和 Reward
3. imu_solver 使用"虚拟用户 IMU"进行解算，不是机器人自己的 IMU

这样模拟了真实场景：人动 → 人的 IMU 变化 → 解算器 → 机器人跟着动
"""

import gymnasium as gym
from gymnasium import spaces
import mujoco
import mujoco.viewer
import numpy as np
import os
from scipy.spatial.transform import Rotation as R

from utils.imu_solver import IMUSolver


class UR3eBlindIMUEnv(gym.Env):
    """盲操 RL 环境：正确的 IMU 架构"""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None, imu_stack_size=10, calib_randomize_deg=0.0):
        super().__init__()

        # 路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
        traj_path = os.path.join(current_dir, "data", "walk_arm_direct.npz")

        # 加载模型 (用于生成虚拟用户 IMU 和运行机器人)
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 第二份模型数据 (用于生成虚拟用户 IMU)
        self.user_data = mujoco.MjData(self.model)

        # 加载参考轨迹
        traj = np.load(traj_path, allow_pickle=True)
        self.ref_qpos = traj["qpos"]
        self.n_frames = self.ref_qpos.shape[0]

        # Home 姿态
        self.home_qpos = np.array([0, 0, 0, -1.57, -1.57, 0], dtype=np.float32)

        # IMU 传感器 Site ID
        self.site_upper_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "upper_arm_imu_site")
        self.site_fore_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "forearm_imu_site")

        # IMU 解算器
        self.imu_solver = IMUSolver()

        # 校准随机化 (模拟用户戴歪 IMU)
        self.calib_randomize_deg = calib_randomize_deg
        self.calib_offset_rot = None  # 每个Episode随机生成

        # IMU 历史堆叠参数
        self.imu_stack_size = imu_stack_size
        self.imu_dim_per_frame = 8
        self.imu_history = np.zeros((imu_stack_size, self.imu_dim_per_frame), dtype=np.float32)

        # 动作空间
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=np.float32)

        # 观测空间
        # qpos(6) + qvel(6) + imu_history(stack*8)
        obs_dim = 6 + 6 + self.imu_stack_size * self.imu_dim_per_frame
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)

        self.render_mode = render_mode
        self.viewer = None

        # Episode 参数
        self.max_episode_steps = 500
        self.current_step = 0
        self.frame_idx = 0

        # 动作平滑
        self.last_action = np.zeros(6, dtype=np.float32)
        self.smoothed_action = np.zeros(6, dtype=np.float32)
        self.smooth_alpha = 0.3
        self.action_scale = 0.1

    def _get_target_qpos(self):
        """获取当前帧的目标关节角度"""
        target = self.home_qpos.copy()
        ref = self.ref_qpos[self.frame_idx]
        target[0] = ref[1]
        target[1] = ref[0]
        target[2] = ref[3]
        target[3] = -1.57 + ref[2]
        return target

    def _get_virtual_user_imu(self, target_qpos):
        """
        生成虚拟用户 IMU 数据
        模拟：人戴着 IMU 摆出 target_qpos 对应的姿势
        """
        # 把"虚拟用户"（另一份模型数据）设置到目标位置
        self.user_data.qpos[:6] = target_qpos
        mujoco.mj_kinematics(self.model, self.user_data)

        # 读取虚拟用户身上的 IMU
        mat_upper = self.user_data.site_xmat[self.site_upper_id].reshape(3, 3)
        mat_fore = self.user_data.site_xmat[self.site_fore_id].reshape(3, 3)
        q_upper = R.from_matrix(mat_upper).as_quat().astype(np.float32)
        q_fore = R.from_matrix(mat_fore).as_quat().astype(np.float32)

        # 施加校准偏置 (整个Episode保持一致)
        if self.calib_offset_rot is not None:
            q_upper = (R.from_quat(q_upper) * self.calib_offset_rot).as_quat().astype(np.float32)
            q_fore = (R.from_quat(q_fore) * self.calib_offset_rot).as_quat().astype(np.float32)

        return q_upper, q_fore

    def _update_imu_history(self, q_upper, q_fore):
        """更新 IMU 历史队列"""
        imu_reading = np.concatenate([q_upper, q_fore])
        self.imu_history = np.roll(self.imu_history, -1, axis=0)
        self.imu_history[-1] = imu_reading

    def _get_obs(self):
        """构建观测向量"""
        qpos = self.data.qpos[:6].copy()
        qvel = self.data.qvel[:6].copy()
        imu_flat = self.imu_history.flatten()
        return np.concatenate([qpos, qvel, imu_flat]).astype(np.float32)

    def _compute_reward(self, action):
        """计算奖励"""
        qpos = self.data.qpos[:6]
        target = self._get_target_qpos()

        pos_error = np.sum((qpos - target) ** 2)
        r_pos = np.exp(-5.0 * pos_error)

        qvel = self.data.qvel[:6]
        r_vel = np.exp(-0.1 * np.sum(qvel ** 2))

        action_diff = np.sum((action - self.last_action) ** 2)
        r_action_smooth = np.exp(-2.0 * action_diff)

        reward = 0.5 * r_pos + 0.25 * r_vel + 0.25 * r_action_smooth
        return reward

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)

        # 随机起始帧
        self.frame_idx = self.np_random.integers(0, self.n_frames)

        # 初始姿态
        target = self._get_target_qpos()
        noise = self.np_random.uniform(-0.3, 0.3, size=6)
        self.data.qpos[:6] = target + noise

        mujoco.mj_forward(self.model, self.data)

        # === 校准随机化: 模拟用户戴歪 IMU ===
        if self.calib_randomize_deg > 0:
            # 生成随机旋转偏置 (±calib_randomize_deg 度)
            calib_offset_euler = self.np_random.uniform(
                -self.calib_randomize_deg, self.calib_randomize_deg, size=3
            )
            self.calib_offset_rot = R.from_euler('xyz', calib_offset_euler, degrees=True)
        else:
            self.calib_offset_rot = None

        # 校准 IMU 解算器 (只在开始时做一次)
        # 注意：_get_virtual_user_imu 会自动施加 calib_offset_rot
        q_upper, q_fore = self._get_virtual_user_imu(target)
        self.imu_solver.calibrate(q_upper, q_fore, offset_joints=target)

        self.current_step = 0
        self.last_action = np.zeros(6, dtype=np.float32)
        self.smoothed_action = np.zeros(6, dtype=np.float32)

        # 初始化 IMU 历史
        self.imu_history = np.tile(np.concatenate([q_upper, q_fore]), (self.imu_stack_size, 1))

        if self.render_mode == "human":
            self._render_frame()

        return self._get_obs(), {}


    def step(self, action):
        # 1. 平滑动作
        self.smoothed_action = (1 - self.smooth_alpha) * self.smoothed_action + self.smooth_alpha * action

        # 2. 获取当前帧的目标 (用于生成虚拟用户 IMU)
        target_qpos = self._get_target_qpos()

        # 3. 生成虚拟用户 IMU (模拟人戴着 IMU)
        q_upper, q_fore = self._get_virtual_user_imu(target_qpos)

        # 4. 用 imu_solver 解算关节角度 (只用虚拟用户 IMU，不看 target_qpos!)
        solver_output = self.imu_solver.solve(q_upper, q_fore)

        # 5. 残差控制
        scaled_action = self.smoothed_action * self.action_scale
        self.data.ctrl[:6] = solver_output + scaled_action

        # 仿真步进
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)

        # 6. 更新 IMU 历史
        self._update_imu_history(q_upper, q_fore)

        # 7. 计算奖励
        reward = self._compute_reward(action)

        # 8. 记录动作
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
