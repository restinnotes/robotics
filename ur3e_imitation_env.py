"""
UR3e 模仿学习环境
使用宇树人形机器人轨迹作为参考目标进行训练
"""

import gymnasium as gym
from gymnasium import spaces
import mujoco
import mujoco.viewer
import numpy as np
import os


class UR3eImitationEnv(gym.Env):
    """基于关节跟踪奖励的 UR3e 模仿学习环境"""

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None, use_both_arms=True):
        super().__init__()

        # 路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
        traj_path = os.path.join(current_dir, "data", "walk_arm_direct.npz")

        # 加载模型
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 加载参考轨迹
        traj = np.load(traj_path, allow_pickle=True)
        self.ref_qpos = traj["qpos"]
        self.ref_frequency = float(traj["frequency"])
        self.n_frames = self.ref_qpos.shape[0]
        self.use_both_arms = use_both_arms

        # Home 姿态
        self.home_qpos = np.array([0, 0, 0, -1.57, -1.57, 0], dtype=np.float32)

        # 动作空间: 6 个关节的位置增量
        self.action_space = spaces.Box(
            low=-0.5, high=0.5, shape=(6,), dtype=np.float32
        )

        # 观测空间: qpos(6) + qvel(6) + target(6) = 18
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32
        )

        self.render_mode = render_mode
        self.viewer = None

        # Episode 参数
        self.max_episode_steps = 500
        self.current_step = 0
        self.frame_idx = 0

        # 动作平滑相关
        self.last_action = np.zeros(6, dtype=np.float32)
        self.smoothed_action = np.zeros(6, dtype=np.float32)
        self.action_scale = 0.1  # 减小动作幅度
        self.smooth_alpha = 0.3  # 低通滤波系数 (越小越平滑)

    def _get_target_qpos(self):
        """获取当前帧的目标关节角度"""
        target = self.home_qpos.copy()

        # 始终使用左臂数据 (与 direct_control_vertical 一致)
        ref = self.ref_qpos[self.frame_idx]

        # 映射 (与 direct_control_vertical.py 完全一致)
        target[0] = ref[1]           # l_arm_shx -> shoulder_pan
        target[1] = ref[0]           # l_arm_shy -> shoulder_lift
        target[2] = ref[3]           # left_elbow -> elbow
        target[3] = -1.57 + ref[2]   # l_arm_shz -> wrist_1

        return target

    def _get_obs(self):
        """构建观测向量"""
        qpos = self.data.qpos[:6].copy()
        qvel = self.data.qvel[:6].copy()
        target = self._get_target_qpos()

        return np.concatenate([qpos, qvel, target]).astype(np.float32)

    def _compute_reward(self, action):
        """计算奖励"""
        qpos = self.data.qpos[:6]
        target = self._get_target_qpos()

        # 1. 关节位置跟踪奖励 (主要目标)
        pos_error = np.sum((qpos - target) ** 2)
        r_pos = np.exp(-5.0 * pos_error)

        # 2. 速度平滑奖励 (防止关节速度过大)
        qvel = self.data.qvel[:6]
        r_vel = np.exp(-0.1 * np.sum(qvel ** 2))

        # 3. 动作平滑惩罚 (防止神经网络输出高频噪声)
        action_diff = np.sum((action - self.last_action) ** 2)
        r_action_smooth = np.exp(-2.0 * action_diff)

        # 总奖励：位置60% + 速度20% + 动作平滑20%
        reward = 0.6 * r_pos + 0.2 * r_vel + 0.2 * r_action_smooth

        return reward

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # 重置仿真
        mujoco.mj_resetData(self.model, self.data)

        # 随机起始帧
        self.frame_idx = self.np_random.integers(0, self.n_frames)

        # 初始姿态 = 目标姿态 + 小噪声
        target = self._get_target_qpos()
        self.data.qpos[:6] = target + self.np_random.uniform(-0.1, 0.1, size=6)

        mujoco.mj_forward(self.model, self.data)

        self.current_step = 0

        # 重置动作平滑状态
        self.last_action = np.zeros(6, dtype=np.float32)
        self.smoothed_action = np.zeros(6, dtype=np.float32)

        if self.render_mode == "human":
            self._render_frame()

        return self._get_obs(), {}

    def step(self, action):
        # 1. 低通滤波: 平滑 action 输出
        self.smoothed_action = (1 - self.smooth_alpha) * self.smoothed_action + self.smooth_alpha * action

        # 2. 获取当前帧的参考目标 (Teacher Pose)
        ref_target = self._get_target_qpos()

        # 3. 应用动作 (Residual Control)
        # Control = Reference Target + Smoothed Action (微调)
        scaled_action = self.smoothed_action * self.action_scale
        self.data.ctrl[:6] = ref_target + scaled_action

        # 仿真步进（多步以提高稳定性）
        for _ in range(10):
            mujoco.mj_step(self.model, self.data)

        # 4. 计算奖励 (传入原始 action 用于平滑惩罚计算)
        reward = self._compute_reward(action)

        # 5. 记录当前 action 用于下一步的平滑惩罚
        self.last_action = action.copy()

        # 更新帧索引
        self.frame_idx = (self.frame_idx + 1) % self.n_frames
        self.current_step += 1

        # 检查终止条件
        terminated = False
        truncated = self.current_step >= self.max_episode_steps

        if self.render_mode == "human":
            self._render_frame()

        return self._get_obs(), reward, terminated, truncated, {}

    def _render_frame(self):
        if self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.lookat[:] = [0, 0, 0.3]
            self.viewer.cam.distance = 1.5
            self.viewer.cam.elevation = -20
            self.viewer.cam.azimuth = 90
        self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None


# 验证环境
if __name__ == "__main__":
    from gymnasium.utils.env_checker import check_env

    env = UR3eImitationEnv()
    check_env(env, warn=True)
    print("✅ 环境验证通过!")

    # 测试运行
    obs, _ = env.reset()
    print(f"观测维度: {obs.shape}")

    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"Step {i}: reward={reward:.4f}")

    env.close()
