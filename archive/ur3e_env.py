import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
import os

class UR3eEnv(gym.Env):
    def __init__(self, render_mode=None):
        super(UR3eEnv, self).__init__()

        # 1. 加载模型
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(current_dir)  # archive -> project root
        xml_path = os.path.join(project_dir, "assets", "universal_robots_ur3e", "ur3e.xml")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # 2. 动作空间 (6个关节)
        self.action_space = spaces.Box(
            low=-np.pi, high=np.pi, shape=(6,), dtype=np.float32
        )

        # 3. 观测空间 (24维)
        # 包含：当前关节角度(6) + 关节速度(6) + 目标关节角度(6) + 误差(6)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(24,), dtype=np.float32
        )

        self.render_mode = render_mode
        self.viewer = None

        # --- 摆臂参数 ---
        self.freq_shoulder = 1.4  # 大臂频率
        self.freq_elbow = 2.8     # 小臂频率 (按你说的，更快，2倍频)

        # 定义一个“Home”姿态 (让其他无关关节保持不动)
        # UR3e 典型姿态: Base=0, Shoulder=-1.57(竖直), Elbow=0, Wrist1=-1.57, Wrist2=-1.57, Wrist3=0
        self.home_qpos = np.array([0, -1.57, 0, -1.57, -1.57, 0], dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        # 初始姿态设为 Home 附近
        self.data.qpos[:] = self.home_qpos + np.random.uniform(-0.05, 0.05, size=6)

        mujoco.mj_forward(self.model, self.data)

        # 初始化目标关节角度
        self._update_joint_target()

        if self.render_mode == "human":
            self._render_frame()

        return self._get_obs(), {}

    def step(self, action):
        # 1. 计算当前的“目标关节角度”
        self._update_joint_target()

        # 2. 执行动作
        self.data.ctrl[:] = action
        for _ in range(20):
            mujoco.mj_step(self.model, self.data)

        # 3. 奖励函数 v5.0 (直接关节追踪)
        # 逻辑极其简单：现在的关节角度 vs 目标的关节角度
        # 越接近越好

        current_qpos = self.data.qpos.flat.copy()

        # 计算每个关节的误差
        diff = current_qpos - self.target_qpos
        # 我们主要关心 Shoulder_Lift (idx 1) 和 Elbow (idx 2)
        # 加大这两个关节权重的惩罚
        weights = np.array([0.1, 1.0, 1.0, 0.1, 0.1, 0.1])
        weighted_error = np.sum(np.abs(diff) * weights)

        # 奖励：误差越小分越高
        reward = -weighted_error

        # 动作平滑惩罚 (避免高频抖动)
        reward -= np.sum(np.square(action)) * 0.01

        obs = self._get_obs()

        # 为了 Eval 能看到误差，我们记录主关节的平均误差
        main_joint_error = (np.abs(diff[1]) + np.abs(diff[2])) / 2
        info = {"joint_error": main_joint_error}

        terminated = False
        truncated = False
        if self.render_mode == "human":
            self._render_frame()

        return obs, reward, terminated, truncated, info

    def _update_joint_target(self):
        """
        核心逻辑：直接计算关节的目标角度
        """
        t = self.data.time

        # 复制一份 Home 姿态作为基准
        target = self.home_qpos.copy()

        # --- 你的逻辑：两个圆弧 ---

        # 1. 大臂 (Shoulder Lift, Index 1)
        # 在 -1.57 (竖直) 的基础上前后摆动 +/- 0.6弧度
        target[1] = -1.57 + 0.6 * np.sin(self.freq_shoulder * t)

        # 2. 小臂 (Elbow, Index 2)
        # 它是相对于大臂动的。
        # 你的要求：速度更快 (2倍频)
        # 我们让它在 0 度 (伸直) 到 1.0 度 (弯曲) 之间摆动
        # sin(2*t) 实现快频
        target[2] = 0.5 + 0.5 * np.sin(self.freq_elbow * t)

        # 其他关节保持 Home 姿态不变 (锁死)
        self.target_qpos = target

    def _get_obs(self):
        qpos = self.data.qpos.flat.copy()
        qvel = self.data.qvel.flat.copy()

        target = self.target_qpos.copy()
        error = qpos - target

        # 观测包含：当前状态、目标状态、误差
        return np.concatenate([qpos, qvel, target, error]).astype(np.float32)

    def _render_frame(self):
        if self.viewer is None:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()