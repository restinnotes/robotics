import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np
import os

class UR3eEnv(gym.Env):
    """
    MuJoCo UR3e 强化学习环境
    - 动作: 6个关节的目标位置 (radians)
    - 观测: 18维向量 (qpos, qvel, accel, gyro)
    """
    def __init__(self, render_mode=None):
        super(UR3eEnv, self).__init__()
        
        # 1. 加载模型
        current_dir = os.path.dirname(os.path.abspath(__file__))
        xml_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e.xml")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # 2. 定义动作空间
        self.action_space = spaces.Box(
            low=-np.pi, high=np.pi, shape=(6,), dtype=np.float32
        )
        
        # 3. 定义观测空间 (18维)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(18,), dtype=np.float32
        )

        # 4. 定义目标点 (Target) [x, y, z]
        # 我们设在机器人前方高处，这是一个它伸直手能够到的地方
        self.target_pos = np.array([0.3, 0.0, 0.5])

        self.render_mode = render_mode
        self.viewer = None

    def step(self, action):
        # 1. 执行动作
        self.data.ctrl[:] = action
        
        # 2. 物理步进
        for _ in range(20):
            mujoco.mj_step(self.model, self.data)
        
        # 3. 获取观测
        obs = self._get_obs()
        
        # --- 核心修改：奖励函数 (Reward Function) ---
        # 获取末端执行器 (wrist_3_link) 的实时位置
        # 在 MuJoCo 中，可以通过 body 名称直接获取 xpos
        eef_pos = self.data.body('wrist_3_link').xpos
        
        # 计算与目标点的距离 (欧氏距离)
        distance = np.linalg.norm(eef_pos - self.target_pos)
        
        # 定义奖励：距离越小，奖励越高 (负距离是常用的 dense reward)
        # 我们加一个 -distance，意味着最佳情况是 0，平时是负数
        reward = -distance 
        
        # 记录一下距离，方便训练时看日志
        info = {"distance": distance}
        # ----------------------------------------
        
        terminated = False
        truncated = False
        
        if self.render_mode == "human":
            self._render_frame()
            
        return obs, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # 重置物理仿真状态
        mujoco.mj_resetData(self.model, self.data)
        
        # (可选) 这里可以加随机化，比如让机械臂随机摆个姿势
        # self.data.qpos[:] += np.random.uniform(-0.1, 0.1, size=6)
        
        # 必须先计算一次正向动力学，更新传感器数据
        mujoco.mj_forward(self.model, self.data)
        
        if self.render_mode == "human":
            self._render_frame()
            
        return self._get_obs(), {}



    def _get_obs(self):
        # 拼凑观测向量
        qpos = self.data.qpos.flat.copy()
        qvel = self.data.qvel.flat.copy()
        
        # 读取 IMU 数据
        accel = self.data.sensor('body_accel').data.flat.copy()
        gyro = self.data.sensor('body_gyro').data.flat.copy()
        
        # 拼接成一个 18 维的数组
        return np.concatenate([qpos, qvel, accel, gyro]).astype(np.float32)

    def _render_frame(self):
        if self.viewer is None:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()