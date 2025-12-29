from stable_baselines3 import PPO
from ur3e_env import UR3eEnv
import os

# 1. 创建环境
# 训练时不需要 render_mode="human"，因为图形界面会拖慢训练速度
env = UR3eEnv(render_mode=None)

# 2. 定义模型 (PPO 算法)
# MlpPolicy 表示使用多层感知机 (神经网络)
# verbose=1 会在终端打印训练进度
model = PPO("MlpPolicy", env, verbose=1)

print("开始训练... (你可以去喝杯咖啡，这需要几分钟)")

# 3. 开始训练
# total_timesteps 是训练的总步数。
# 对于简单的 Reach 任务，50,000 步通常能看到初步效果
model.learn(total_timesteps=50000)

# 4. 保存模型
models_dir = "models"
if not os.path.exists(models_dir):
    os.makedirs(models_dir)
model.save(f"{models_dir}/ur3e_ppo_reach")

print("训练完成！模型已保存。")