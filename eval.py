from stable_baselines3 import PPO
from ur3e_env import UR3eEnv
import time

# 1. 加载环境 (这次开启 render 来看它表演)
env = UR3eEnv(render_mode="human")

# 2. 加载训练好的模型
model = PPO.load("models/ur3e_ppo_reach")

print("加载模型成功，准备展示...")
obs, _ = env.reset()

try:
    while True:
        # 让 AI 决定动作，deterministic=True 表示不使用随机探索
        action, _ = model.predict(obs, deterministic=True)
        
        obs, reward, terminated, truncated, info = env.step(action)
        
        # 打印当前距离目标的距离
        print(f"当前距离目标: {info['distance']:.3f} 米")
        
        time.sleep(0.04) # 实时速度演示

except KeyboardInterrupt:
    print("展示结束")