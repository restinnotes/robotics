from ur3e_env import UR3eEnv
import time
import numpy as np

# 1. 实例化环境，开启渲染模式
print("初始化环境...")
env = UR3eEnv(render_mode="human")

# 2. 重置环境，拿到第一个观测值
obs, info = env.reset()
print(f"初始观测向量长度: {len(obs)}")
print(f"初始观测值: {obs}")

print("开始随机动作测试 (按 Ctrl+C 停止)...")

try:
    # 模拟 1000 步
    for i in range(1000):
        # 3. 随机生成一个动作 (AI 的大脑在这里工作)
        # action_space.sample() 会自动生成一个符合范围的随机动作
        action = env.action_space.sample()
        
        # 4. 环境推进一步
        obs, reward, terminated, truncated, info = env.step(action)
        
        # 打印一下数据看看是不是活的
        if i % 10 == 0:
            # 观测向量的第 12,13,14 位是加速度
            accel_z = obs[14] 
            print(f"Step {i} | 奖励: {reward} | Z轴加速度: {accel_z:.2f}")
            
        time.sleep(0.05) # 稍微慢一点，让人眼能跟上

except KeyboardInterrupt:
    print("测试停止")
finally:
    env.close()