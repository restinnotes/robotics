from stable_baselines3 import PPO
from ur3e_env import UR3eEnv
import time

# 1. 加载环境 (开启渲染)
# 提示：确保 ur3e_env.py 中 _render_frame 里的报错行已经注释掉
env = UR3eEnv(render_mode="human")

# 2. 加载 Phase 6 训练好的模型
# 注意：如果你修改了模型保存名称，请在此处同步修改
model = PPO.load("models/ur3e_ppo_reach")

print("加载模型成功，准备展示‘双圆弧’关节追踪摆臂...")

# 重置环境
obs, _ = env.reset()

try:
    while True:
        # 使用训练好的策略进行预测
        action, _ = model.predict(obs, deterministic=True)
        
        obs, reward, terminated, truncated, info = env.step(action)
        
        # --- [关键修改点] ---
        # 匹配 Phase 6 环境中返回的 "joint_error"
        if "joint_error" in info:
            err = info['joint_error']
            # 这里的单位是弧度 (rad)
            print(f"当前主关节(大臂/小臂)平均误差: {err:.4f} rad")
        else:
            # 如果你没有删除旧模型直接运行，可能会看到这个
            print(f"等待环境信息: {info}")
            
        # 保持显示频率。0.04s 对应环境内部 20 步的物理仿真时间
        time.sleep(0.04) 

except KeyboardInterrupt:
    print("展示结束")
finally:
    env.close()