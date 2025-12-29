import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# 1. 加载模型
current_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e.xml")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 2. 定义 Home 姿态 (参考 Phase 6)
# Shoulder_Lift(1) 和 Elbow(2) 是我们要动的
home_qpos = np.array([0, -1.57, 0, -1.57, -1.57, 0], dtype=np.float32)

# 3. 参数
freq_shoulder = 1.4  # 大臂慢
freq_elbow = 2.8     # 小臂快

print("启动‘数学直驱’模式：这是绝对完美的轨迹，不需要训练。")

with mujoco.viewer.launch_passive(model, data) as viewer:
    # 初始化
    start_time = time.time()
    
    while viewer.is_running():
        # 获取当前仿真时间
        # 注意：这里我们用系统时间来驱动动画，或者用 data.time
        t = data.time

        # --- 核心逻辑：直接把公式写给电机 ---
        # 这就是你说的：直接告诉它每时每刻应该在哪
        
        target = home_qpos.copy()
        
        # 大臂：sin(t)
        target[1] = -1.57 + 0.6 * np.sin(freq_shoulder * t)
        
        # 小臂：sin(2t) + 0.5 (偏置，保持微屈)
        target[2] = 0.5 + 0.5 * np.sin(freq_elbow * t)

        # ----------------------------------
        
        # 将计算好的角度直接赋给控制信号 (ctrl)
        # 因为 xml 里定义的是位置伺服 (position actuator)，所以这就行了
        data.ctrl[:] = target
        
        # 物理步进
        mujoco.mj_step(model, data)

        # 渲染
        viewer.sync()
        
        # 稍微控速，不然跑太快
        time.sleep(0.002)