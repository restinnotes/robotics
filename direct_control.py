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

# 2. 定义 Home 姿态
home_qpos = np.array([0, 0, 0, -1.57, -1.57, 0], dtype=np.float32)

# 3. 摆动参数
freq = 0.8               # 统一频率 (Hz)
amp_shoulder = 1.0       # 大臂摆动幅度 (rad) - 加大幅度
amp_elbow = 1.4          # 小臂摆动幅度 - 加大到140度左右

print("启动'相位同步的平面双摆'模式（修正版）：")
print("- 大臂摆到最左：小臂伸直（一条线）")
print("- 大臂摆到最右：小臂弯曲最大（夹角最大）")
print("- 弯曲过程：cos函数，越来越快")

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()

    while viewer.is_running():
        t = data.time

        # --- 核心逻辑：相位同步的双摆 ---
        target = home_qpos.copy()

        # Base 保持 0
        target[0] = 0

        # 大臂：sin(ωt)
        # sin = +1 时在最右端，sin = -1 时在最左端
        shoulder_angle = amp_shoulder * np.sin(2 * np.pi * freq * t)
        target[1] = shoulder_angle

        # 小臂：用 cos 来实现"越来越快"的效果
        # cos(ωt) 的变化率在 0 附近最大，在 ±1 附近最小
        #
        # 当 sin(ωt) = -1（最左）时，cos(ωt) = 0 → elbow = 0（伸直）
        # 当 sin(ωt) = +1（最右）时，cos(ωt) = 0 → elbow = amp（弯曲最大）
        #
        # 等等，cos 和 sin 相差 90度...
        # sin(ωt) = -1 时，ωt = -π/2，此时 cos(ωt) = 0
        # sin(ωt) = +1 时，ωt = π/2，此时 cos(ωt) = 0
        #
        # 我们要：
        # sin = -1（最左）→ elbow = 0
        # sin = +1（最右）→ elbow = max
        #
        # 用 (1 + sin(ωt))/2，但要 cos 的速度特性
        # 实际上，cos²((ωt + π/2)/2) 可以实现
        #
        # 更简单：直接用 cos²，然后调整相位
        # elbow = amp * cos²(ωt/2 + π/4)
        #
        # 算了，直接用最简单的：
        # elbow = amp * (1 + sin(ωt))² / 4  # 平方让它加速

        # 使用二次函数让变化"越来越快"
        s = np.sin(2 * np.pi * freq * t)  # -1 到 +1
        # 归一化到 0-1：(s+1)/2
        # 然后平方：((s+1)/2)²，这样在两端慢，中间快
        elbow_normalized = ((s + 1) / 2) ** 2
        elbow_angle = amp_elbow * elbow_normalized

        target[2] = elbow_angle

        # 腕部保持固定
        target[3] = -1.57
        target[4] = -1.57
        target[5] = 0

        # 控制信号
        data.ctrl[:] = target

        # 物理步进
        mujoco.mj_step(model, data)

        # 渲染
        viewer.sync()

        # 控制帧率
        time.sleep(0.002)