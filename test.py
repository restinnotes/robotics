import mujoco
import mujoco.viewer
import time
import os

# 1. 加载模型
xml_path = os.path.join("assets", "universal_robots_ur3e", "ur3e.xml")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

print("正在读取 IMU 数据... 现在的速度是每秒打印 1 次")

# 2. 引入一个变量记录上次打印的时间
last_print_time = 0.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 物理步进
        mujoco.mj_step(model, data)
        
        # 3. 修正后的打印逻辑：
        # 只有当 “当前仿真时间” 比 “上次打印时间” 大于等于 1.0 秒时才触发
        if data.time - last_print_time >= 1.0:
            # 读取加速度计数据 (3维: x, y, z)
            accel = data.sensor('body_accel').data
            # 读取陀螺仪数据 (3维: x, y, z)
            gyro = data.sensor('body_gyro').data
            
            print(f"Time: {data.time:.1f}s | Accel: {accel} | Gyro: {gyro}")
            
            # 更新上次打印时间为当前仿真时间
            last_print_time = data.time
            
        viewer.sync()
        # 控制仿真循环的速度，使其接近实时
        time.sleep(0.002)