import mujoco
import mujoco.viewer
import time
import os

# 1. 设定模型文件的路径
current_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(current_dir)
xml_path = os.path.join(project_dir, "assets", "universal_robots_ur3e", "ur3e.xml")

print(f"正在尝试加载模型: {xml_path}")

try:
    # 2. 加载模型 (Model) 和 数据 (Data)
    # Model 包含物理参数（质量、惯量、几何形状等静态信息）
    # Data 包含实时状态（位置、速度、受力等动态信息）
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    print("模型加载成功！")

except ValueError as e:
    print(f"加载失败，请检查路径。错误信息: {e}")
    exit()

# 3. 启动交互式查看器 (Viewer)
print("启动查看器... 按 ESC 退出")
with mujoco.viewer.launch_passive(model, data) as viewer:
    # 这里的循环是模拟的主循环
    while viewer.is_running():
        # (a) 物理步进：计算下一个时间步的物理状态
        mujoco.mj_step(model, data)

        # (b) 同步画面：把计算结果画在屏幕上
        viewer.sync()

        # (c) 稍微延时一下，不然跑太快看不清（实际训练时不需要这个）
        time.sleep(0.002)