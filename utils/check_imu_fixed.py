import mujoco
from mujoco import viewer
import time
import os

# 尝试设置不同的 GL 后端
# 注意：launch_passive 可能不支持直接指定后端，但我们可以尝试设置环境变量
os.environ.setdefault('MUJOCO_GL', 'glfw')  # 默认使用 glfw

# 1. 加载模型
current_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(current_dir)  # 上级目录
xml_path = os.path.join(project_dir, "assets", "universal_robots_ur3e", "ur3e.xml")

print(f"正在加载模型: {xml_path}")
try:
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    print("✅ 模型加载成功！")
except Exception as e:
    print(f"❌ 模型加载失败: {e}")
    exit(1)

print("正在读取 IMU 数据... 现在的速度是每秒打印 1 次")
print(f"MUJOCO_GL: {os.environ.get('MUJOCO_GL', '未设置')}")

# 2. 引入一个变量记录上次打印的时间
last_print_time = 0.0

try:
    print("\n正在启动 viewer...")
    with viewer.launch_passive(model, data) as v:
        print("✅ Viewer 启动成功！")
        print("按 ESC 退出窗口")
        
        while v.is_running():
            # 物理步进
            mujoco.mj_step(model, data)

            # 3. 修正后的打印逻辑：
            # 只有当 "当前仿真时间" 比 "上次打印时间" 大于等于 1.0 秒时才触发
            if data.time - last_print_time >= 1.0:
                # 读取加速度计数据 (3维: x, y, z)
                accel = data.sensor('body_accel').data
                # 读取陀螺仪数据 (3维: x, y, z)
                gyro = data.sensor('body_gyro').data

                print(f"Time: {data.time:.1f}s | Accel: {accel} | Gyro: {gyro}")

                # 更新上次打印时间为当前仿真时间
                last_print_time = data.time

            v.sync()
            # 控制仿真循环的速度，使其接近实时
            time.sleep(0.002)
            
except Exception as e:
    print(f"\n❌ Viewer 启动失败: {e}")
    import traceback
    traceback.print_exc()
    
    print("\n" + "="*60)
    print("可能的解决方案:")
    print("="*60)
    print("\n1. 检查是否有真实的图形界面环境")
    print("2. 尝试设置环境变量:")
    print("   export MUJOCO_GL=egl")
    print("   python3 utils/check_imu_fixed.py")
    print("\n3. 或者在有图形界面的桌面环境中运行")
    print("4. 如果通过 SSH，使用 X11 转发:")
    print("   ssh -X user@host")
    exit(1)
