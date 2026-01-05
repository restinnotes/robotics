"""
BHy2CLI 单轴机械臂控制 (Lift Only)
================================
使用 Game Rotation Vector (ID 37) 控制 MuJoCo 机械臂。
**仅控制大臂俯仰 (Lift)**，水平旋转 (Pan) 被锁定。

原理:
    使用向量投影 (Vector Projection) 取代欧拉角。
    将传感器视为一个指向矢。计算该指向矢在垂直方向的分量 (Z分量) 来确定俯仰角。
    这种方法完全避免了 Gimbal Lock (死锁) 和 Yaw 轴旋转带来的干扰。
"""

import time
import numpy as np
import mujoco
import mujoco.viewer
import sys
import os
import argparse
from scipy.spatial.transform import Rotation as R

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

# 切换到项目根目录
os.chdir(project_root)

from utils.bhy2cli_receiver import BHy2CLIReceiver


def get_pitch_from_vector(orientation: R) -> float:
    """
    通过向量投影计算俯仰角 (Pitch)。

    原理:
    1. 假设传感器初始指向 X 轴正方向 [1, 0, 0]。
    2. 用当前的四元数旋转这个向量。
    3. 旋转后的向量的 Z 分量 (v_z) 就是其在垂直方向的投影。
    4. Pitch = arcsin(v_z)。

    优点:
    - 无论传感器怎么绕 Z 轴 (Yaw) 旋转，只要它与地面的夹角不变，Pitch 就不变。
    - 没有万向节死锁 (Gimbal Lock)。
    - 没有 ±180 度跳变问题。
    """
    # 1. 定义初始指向向量 (假设传感器 X 轴朝前)
    #    如果你觉得传感器侧着拿舒服，可以改为 [0, 1, 0]
    ref_vector = np.array([1.0, 0.0, 0.0])

    # 2. 应用旋转
    #    scipy 的 apply 方法可以直接旋转向量
    rotated_vector = orientation.apply(ref_vector)

    # 3. 提取 Z 分量 (垂直分量)
    v_z = rotated_vector[2]

    # 4. 限制范围以防数值误差导致 arcsin 报错
    v_z = np.clip(v_z, -1.0, 1.0)

    # 5. 计算俯仰角 (弧度)
    pitch = np.arcsin(v_z)

    return pitch


def main():
    parser = argparse.ArgumentParser(description="单轴 (Lift) 机械臂控制")
    parser.add_argument("--rate", "-r", type=int, default=50, help="采样率 Hz")
    parser.add_argument("--scale", type=float, default=1.5, help="旋转灵敏度")
    parser.add_argument("--no-viewer", action="store_true", help="无图形界面模式")
    parser.add_argument("--enable-drift-compensation", action="store_true", help="启用自适应漂移补偿")
    args = parser.parse_args()

    # 加载模型
    possible_paths = [
        os.path.join("assets", "universal_robots_ur3e", "ur3e_vertical.xml"),
        os.path.join("assets", "universal_robots_ur3e", "ur3e.xml"),
    ]
    model_path = None
    for path in possible_paths:
        if os.path.exists(path):
            model_path = path
            break

    if not model_path:
        print(f"❌ 找不到模型文件!")
        return

    print(f"正在加载模型: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print("\n" + "="*60)
    print("【模式：单轴 Lift 控制】")
    print("  - Pan (水平): 锁定")
    print("  - Lift (俯仰): 跟随手臂 (向量算法，无死区)")
    print("="*60)
    print("【重要提示】启动时请保持传感器静止以进行 FOC 校准！")
    print("="*60)
    input("按 Enter 键继续...")

    # 连接传感器
    receiver = BHy2CLIReceiver(
        sensor_id=37,
        sample_rate=args.rate,
        enable_drift_compensation=args.enable_drift_compensation
    )

    if not receiver.connect(perform_gyro_foc=True):
        print("❌ 连接失败!")
        return

    print("✅ 传感器已连接!")
    print("\n请将手臂平举 (或保持在你舒服的零点位置)")
    input("按 Enter 进行零点校准...")
    receiver.calibrate()
    print("🏁 开始控制... (Pan 轴已锁定)")

    # 初始位置
    initial_qpos = np.array([0, -1.57, -1.57, -1.57, -1.57, 0])
    data.qpos[:] = initial_qpos
    mujoco.mj_step(model, data)

    last_print_time = 0
    PRINT_INTERVAL = 0.5  # 打印间隔 (秒)

    # 定义控制循环逻辑
    def control_loop_step():
        nonlocal last_print_time

        # 1. 获取姿态
        orientation = receiver.get_orientation()

        if orientation:
            # 2. 计算 Pitch (使用向量投影法)
            pitch = get_pitch_from_vector(orientation)

            # 3. 映射到机械臂
            # Pan 轴锁定为 0
            target_pan = 0.0

            # Lift 轴: 初始 -1.57 (-90度, 上竖) + Pitch
            # 此时如果手臂平举 (Pitch=0)，机械臂也是竖直的 (-90)
            # 如果你希望 Pitch=0 时机械臂水平，你需要调整这里的 offset
            # 假设:
            # - 用户平举 (Pitch=0) -> 机械臂水平 (Lift = -1.57 + 1.57 = 0) ?
            # - 现在的代码是: -1.57 + pitch。
            #   如果 pitch=0, lift=-1.57 (竖直向上/向后, 取决于安装)
            #   ur3e 默认: -1.57 是竖直向上。
            target_lift = -1.57 + pitch * args.scale

            # 4. 应用控制
            data.qpos[0] = target_pan
            # 限制范围 [-180, 0] 防止打到自己
            data.qpos[1] = np.clip(target_lift, -3.14, 0)

            # 5. 调试输出 (限流)
            if time.time() - last_print_time > PRINT_INTERVAL:
                last_print_time = time.time()
                print(f"Pitch: {np.degrees(pitch):6.1f}°  ->  Lift: {np.degrees(target_lift):6.1f}° (Pan Locked)")

        mujoco.mj_step(model, data)


    # 运行循环
    try:
        if args.no_viewer:
            print("Running in No-Viewer mode...")
            while True:
                control_loop_step()
                time.sleep(0.01)
        else:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                viewer.cam.lookat[:] = [0, 0, 0.7]
                viewer.cam.distance = 1.5
                viewer.cam.azimuth = 135

                while viewer.is_running():
                    control_loop_step()
                    viewer.sync()
                    time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\n❌ 运行时错误: {e}")
    finally:
        receiver.disconnect()


if __name__ == "__main__":
    main()
