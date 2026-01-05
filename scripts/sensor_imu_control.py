"""
BHy2CLI 传感器融合控制脚本 (ID 37)
================================
使用 Game Rotation Vector (ID 37) 控制 MuJoCo 机械臂。
包含 Yaw (水平旋转) 控制。

注意：由于固件问题，此模式可能会有漂移。
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

# 切换到项目根目录，确保相对路径可以工作（与可工作的 check_imu 保持一致）
os.chdir(project_root)

from utils.bhy2cli_receiver import BHy2CLIReceiver


def main():
    parser = argparse.ArgumentParser(description="传感器融合机械臂控制 (ID 37)")
    parser.add_argument("--rate", "-r", type=int, default=50, help="采样率 Hz")
    parser.add_argument("--scale", type=float, default=1.5, help="旋转灵敏度")
    parser.add_argument("--no-viewer", action="store_true", help="无图形界面模式（适用于无显示环境）")
    args = parser.parse_args()

    # 加载模型（使用相对路径，与可工作的 check_imu 保持一致）
    # 尝试多个可能的模型文件
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
        print(f"找不到模型文件，尝试了以下路径:")
        for path in possible_paths:
            print(f"  - {path}")
        return

    print(f"正在加载模型: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 启动传感器接收器 (ID 37)
    print(f"正在连接传感器 (ID 37, Rate {args.rate})...")
    # 注意：这里使用 BHy2CLIReceiver 而不是 BHy2CLIAccelReceiver
    receiver = BHy2CLIReceiver(sensor_id=37, sample_rate=args.rate)

    if not receiver.connect(perform_gyro_foc=True):
        print("连接失败!")
        return

    print("="*60)
    print("传感器已连接 (Game Rotation Vector)!")
    print("包含 Yaw (水平旋转) 控制")
    print("="*60)
    print("注意: 陀螺仪校准 (foc 3) 已在启动时自动执行")
    print("="*60)

    # 零点校准（设置初始姿态为零点）
    print("\n请将传感器保持在初始位置（你希望作为零点的位置）")
    print("按 Enter 进行零点校准...")
    input()
    receiver.calibrate()
    print("开始控制...")

    # 初始位置
    initial_qpos = np.array([0, -1.57, -1.57, -1.57, -1.57, 0])
    data.qpos[:] = initial_qpos
    mujoco.mj_step(model, data)

    last_print_time = 0

    if args.no_viewer:
        # 无 viewer 模式
        print("="*60)
        print("无图形界面模式运行中...")
        print("按 Ctrl+C 退出")
        print("="*60)

        try:
            while True:
                # 获取相对旋转 (Rotation 对象)
                orientation = receiver.get_orientation()

                if orientation:
                    # 将四元数转换为欧拉角 (zyx顺序: yaw, pitch, roll)
                    euler = orientation.as_euler('zyx', degrees=False)
                    yaw, pitch, roll = euler

                    # 限制角度范围，防止过度旋转
                    target_pan = yaw * args.scale
                    target_lift = -1.57 + pitch * args.scale
                    target_elbow = -1.57 + roll * args.scale

                    # 应用控制
                    data.qpos[0] = target_pan
                    data.qpos[1] = np.clip(target_lift, -3.14, 0)

                    # 调试输出
                    if time.time() - last_print_time > 0.5:
                        last_print_time = time.time()
                        y_deg, p_deg, r_deg = np.degrees([yaw, pitch, roll])
                        print(f"Y:{y_deg:6.1f}° P:{p_deg:6.1f}° R:{r_deg:6.1f}° -> Pan:{np.degrees(target_pan):5.1f}° Lift:{np.degrees(target_lift+1.57):5.1f}°")

                mujoco.mj_step(model, data)
                time.sleep(0.01)

        except KeyboardInterrupt:
            pass
        finally:
            receiver.disconnect()
    else:
        # 有 viewer 模式（使用与可工作的 check_imu 相同的代码结构）
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                viewer.cam.lookat[:] = [0, 0, 0.7]
                viewer.cam.distance = 1.2
                viewer.cam.azimuth = 90

                try:
                    while viewer.is_running():
                        # 获取相对旋转 (Rotation 对象)
                        orientation = receiver.get_orientation()

                        if orientation:
                            # 将四元数转换为欧拉角 (zyx顺序: yaw, pitch, roll)
                            euler = orientation.as_euler('zyx', degrees=False)
                            yaw, pitch, roll = euler

                            # 限制角度范围，防止过度旋转
                            target_pan = yaw * args.scale
                            target_lift = -1.57 + pitch * args.scale
                            target_elbow = -1.57 + roll * args.scale

                            # 应用控制
                            data.qpos[0] = target_pan
                            data.qpos[1] = np.clip(target_lift, -3.14, 0)

                            # 调试输出
                            if time.time() - last_print_time > 0.5:
                                last_print_time = time.time()
                                y_deg, p_deg, r_deg = np.degrees([yaw, pitch, roll])
                                print(f"Y:{y_deg:6.1f}° P:{p_deg:6.1f}° R:{r_deg:6.1f}° -> Pan:{np.degrees(target_pan):5.1f}° Lift:{np.degrees(target_lift+1.57):5.1f}°")

                        mujoco.mj_step(model, data)
                        viewer.sync()
                        time.sleep(0.01)

                except KeyboardInterrupt:
                    pass
        except Exception as e:
            print(f"\n❌ 无法创建图形窗口: {e}")
            print("\n💡 解决方案:")
            print("1. 使用 --no-viewer 参数运行（无图形界面）:")
            print("   python3 scripts/sensor_imu_control.py --no-viewer")
            print("\n2. 或者在有图形界面的环境中运行")
            print("\n3. 或者使用 SSH X11 转发:")
            print("   ssh -X user@host")
            print("   python3 scripts/sensor_imu_control.py")
        finally:
            receiver.disconnect()


if __name__ == "__main__":
    main()
