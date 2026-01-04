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

from utils.bhy2cli_receiver import BHy2CLIReceiver


def main():
    parser = argparse.ArgumentParser(description="传感器融合机械臂控制 (ID 37)")
    parser.add_argument("--rate", "-r", type=int, default=50, help="采样率 Hz")
    parser.add_argument("--scale", type=float, default=1.5, help="旋转灵敏度")
    args = parser.parse_args()

    # 加载模型
    model_path = os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
    if not os.path.exists(model_path):
        print(f"找不到模型文件: {model_path}")
        return

    print(f"正在加载模型: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 启动传感器接收器 (ID 37)
    print(f"正在连接传感器 (ID 37, Rate {args.rate})...")
    # 注意：这里使用 BHy2CLIReceiver 而不是 BHy2CLIAccelReceiver
    receiver = BHy2CLIReceiver(sensor_id=37, sample_rate=args.rate)

    if not receiver.connect():
        print("连接失败!")
        return

    print("="*60)
    print("传感器已连接 (Game Rotation Vector)!")
    print("包含 Yaw (水平旋转) 控制")
    print("如果发现漂移，请尝试保持静止")
    print("按 Enter 键进行零点校准...")
    print("="*60)

    input()
    receiver.calibrate()
    print("开始控制...")

    # 仿真
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.7]
        viewer.cam.distance = 1.2
        viewer.cam.azimuth = 90

        # 初始位置
        initial_qpos = np.array([0, -1.57, -1.57, -1.57, -1.57, 0])
        data.qpos[:] = initial_qpos
        mujoco.mj_step(model, data)

        last_print_time = 0

        try:
            while viewer.is_running():
                # 获取相对旋转 (Rotation 对象)
                orientation = receiver.get_orientation()

                if orientation:
                    # 将四元数转换为欧拉角 (zyx顺序: yaw, pitch, roll)
                    euler = orientation.as_euler('zyx', degrees=False)
                    yaw, pitch, roll = euler

                    # 限制角度范围，防止过度旋转
                    # 映射:
                    # Yaw (z)   -> Base Pan (qpos[0])
                    # Pitch (y) -> Shoulder Lift (qpos[1])
                    # Roll (x)  -> Elbow (qpos[2]) 或者 Wrist

                    target_pan = yaw * args.scale
                    target_lift = -1.57 + pitch * args.scale
                    target_elbow = -1.57 + roll * args.scale

                    # 应用控制
                    data.qpos[0] = target_pan
                    data.qpos[1] = np.clip(target_lift, -3.14, 0)
                    # data.qpos[2] = np.clip(target_elbow, -3.14, 0) # 可以选择是否启用

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
        finally:
            receiver.disconnect()


if __name__ == "__main__":
    main()
