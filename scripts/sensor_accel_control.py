"""
BHy2CLI 加速度计控制脚本
========================
使用加速度计直接计算 Roll/Pitch 控制 MuJoCo 机械臂。
完全避免陀螺仪漂移问题！

使用说明:
1. 确保传感器已连接
2. 运行: python scripts/sensor_accel_control.py
3. 校准时保持静止
4. 开始控制!

限制:
- 只有 Roll (左右倾斜) 和 Pitch (前后倾斜)
- 没有 Yaw (旋转方向)
"""

import time
import numpy as np
import mujoco
import mujoco.viewer
import sys
import os
import argparse

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

from utils.bhy2cli_accel_receiver import BHy2CLIAccelReceiver


def main():
    parser = argparse.ArgumentParser(description="加速度计机械臂控制")
    parser.add_argument("--rate", "-r", type=int, default=50, help="采样率 Hz")
    parser.add_argument("--scale-roll", type=float, default=2.0, help="Roll 灵敏度")
    parser.add_argument("--scale-pitch", type=float, default=2.0, help="Pitch 灵敏度")
    args = parser.parse_args()

    # 加载模型
    model_path = os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
    if not os.path.exists(model_path):
        print(f"找不到模型文件: {model_path}")
        return

    print(f"正在加载模型: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 启动加速度计接收器
    print("正在连接加速度计...")
    receiver = BHy2CLIAccelReceiver(sample_rate=args.rate)

    if not receiver.connect():
        print("连接失败!")
        return

    print("="*60)
    print("加速度计已连接!")
    print("优点: 永远不会漂移")
    print("限制: 只能获取 Roll 和 Pitch，没有 Yaw")
    print("按 Enter 键进行校准...")
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
        data.qpos[0] = 0
        data.qpos[1] = -1.57
        data.qpos[2] = -1.57
        data.qpos[3] = -1.57
        data.qpos[4] = -1.57
        data.qpos[5] = 0
        mujoco.mj_step(model, data)

        last_print_time = 0

        try:
            while viewer.is_running():
                tilt = receiver.get_tilt_degrees()

                if tilt:
                    roll, pitch = tilt

                    # 控制映射
                    # Roll -> 底座旋转 (Pan)
                    # Pitch -> 肩部升降 (Lift)
                    target_pan = np.radians(roll * args.scale_roll)
                    target_lift = -1.57 + np.radians(pitch * args.scale_pitch)

                    data.qpos[0] = target_pan
                    data.qpos[1] = np.clip(target_lift, -3.14, 0)

                    # 固定其他关节
                    data.qpos[2] = -1.57
                    data.qpos[3] = -1.57
                    data.qpos[4] = -1.57

                    # 调试输出
                    if time.time() - last_print_time > 0.5:
                        last_print_time = time.time()
                        print(f"Roll:{roll:6.1f}° Pitch:{pitch:6.1f}° -> Pan:{np.degrees(target_pan):5.1f}° Lift:{np.degrees(target_lift+1.57):5.1f}°")

                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.01)

        except KeyboardInterrupt:
            pass
        finally:
            receiver.disconnect()


if __name__ == "__main__":
    main()
