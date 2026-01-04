"""
BHy2CLI 传感器实时控制
========================
使用 Bosch BHI360/BHI260 传感器控制 MuJoCo 仿真机械臂。
模式: 单传感器绑大臂 + 肘部固定/自动控制

使用说明:
1. 确保 BHy2CLI 目录中有 bhy2cli.exe 且传感器已连接 (USB)
2. 运行: python scripts/sensor_imu_control.py
3. 校准:
   - 将传感器绑在手臂上，手臂自然下垂 (或保持预设零位)
   - 按 Enter 键进行校准
4. 开始挥动手臂!
"""

import time
import numpy as np
import mujoco
import mujoco.viewer
import sys
import os
import argparse

# 添加项目根目录到 path 以便导入 utils
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

from utils.bhy2cli_receiver import BHy2CLIReceiver


def main():
    parser = argparse.ArgumentParser(description="BHy2CLI 传感器机械臂控制")
    parser.add_argument("--sensor-id", "-s", type=int, default=37,
                        help="Sensor ID (默认: 37 = Game Rotation Vector, 推荐)")
    parser.add_argument("--rate", "-r", type=int, default=50,
                        help="采样率 Hz (默认: 50)")
    parser.add_argument("--scale-pan", type=float, default=1.5, help="Pan 轴灵敏度")
    parser.add_argument("--scale-lift", type=float, default=1.5, help="Lift 轴灵敏度")
    args = parser.parse_args()

    # 1. 路径设置
    model_path = os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
    if not os.path.exists(model_path):
        print(f"找不到模型文件: {model_path}")
        return

    # 2. 加载 MuJoCo
    print(f"正在加载模型: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 3. 启动接收器
    print(f"正在连接传感器 (ID: {args.sensor_id})...")
    receiver = BHy2CLIReceiver(sensor_id=args.sensor_id, sample_rate=args.rate)

    if not receiver.connect():
        print("传感器连接失败，请检查 USB 连接和 bhy2cli.exe")
        return

    print("="*60)
    print("传感器已连接!")
    print("请保持传感器和机械臂处于相同的初始姿态 (推荐: 自然下垂)")
    print("按 Enter 键进行校准...")
    print("="*60)

    # 简单的等待循环，显示当前数值方便调试
    try:
        start_wait = time.time()
        while True:
            # 只有当用户输入 Enter 时才退出循环 (但 input() 是阻塞的)
            # 这里我们使用 input() 阻塞，但在此之前先让接收器跑一会儿稳定数据
            if time.time() - start_wait > 1.0:
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        receiver.disconnect()
        return

    input() # 等待用户按回车
    receiver.calibrate()
    print("校准完成! 开始控制...")

    # 4. 仿真循环
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 相机设置
        viewer.cam.lookat[:] = [0, 0, 0.7]
        viewer.cam.distance = 1.2
        viewer.cam.azimuth = 90

        # 初始位置
        # Home Pose: [0, -1.57, -1.57, -1.57, -1.57, 0]
        data.qpos[0] = 0
        data.qpos[1] = -1.57
        data.qpos[2] = -1.57
        data.qpos[3] = -1.57
        data.qpos[4] = -1.57
        data.qpos[5] = 0
        mujoco.mj_step(model, data)

        try:
            while viewer.is_running():
                # 获取欧拉角 (度)
                euler = receiver.get_euler_degrees()

                if euler:
                    # euler: [roll, pitch, yaw]
                    #
                    # 注意: Game Rotation Vector (ID 37) 的 Yaw 会漂移 (陀螺积分误差)
                    # 所以我们只使用 Roll 和 Pitch 来控制
                    #
                    # 假设传感器佩戴方式 (传感器扁平面贴在手臂外侧):
                    # - Roll (绕前臂延长线旋转) -> Base Pan (左右旋转)
                    # - Pitch (抬臂/放下) -> Shoulder Lift (抬起/放下)

                    # 转换为弧度
                    roll_rad = np.deg2rad(euler[0])
                    pitch_rad = np.deg2rad(euler[1])
                    # yaw_rad = np.deg2rad(euler[2])  # 不使用 Yaw，避免漂移

                    # 控制映射
                    # Roll -> Pan (左右旋转底座)
                    # Pitch -> Lift (抬起/放下大臂)
                    target_pan = roll_rad * args.scale_pan
                    target_lift = -1.57 + (pitch_rad * args.scale_lift)

                    # 应用控制
                    data.qpos[0] = target_pan
                    data.qpos[1] = np.clip(target_lift, -3.14, 0) # 限位

                    # 固定其他关节
                    data.qpos[2] = -1.57 # Elbow
                    data.qpos[3] = -1.57 # Wrist 1
                    data.qpos[4] = -1.57 # Wrist 2

                    # 显示调试信息 (每 0.5 秒)
                    if int(time.time() * 2) % 2 == 0 and not hasattr(main, '_last_print') or time.time() - getattr(main, '_last_print', 0) > 0.5:
                        main._last_print = time.time()
                        print(f"R:{euler[0]:6.1f} P:{euler[1]:6.1f} Y:{euler[2]:6.1f} -> Pan:{np.rad2deg(target_pan):5.1f}° Lift:{np.rad2deg(target_lift + 1.57):5.1f}°")


                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(0.01) # 100Hz 循环

        except KeyboardInterrupt:
            pass
        finally:
            receiver.disconnect()

if __name__ == "__main__":
    main()
