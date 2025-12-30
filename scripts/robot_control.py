"""
统一机器人控制入口
==================
支持多种数据源 (BLE/WiFi) 和多种目标 (仿真/真机)。

使用方法:
    # WiFi 控制仿真
    python scripts/robot_control.py --source wifi --url http://192.168.1.31:8080 --target sim

    # BLE 控制仿真
    python scripts/robot_control.py --source ble --address AA:BB:CC:DD:EE:FF --target sim

    # WiFi 控制真机 (需要 ur_rtde)
    python scripts/robot_control.py --source wifi --url http://192.168.1.31:8080 --target real --robot_ip 192.168.1.100
"""

import argparse
import asyncio
import time
import sys
import os

# 添加项目根目录到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

import numpy as np
import mujoco
import mujoco.viewer

from utils.imu_data_source import IMUDataSource
from utils.imu_solver import IMUSolver


def create_data_source(source_type: str, **kwargs) -> IMUDataSource:
    """
    工厂方法：根据类型创建数据源
    """
    if source_type == "wifi":
        from utils.phyphox_receiver import PhyphoxIMUReceiver
        url = kwargs.get("url", "http://192.168.1.31:8080")
        return PhyphoxIMUReceiver(url)

    elif source_type == "ble":
        from utils.ble_imu_receiver import BLEIMUReceiver
        address = kwargs.get("address")
        if not address:
            raise ValueError("BLE 模式需要指定 --address 参数")
        return BLEIMUReceiver(address)

    else:
        raise ValueError(f"未知的数据源类型: {source_type}")


def run_simulation_control(source: IMUDataSource):
    """
    运行仿真控制循环
    """
    # 加载模型
    model_path = os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print("\n[第一步] 等待数据源连接...")

    # 等待连接
    if hasattr(source, 'wait_for_connection'):
        if not source.wait_for_connection():
            print("无法连接到数据源")
            return
    else:
        # BLE 需要异步连接
        print("请确保 BLE 设备已连接")

    print("\n[第二步] 校准: 请将设备保持静止，然后按 Enter...")
    input()
    source.calibrate()

    print("\n[第三步] 开始控制! 按 ESC 退出窗口")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.7]
        viewer.cam.distance = 1.2
        viewer.cam.elevation = 0
        viewer.cam.azimuth = 90

        while viewer.is_running():
            orientation = source.get_orientation()

            if orientation is not None:
                euler = orientation.as_euler('xyz', degrees=False)

                # 映射到 UR3e 关节
                data.qpos[0] = euler[2]  # Yaw -> Pan
                data.qpos[1] = euler[0]  # Roll -> Lift
                data.qpos[2] = -1.57     # 肘部固定
                data.qpos[3] = -1.57     # 手腕固定

                mujoco.mj_step(model, data)
                viewer.sync()

            time.sleep(0.02)  # 50Hz


def run_real_robot_control(source: IMUDataSource, robot_ip: str):
    """
    运行真机控制循环 (需要 ur_rtde)
    """
    try:
        from rtde_control import RTDEControlInterface
        from rtde_receive import RTDEReceiveInterface
    except ImportError:
        print("错误: 真机控制需要安装 ur_rtde 库")
        print("请运行: pip install ur_rtde")
        return

    print(f"连接到 UR3e: {robot_ip}")
    rtde_c = RTDEControlInterface(robot_ip)
    rtde_r = RTDEReceiveInterface(robot_ip)

    print("\n[第一步] 等待数据源连接...")
    if hasattr(source, 'wait_for_connection'):
        if not source.wait_for_connection():
            print("无法连接到数据源")
            return

    print("\n[第二步] 校准: 请将设备保持静止，然后按 Enter...")
    input()
    source.calibrate()

    print("\n[第三步] 开始控制! 按 Ctrl+C 退出")
    print("警告: 请确保机械臂周围无障碍物!")

    try:
        while True:
            orientation = source.get_orientation()

            if orientation is not None:
                euler = orientation.as_euler('xyz', degrees=False)

                # 获取当前关节角度
                current_q = rtde_r.getActualQ()

                # 计算目标关节角度 (只修改前两个)
                target_q = current_q.copy()
                target_q[0] = euler[2]  # Yaw -> Pan
                target_q[1] = euler[0]  # Roll -> Lift

                # 发送速度控制 (更安全)
                velocity = 0.5
                acceleration = 0.5
                rtde_c.servoJ(target_q, velocity, acceleration, 0.02, 0.1, 300)

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n停止控制")
        rtde_c.servoStop()
        rtde_c.disconnect()


def main():
    parser = argparse.ArgumentParser(description="统一机器人控制入口")
    parser.add_argument("--source", type=str, required=True, choices=["wifi", "ble"],
                        help="数据源类型: wifi (phyphox) 或 ble (BHI3xx)")
    parser.add_argument("--url", type=str, default="http://192.168.1.31:8080",
                        help="WiFi 模式: phyphox URL")
    parser.add_argument("--address", type=str,
                        help="BLE 模式: 设备地址 (如 AA:BB:CC:DD:EE:FF)")
    parser.add_argument("--target", type=str, default="sim", choices=["sim", "real"],
                        help="控制目标: sim (仿真) 或 real (真机)")
    parser.add_argument("--robot_ip", type=str, default="192.168.1.100",
                        help="真机模式: UR3e 的 IP 地址")

    args = parser.parse_args()

    # 创建数据源
    source = create_data_source(
        args.source,
        url=args.url,
        address=args.address
    )

    # 运行控制
    if args.target == "sim":
        run_simulation_control(source)
    else:
        run_real_robot_control(source, args.robot_ip)


if __name__ == "__main__":
    main()
