#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BHy2CLI 硬件直接控制机械臂
==========================
使用 BHy2CLI 硬件读取的四元数数据直接控制 MuJoCo 仿真机械臂
完全替换手机版本，使用相同的控制逻辑
"""

import subprocess
import re
import time
import sys
import os
import threading
import numpy as np
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R


class BHy2CLIReceiver:
    """BHy2CLI 硬件数据接收器"""
    
    def __init__(self, bhy2cli_path="./BHy2CLI/i2c_bhy2cli", firmware_path=None):
        self.bhy2cli_path = bhy2cli_path
        self.firmware_path = firmware_path
        self.process = None
        
        # 数据格式: [D]SID: 37; T: 626.982968750; x: 0.200867, y: -0.598511, z: -0.775391, w: 0.010498; acc: 0.000000
        self.pattern = re.compile(
            r'\[D\]SID:\s*(\d+);\s*T:\s*([\d.]+);\s*x:\s*([\d.-]+),\s*y:\s*([\d.-]+),\s*z:\s*([\d.-]+),\s*w:\s*([\d.-]+);\s*acc:\s*([\d.-]+)'
        )
        
        self.q_curr = np.array([0, 0, 0, 1], dtype=np.float32)  # [x, y, z, w]
        self.calib_inv = R.identity()  # 校准逆矩阵
        self.running = True
        
        # 线程
        self.thread = threading.Thread(target=self._loop)
        self.thread.daemon = True
    
    def start(self):
        """启动 bhy2cli 进程"""
        # 如果需要，先烧录固件
        if self.firmware_path:
            print(f"📤 烧录传感器固件: {self.firmware_path}")
            flash_cmd = [self.bhy2cli_path, '-b', self.firmware_path]
            flash_result = subprocess.run(flash_cmd, capture_output=True, text=True, timeout=10)
            if flash_result.returncode != 0:
                print(f"⚠️  固件烧录可能有问题")
            else:
                print("✅ 固件烧录完成")
            time.sleep(3)
        
        # 启动读取进程
        print(f"🚀 启动 bhy2cli: {self.bhy2cli_path}")
        self.process = subprocess.Popen(
            [self.bhy2cli_path, '-c', '37:25'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            universal_newlines=True
        )
        print("✅ bhy2cli 已启动")
        
        # 启动后台读取线程
        self.thread.start()
        print("✅ 数据读取线程已启动")
    
    def parse_line(self, line):
        """解析数据行"""
        match = self.pattern.search(line)
        if match:
            x = float(match.group(3))
            y = float(match.group(4))
            z = float(match.group(5))
            w = float(match.group(6))
            return np.array([x, y, z, w], dtype=np.float32)
        return None
    
    def _loop(self):
        """后台线程持续读取数据"""
        while self.running:
            try:
                if self.process and self.process.stdout:
                    line = self.process.stdout.readline()
                    if line:
                        quat = self.parse_line(line)
                        if quat is not None:
                            self.q_curr = quat
            except:
                pass
            time.sleep(0.001)  # 避免 CPU 占用过高
    
    def calibrate(self):
        """将当前姿态设为 '零位'"""
        # 等待一个有效数据
        while np.allclose(self.q_curr, [0, 0, 0, 1]):
            time.sleep(0.01)
        
        self.calib_inv = R.from_quat(self.q_curr).inv()
        print(">>> 校准完成!")
    
    def get_rotation(self):
        """获取相对于校准位置的旋转（与 phone_imu_control.py 完全相同）"""
        r_curr = R.from_quat(self.q_curr)
        # 计算相对于初始时刻的旋转差
        r_rel = self.calib_inv * r_curr
        return r_rel
    
    def stop(self):
        """停止进程"""
        self.running = False
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except:
                self.process.kill()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='BHy2CLI 硬件直接控制机械臂')
    parser.add_argument('--bhy2cli', type=str, 
                       default=None,
                       help='bhy2cli 可执行文件路径（默认：自动查找）')
    parser.add_argument('--firmware', type=str,
                       default=None,
                       help='传感器固件路径（默认：自动查找）')
    parser.add_argument('--no-viewer', action='store_true',
                       help='不使用图形界面（用于无显示环境）')
    
    args = parser.parse_args()
    
    # 设置环境变量以支持软件渲染
    if 'LIBGL_ALWAYS_SOFTWARE' not in os.environ:
        os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    
    # 路径（与 phone_imu_control.py 完全相同）
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(current_dir)
    model_path = os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e.xml")
    
    # 自动查找 bhy2cli
    if args.bhy2cli is None:
        possible_bhy2cli = [
            os.path.join(project_root, "BHy2CLI", "i2c_bhy2cli"),
            os.path.join(project_root, "BHy2CLI", "release", "PC", "bin", "x64", "i2c_bhy2cli"),
            "./i2c_bhy2cli",
        ]
        for path in possible_bhy2cli:
            if os.path.exists(path):
                args.bhy2cli = path
                break
        if args.bhy2cli is None:
            print("❌ 未找到 i2c_bhy2cli，请使用 --bhy2cli 指定路径")
            return 1
    
    # 自动查找固件
    if args.firmware is None:
        possible_firmware = [
            os.path.join(project_root, "BHy2CLI", "submodules", "bhi360", "firmware", "bhi360", "Bosch_Shuttle3_BHI360.fw"),
        ]
        for path in possible_firmware:
            if os.path.exists(path):
                args.firmware = path
                break
    
    print(f"使用 bhy2cli: {args.bhy2cli}")
    if args.firmware:
        print(f"使用固件: {args.firmware}")
    
    # 1. 加载 MuJoCo
    print("加载 MuJoCo 模型...")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # 2. 启动 BHy2CLI 接收器
    receiver = BHy2CLIReceiver(args.bhy2cli, args.firmware)
    receiver.start()
    
    print("\n等待数据...")
    time.sleep(2)
    
    # 等待第一个有效数据
    while np.allclose(receiver.q_curr, [0, 0, 0, 1]):
        time.sleep(0.1)
    
    print("✅ 数据接收正常")
    
    print("\n请保持设备在初始位置，按 Enter 进行校准...")
    input()
    receiver.calibrate()
    
    # 3. 仿真循环（完全仿照 phone_imu_control.py 的逻辑）
    if args.no_viewer:
        # 无 viewer 模式：只运行仿真逻辑
        print("\n⚠️  无 viewer 模式：运行仿真但不显示窗口")
        print("按 Ctrl+C 退出")
        try:
            while True:
                # 获取相对旋转（与 phone_imu_control.py 完全相同）
                r_rel = receiver.get_rotation()
                euler = r_rel.as_euler('xyz', degrees=False)  # [roll, pitch, yaw]
                
                # --- 简易映射逻辑 (与 phone_imu_control.py 完全相同) ---
                # UR3e (竖直安装):
                # Joint 0 (Shoulder Pan): 控制左右旋转 -> 映射到 Yaw (euler[2])
                # Joint 1 (Shoulder Lift): 控制大臂抬起 -> 映射到 Roll (euler[0])
                
                # 增益系数 (调整灵敏度)
                k_pan = 1.0
                k_lift = 1.0
                
                # 设置目标角度
                target_pan = euler[2] * k_pan
                target_lift = euler[0] * k_lift
                
                # 安全限位
                target_lift = np.clip(target_lift, -3.14, 0.5)
                
                # 写入控制（与 phone_imu_control.py 完全相同）
                data.qpos[0] = target_pan
                data.qpos[1] = target_lift
                
                # 固定肘部 (90度弯曲，比较自然)
                data.qpos[2] = -1.57
                data.qpos[3] = -1.57  # 手腕
                data.qpos[4] = -1.57
                
                mujoco.mj_step(model, data)
                time.sleep(0.01)  # 与 phone_imu_control.py 相同
        except KeyboardInterrupt:
            print("\n收到退出信号")
    else:
        # 尝试创建 viewer
        try:
            with mujoco.viewer.launch_passive(model, data) as viewer:
                # 相机设置（与 phone_imu_control.py 完全相同）
                viewer.cam.lookat[:] = [0, 0, 0.7]
                viewer.cam.distance = 1.2
                viewer.cam.azimuth = 90
                
                while viewer.is_running():
                    # 获取相对旋转（与 phone_imu_control.py 完全相同）
                    r_rel = receiver.get_rotation()
                    euler = r_rel.as_euler('xyz', degrees=False)  # [roll, pitch, yaw]
                    
                    # --- 简易映射逻辑 (与 phone_imu_control.py 完全相同) ---
                    # UR3e (竖直安装):
                    # Joint 0 (Shoulder Pan): 控制左右旋转 -> 映射到 Yaw (euler[2])
                    # Joint 1 (Shoulder Lift): 控制大臂抬起 -> 映射到 Roll (euler[0])
                    
                    # 增益系数 (调整灵敏度)
                    k_pan = 1.0
                    k_lift = 1.0
                    
                    # 设置目标角度
                    target_pan = euler[2] * k_pan
                    target_lift = euler[0] * k_lift
                    
                    # 安全限位
                    target_lift = np.clip(target_lift, -3.14, 0.5)
                    
                    # 写入控制（与 phone_imu_control.py 完全相同）
                    data.qpos[0] = target_pan
                    data.qpos[1] = target_lift
                    
                    # 固定肘部 (90度弯曲，比较自然)
                    data.qpos[2] = -1.57
                    data.qpos[3] = -1.57  # 手腕
                    data.qpos[4] = -1.57
                    
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    time.sleep(0.01)  # 与 phone_imu_control.py 相同
        except Exception as e:
            print(f"\n❌ 无法创建图形窗口: {e}")
            print("\n💡 解决方案:")
            print("1. 使用 --no-viewer 参数运行（无图形界面）:")
            print("   python3 scripts/bhy2cli_robot_control.py --no-viewer")
            print("\n2. 或者安装 xvfb 并使用虚拟显示:")
            print("   sudo apt install xvfb")
            print("   xvfb-run -a python3 scripts/bhy2cli_robot_control.py")
            print("\n3. 或者检查显示服务器配置")
            return 1
    
    # 清理
    receiver.stop()
    print("\n退出")


if __name__ == "__main__":
    main()
