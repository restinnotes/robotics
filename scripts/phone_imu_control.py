"""
手机 IMU 实时控制 (单手机简易版)
========================================
使用 "Sensor Logger" App (iOS/Android) 控制 MuJoCo 仿真机械臂
模式: 单手机绑大臂 + 肘部固定 (1 Phone + Fixed Elbow)

使用说明:
1. 手机安装 "Sensor Logger" App
2. 在 App 设置中:
   - 开启 UDP Stream
   - Target IP: 你的电脑局域网 IP (脚本运行后会显示推荐 IP)
   - Port: 5555
   - Log Format: JSON
3. 开启传感器: "Orientation" (或者 Rotation Vector)
4. 运行此脚本: python scripts/phone_imu_control.py
5. 校准:
   - 将手机和机械臂摆出相同的初始姿势 (比如垂在身体一侧)
   - 按电脑上的 Enter 键
6. 开始挥动手臂!
"""

import socket
import json
import time
import threading
import numpy as np
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
import os

# 获取本机 IP
def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"

class SensorLoggerReceiver:
    def __init__(self, port=5555):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.settimeout(0.1)
        self.running = True

        # 状态
        self.q_curr = np.array([0, 0, 0, 1], dtype=np.float32)  # [x,y,z,w]
        self.calib_inv = R.identity()  # 校准逆矩阵

        # 线程
        self.thread = threading.Thread(target=self._loop)
        self.thread.daemon = True

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        self.sock.close()

    def calibrate(self):
        """将当前手机姿态设为 '零位'"""
        self.calib_inv = R.from_quat(self.q_curr).inv()
        print(">>> 校准完成!")

    def get_rotation(self):
        """获取相对于校准位置的旋转 (Global Frame)"""
        # R_rel = R_meas * R_calib_inv
        # 但我们希望在 Global 坐标系下校准，所以通常是 R_calib_inv * R_meas
        # 这里简化处理：我们只关心相对变化
        r_curr = R.from_quat(self.q_curr)

        # 计算相对于初始时刻的旋转差
        r_rel = self.calib_inv * r_curr
        return r_rel

    def _loop(self):
        print(f"UDP 监听中... (Port {self.sock.getsockname()[1]})")
        while self.running:
            try:
                data, _ = self.sock.recvfrom(4096)
                msg = json.loads(data.decode('utf-8'))

                # Sensor Logger 的 JSON 格式:
                # {"messageId":..., "payload": [{"name":"orientation", "values": {"qx":..., "qy":..., "qz":..., "qw":...}}]}
                if "payload" in msg:
                    for p in msg["payload"]:
                        if p["name"] == "orientation":
                            v = p["values"]
                            # 注意: Scipy 是 [x, y, z, w]
                            self.q_curr = np.array([v["qx"], v["qy"], v["qz"], v["qw"]], dtype=np.float32)

            except socket.timeout:
                continue
            except Exception as e:
                pass # 忽略错误帧

def main():
    # 路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(current_dir)
    model_path = os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")

    # 1. 加载 MuJoCo
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 2. 启动接收器
    receiver = SensorLoggerReceiver(port=5555)
    receiver.start()

    print("="*60)
    print(f"请在 Sensor Logger App 中设置 Target IP 为: {get_local_ip()}")
    print("="*60)

    print("\n请保持手机静止在初始位置，按 Enter 进行校准...")
    input()
    receiver.calibrate()

    # 3. 仿真循环
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 相机设置
        viewer.cam.lookat[:] = [0, 0, 0.7]
        viewer.cam.distance = 1.2
        viewer.cam.azimuth = 90

        while viewer.is_running():
            # 获取手机相对旋转
            r_rel = receiver.get_rotation()
            euler = r_rel.as_euler('xyz', degrees=False) # [roll, pitch, yaw]

            # --- 简易映射逻辑 (1 Phone + Fixed Elbow) ---

            # UR3e (竖直安装):
            # Joint 0 (Shoulder Pan): 控制左右旋转 -> 映射到手机 Yaw (euler[2])
            # Joint 1 (Shoulder Lift): 控制大臂抬起 -> 映射到手机 Pitch/Roll (视握持方式而定)

            # 假设手机竖着拿 (屏幕对着人)
            # Yaw (Z) -> Pan
            # Pitch (X) -> Lift

            # 增益系数 (调整灵敏度)
            k_pan = 1.0
            k_lift = 1.0

            # 设置目标角度
            target_pan = euler[2] * k_pan
            target_lift = euler[0] * k_lift # 尝试用 Roll 或 Pitch，具体看手机握法

            # 安全限位
            target_lift = np.clip(target_lift, -3.14, 0.5)

            # 写入控制
            # Home Pose: [0, -1.57, -1.57, -1.57, -1.57, 0] (竖直状态可能不同)
            # ur3e_vertical.xml 的默认姿态是竖直的

            # 简单控制:
            data.qpos[0] = target_pan
            data.qpos[1] = target_lift

            # 固定肘部 (90度弯曲，比较自然)
            data.qpos[2] = -1.57
            data.qpos[3] = -1.57 # 手腕
            data.qpos[4] = -1.57

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)

    receiver.stop()

if __name__ == "__main__":
    main()
