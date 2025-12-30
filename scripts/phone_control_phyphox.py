"""
手机控制 (phyphox 版) - 无需 UDP 配置
========================================
使用 "phyphox" App 实时控制 MuJoCo 仿真机械臂
模式: 单手机控制 (1 Phone Control)

使用说明:
1. 手机安装 "phyphox" App (免费开源)
2. 打开 App -> 找到 "Attitude" (姿态)
3. 点击进入 -> 点右上角三个点 -> 选 "Allow remote access"
4. 手机会显示一个网址 (例如 http://192.168.1.50:8080)
5. 运行此脚本，并在提示时输入该网址:
   python scripts/phone_control_phyphox.py --url http://192.168.1.50:8080
"""

import requests
import time
import argparse
import numpy as np
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
import os

class PhyphoxReceiver:
    def __init__(self, base_url):
        self.base_url = base_url.rstrip('/')
        # 兼容 'Attitude' 或 'Inclination' 实验
        self.api_url = f"{self.base_url}/get?"
        self.calib_inv = R.identity()

    def get_data(self):
        """通过网页 API 获取当前姿态数据 (强制不使用代理)"""
        try:
            # 关键：proxies={'http': None, 'https': None} 绕过 VPN/代理
            # 使用正确的 buffer 名：angle 和 anglePlane (斜面实验)
            response = requests.get(
                f"{self.api_url}angle&anglePlane",
                timeout=0.3,
                proxies={'http': None, 'https': None}
            )

            if response.status_code != 200:
                return None

            data = response.json()['buffer']

            # 使用 Inclination 实验的 angle (前后倾斜) 和 anglePlane (左右倾斜)
            if 'angle' in data and len(data['angle']['buffer']) > 0:
                angle = data['angle']['buffer'][-1]      # 前后倾斜 (度)
                anglePlane = 0
                if 'anglePlane' in data and len(data['anglePlane']['buffer']) > 0:
                    anglePlane = data['anglePlane']['buffer'][-1]
                # 映射到欧拉角格式
                return R.from_euler('xyz', [np.radians(angle), 0, np.radians(-anglePlane)], degrees=False)

            return None
        except requests.exceptions.ProxyError:
            print("连接错误: 检测到代理冲突，请尝试关闭 VPN")
            return None
        except requests.exceptions.ConnectionError:
            print("连接错误: 无法连接到手机，请检查 IP 和远程访问是否开启")
            return None
        except Exception:
            return None

    def calibrate(self, r_curr):
        self.calib_inv = r_curr.inv()
        print(">>> 校准完成!")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", type=str, required=True, help="phyphox 显示的网址 (如 http://192.168.1.xx:8080)")
    args = parser.parse_args()

    # 路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(current_dir)
    model_path = os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")

    # 仿真模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    receiver = PhyphoxReceiver(args.url)

    print("\n[第一步] 正在连接手机...")
    while True:
        r_init = receiver.get_data()
        if r_init:
            print("连接成功!")
            break
        print("等待连接... 请检查手机是否开启了 Remote Access")
        time.sleep(1.0)

    print("\n[第二步] 校准: 请将手机屏幕正对自己竖着拿稳，然后按 Enter...")
    input()
    r_curr = receiver.get_data()
    receiver.calibrate(r_curr)

    print("\n[第三步] 开始控制! 按 ESC 退出窗口")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0, 0, 0.7]
        viewer.cam.distance = 1.2

        while viewer.is_running():
            r_curr = receiver.get_data()
            if r_curr:
                # 计算相对旋转
                r_rel = receiver.calib_inv * r_curr
                euler = r_rel.as_euler('xyz', degrees=False)

                # 映射到 UR3e (单手机控制前 2 个关节)
                data.qpos[0] = euler[2] # Yaw -> Pan
                data.qpos[1] = euler[0] # Roll -> Lift

                # 固定动作，让它看起来像在控制
                data.qpos[2] = -1.57 # 肘部
                data.qpos[3] = -1.57 # 手腕

                mujoco.mj_step(model, data)
                viewer.sync()

            time.sleep(0.02) # 50Hz 采样

if __name__ == "__main__":
    main()
