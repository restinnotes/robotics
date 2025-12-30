"""
IMU 驱动仿真验证脚本 - Bridge Phase 1 & 2
目标：可视化验证 imu_solver.py 的驱动能力
逻辑：
1. 读取 reference trajectory
2. 转换为 虚拟 IMU 四元数
3. 喂给 IMU Solver -> 得到 target angles
4. 驱动 MuJoCo 仿真 (Visual)
"""

import os
import sys
# Add project root to sys.path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import time
import mujoco
import mujoco.viewer
import numpy as np
from utils.imu_solver import IMUSolver
from scipy.spatial.transform import Rotation as R

def main():
    # 1. 路径设置
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(current_dir))
    model_path = os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
    traj_path = os.path.join(project_root, "data", "walk_arm_direct.npz")

    # 2. 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 3. 加载轨迹
    print(f"Loading trajectory: {traj_path}")
    traj = np.load(traj_path, allow_pickle=True)
    ref_qpos = traj["qpos"]  # (N, 6)
    n_frames = ref_qpos.shape[0]

    # 4. 获取 Site ID
    site_upper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "upper_arm_imu_site")
    site_fore_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "forearm_imu_site")

    if site_upper_id == -1 or site_fore_id == -1:
        print("Error: IMU sites not found in XML!")
        return

    # 5. 初始化 Solver
    solver = IMUSolver()

    # 6. 启动 Viewer
    print("Starting simulation... Press ESC to exit.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 调整相机视角 (与 trajectory_player 一致)
        viewer.cam.lookat[:] = [0, 0, 0.7]
        viewer.cam.distance = 1.2
        viewer.cam.elevation = 0
        viewer.cam.azimuth = 90

        # 循环播放
        step = 0
        camera_angles = [
            {"name": "正面", "azimuth": 90, "elevation": 0, "distance": 1.2},
            {"name": "侧面", "azimuth": 0, "elevation": 0, "distance": 1.2},
            {"name": "俯视", "azimuth": 90, "elevation": -45, "distance": 1.5},
        ]
        cam_switch_interval = 250 # 5 seconds at 50Hz
        current_cam_idx = 0

        while viewer.is_running():
            # 循环索引
            frame_idx = step % n_frames

            # 切换镜头
            if step % cam_switch_interval == 0:
                idx = (step // cam_switch_interval) % 3
                cam = camera_angles[idx]
                viewer.cam.azimuth = cam["azimuth"]
                viewer.cam.elevation = cam["elevation"]
                viewer.cam.distance = cam["distance"]
                print(f"Switching to [{cam['name']}] view")

            # --- 步骤 A: 生成虚拟 IMU 数据 (Ground Truth) ---
            # 1. 把机器人摆到 GT 位置 (仅用于生成数据，不用来显示)
            ref = ref_qpos[frame_idx]
            target_gt = np.zeros(6)
            target_gt[0] = ref[1]           # Pan
            target_gt[1] = ref[0]           # Lift
            target_gt[2] = ref[3]           # Elbow
            target_gt[3] = -1.57 + ref[2]   # Wrist1
            target_gt[4] = -1.57
            target_gt[5] = 0

            # 写入状态并计算正运动学
            data.qpos[:6] = target_gt
            mujoco.mj_kinematics(model, data)

            # 读取 Site 四元数 (这就是我们要模拟的"传感器输入")
            mat_upper = data.site_xmat[site_upper_id].reshape(3, 3)
            mat_fore = data.site_xmat[site_fore_id].reshape(3, 3)
            q_upper = R.from_matrix(mat_upper).as_quat()
            q_fore = R.from_matrix(mat_fore).as_quat()

            # --- 步骤 B: 校准 (第一帧 或 每一轮开始) ---
            if step == 0:
                print("Calibrating...")
                solver.calibrate(q_upper, q_fore, offset_joints=target_gt)

            # --- 步骤 C: 解算 (Core Logic) ---
            # 现在我们假装只知道 q_upper 和 q_fore，不知道 target_gt
            solved_joints = solver.solve(q_upper, q_fore)

            # --- 步骤 D: 驱动显示 ---
            # 将解算出来的角度赋给机器人 (真正用于显示的)
            data.qpos[:6] = solved_joints

            # 前进一小步 (仅为了渲染)
            mujoco.mj_forward(model, data)
            viewer.sync()

            # 帧率控制 (50Hz)
            time.sleep(0.02)
            step += 1

if __name__ == "__main__":
    main()
