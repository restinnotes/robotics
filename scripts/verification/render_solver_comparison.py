"""
渲染并排对比视频：Ground Truth (Direct) vs IMU Solver
原理：
在一个脚本中维护两个 MuJoCo 环境实例：
1. env_gt: 直接使用参考轨迹驱动
2. env_imu: 使用 IMU Solver 解算驱动

每一帧采集图像，拼接，保存为视频。
"""

import mujoco
import numpy as np
import cv2
import os
from utils.imu_solver import IMUSolver
from scipy.spatial.transform import Rotation as R

def main():
    # --- 配置 ---
    width, height = 640, 480
    output_fps = 50
    duration_frames = 500 # 录制 10 秒
    video_path = "comparison_solver.mp4"

    # 路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
    traj_path = os.path.join(current_dir, "data", "walk_arm_direct.npz")

    # --- 加载数据 ---
    traj = np.load(traj_path, allow_pickle=True)
    ref_qpos = traj["qpos"]
    n_frames = ref_qpos.shape[0]

    # --- 初始化两个环境 ---
    # Env 1: Ground Truth
    model_1 = mujoco.MjModel.from_xml_path(model_path)
    data_1 = mujoco.MjData(model_1)
    renderer_1 = mujoco.Renderer(model_1, height=height, width=width)

    # Env 2: IMU Solver
    model_2 = mujoco.MjModel.from_xml_path(model_path)
    data_2 = mujoco.MjData(model_2)
    renderer_2 = mujoco.Renderer(model_2, height=height, width=width)

    # 设置相机 (两边一致)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0, 0, 0.7]
    cam.distance = 1.2
    cam.elevation = 0
    cam.azimuth = 90

    # IMU 相关 ID
    site_upper_id = mujoco.mj_name2id(model_2, mujoco.mjtObj.mjOBJ_SITE, "upper_arm_imu_site")
    site_fore_id = mujoco.mj_name2id(model_2, mujoco.mjtObj.mjOBJ_SITE, "forearm_imu_site")

    solver = IMUSolver()

    # --- 视频写入器 ---
    # 并排宽度 = 640 * 2 = 1280
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(video_path, fourcc, output_fps, (width * 2, height))

    print(f"Recording comparison video to {video_path}...")

    for i in range(duration_frames):
        frame_idx = i % n_frames

        # --- 1. 准备 GT 数据 ---
        ref = ref_qpos[frame_idx]
        target_gt = np.zeros(6)
        target_gt[0] = ref[1]           # Pan
        target_gt[1] = ref[0]           # Lift
        target_gt[2] = ref[3]           # Elbow
        target_gt[3] = -1.57 + ref[2]   # Wrist1
        target_gt[4] = -1.57            # Wrist2
        target_gt[5] = 0                # Wrist3

        # --- Env 1 更新 (Ground Truth) ---
        data_1.qpos[:6] = target_gt
        mujoco.mj_forward(model_1, data_1)

        renderer_1.update_scene(data_1, camera=cam)
        img_1 = renderer_1.render()

        # 给图像加文字
        img_1 = cv2.cvtColor(img_1, cv2.COLOR_RGB2BGR) # MuJoCo returns RGB
        cv2.putText(img_1, "Ground Truth (Trajectory)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # --- Env 2 更新 (IMU Solver) ---
        # A. 生成虚拟 IMU (借用 Env 2 的 FK)
        data_2.qpos[:6] = target_gt # 先设为 GT 位置来计算 Sensor
        mujoco.mj_kinematics(model_2, data_2)

        mat_upper = data_2.site_xmat[site_upper_id].reshape(3, 3)
        mat_fore = data_2.site_xmat[site_fore_id].reshape(3, 3)
        q_upper = R.from_matrix(mat_upper).as_quat()
        q_fore = R.from_matrix(mat_fore).as_quat()

        # B. 校准 (第一帧)
        if i == 0:
            # 传入 offset，模拟我们已知的初始状态
            solver.calibrate(q_upper, q_fore, offset_joints=target_gt)

        # C. 解算
        solved_joints = solver.solve(q_upper, q_fore)

        # D. 设置解算后的姿态
        data_2.qpos[:6] = solved_joints
        mujoco.mj_forward(model_2, data_2)

        renderer_2.update_scene(data_2, camera=cam)
        img_2 = renderer_2.render()

        img_2 = cv2.cvtColor(img_2, cv2.COLOR_RGB2BGR)
        cv2.putText(img_2, "IMU Solver (Calculated)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

        # --- 拼接 ---
        # 水平拼接
        combined = np.hstack((img_1, img_2))
        out.write(combined)

        if i % 50 == 0:
            print(f"Frame {i}/{duration_frames}")

    out.release()
    print("Done!")

if __name__ == "__main__":
    main()
