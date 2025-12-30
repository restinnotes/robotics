"""
IMU Solver 验证脚本 - Phase 2
目标：验证 imu_solver.py 的数学逻辑是否正确
步骤：
1. 加载 reference trajectory (walk_arm_direct.npz)
2. 用 MuJoCo 正向运动学(FK)计算每一帧的 "虚拟 IMU 四元数"
3. 把这些四元数喂给 IMU Solver -> 得到 "解算关节角"
4. 对比 "解算关节角" vs "真实关节角"
"""

import mujoco
import numpy as np
import os
import matplotlib.pyplot as plt
from utils.imu_solver import IMUSolver
from scipy.spatial.transform import Rotation as R

def main():
    # 1. 路径设置
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
    traj_path = os.path.join(current_dir, "data", "walk_arm_direct.npz")

    # 2. 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # 3. 加载轨迹
    traj = np.load(traj_path, allow_pickle=True)
    ref_qpos = traj["qpos"]  # (N, 6)
    n_frames = ref_qpos.shape[0]

    print(f"Traj frames: {n_frames}")

    # 4. 获取 Site ID
    # 这些是我们在 xml 里加的
    site_upper_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "upper_arm_imu_site")
    site_fore_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "forearm_imu_site")

    if site_upper_id == -1 or site_fore_id == -1:
        print("Error: IMU sites not found in XML!")
        return

    # 5. 初始化 Solver
    solver = IMUSolver()

    # 6. 循环验证
    errors = []

    # 映射逻辑 (Direct Control 用的)：
    # target[0] = ref[1]           # l_arm_shx -> shoulder_pan
    # target[1] = ref[0]           # l_arm_shy -> shoulder_lift
    # target[2] = ref[3]           # left_elbow -> elbow
    # target[3] = -1.57 + ref[2]   # l_arm_shz -> wrist_1

    # 我们不仅要解算，还要看解算出来的值跟这个 target 像不像

    # 记录 Ground Truth 和 Solved
    gt_joints = []
    solved_joints = []

    for i in range(n_frames):
        # A. 构建当前帧的 GT 关节角 (UR3e 空间)
        # 注意：这里我们用 Direct Control 的映射逻辑来构建 "真实 UR3e 姿态"
        # 因为我们的 Solver 是要把 IMU 映射回 UR3e

        # 获取原始人体关节
        ref = ref_qpos[i]
        # 构建目标 (Ground Truth)
        target = np.zeros(6)
        target[0] = ref[1]           # shoulder_pan
        target[1] = ref[0]           # shoulder_lift
        target[2] = ref[3]           # elbow
        target[3] = -1.57 + ref[2]   # wrist_1 (原始逻辑)
        target[4] = -1.57
        target[5] = 0

        gt_joints.append(target)

        # B. 用这个 GT 姿态驱动 MuJoCo，产生 "虚拟 IMU 数据"
        data.qpos[:6] = target
        mujoco.mj_kinematics(model, data)

        # 获取 Site 的旋转矩阵 (flattened 3x3 -> 9)
        mat_upper = data.site_xmat[site_upper_id].reshape(3, 3)
        mat_fore = data.site_xmat[site_fore_id].reshape(3, 3)

        # 转为四元数 [x, y, z, w]
        q_upper = R.from_matrix(mat_upper).as_quat()
        q_fore = R.from_matrix(mat_fore).as_quat()

        # C. 校准 (第一帧)
        if i == 0:
            solver.calibrate(q_upper, q_fore, offset_joints=target)

        # D. 解算
        solved = solver.solve(q_upper, q_fore)
        solved_joints.append(solved)

        # 计算误差
        error = np.abs(solved - target)
        errors.append(error)

        if i % 100 == 0:
            print(f"Frame {i}: GT Elbow={target[2]:.4f}, Solved Elbow={solved[2]:.4f}")

    # 转换为 numpy 方便分析
    gt_joints = np.array(gt_joints)
    solved_joints = np.array(solved_joints)
    errors = np.array(errors)

    # 7. 打印统计结果
    joint_names = ["Pan", "Lift", "Elbow", "Wrist1", "Wrist2", "Wrist3"]
    print(f"\nTotal Mean Error: {np.mean(errors):.4f}")

    # 保存结果到文件以便读取
    output_file = os.path.join(current_dir, "validation_results.txt")
    with open(output_file, "w") as f:
        f.write("=== Validation Results ===\n")
        f.write("Mean Absolute Error (rad):\n")
        for j in range(6):
            mae = np.mean(errors[:, j])
            f.write(f"  {joint_names[j]}: {mae:.4f}\n")
        f.write(f"\nTotal Mean Error: {np.mean(errors):.4f}\n")
    print(f"Results saved to {output_file}")

    # 8. 可视化对比 (前 3 个关键关节)
    plt.figure(figsize=(12, 8))

    for j in range(3): # Pan, Lift, Elbow
        plt.subplot(3, 1, j+1)
        plt.plot(gt_joints[:, j], label='Ground Truth', color='black', alpha=0.7)
        plt.plot(solved_joints[:, j], label='Solved IK', color='orange', linestyle='--')
        plt.title(f"Joint: {joint_names[j]}")
        plt.legend()
        plt.grid(True)

    plt.tight_layout()
    plt.savefig("imu_solver_validation.png")
    print("\nPlot saved to imu_solver_validation.png")

if __name__ == "__main__":
    main()
