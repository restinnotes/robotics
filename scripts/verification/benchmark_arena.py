"""
对比竞技场 (Benchmark Arena)
同屏显示两个机器人对比抗噪能力：
- 🔴 红色 = 数学解算器 (imu_solver.py)
- 🟢 绿色 = RL 策略 (blind_ppo)

两者接收相同的噪声输入，直观验证 RL 的抗噪价值。
"""

import time
import mujoco
import numpy as np
import cv2
import os
import argparse
from scipy.spatial.transform import Rotation as R

from utils.imu_solver import IMUSolver
from utils.noise_injector import NoiseInjector, get_injector_by_preset
from simple_jax_ppo import PPOAgent
from ur3e_blind_env import UR3eBlindIMUEnv


def main(args):
    # --- 路径设置 ---
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
    traj_path = os.path.join(current_dir, "data", "walk_arm_direct.npz")

    # --- 加载数据 ---
    traj = np.load(traj_path, allow_pickle=True)
    ref_qpos = traj["qpos"]
    n_frames = ref_qpos.shape[0]

    # --- 初始化两个机器人环境 ---
    # Robot 1: 数学解算器驱动
    model_math = mujoco.MjModel.from_xml_path(model_path)
    data_math = mujoco.MjData(model_math)
    renderer_math = mujoco.Renderer(model_math, height=480, width=640)

    # Robot 2: RL 策略驱动
    model_rl = mujoco.MjModel.from_xml_path(model_path)
    data_rl = mujoco.MjData(model_rl)
    renderer_rl = mujoco.Renderer(model_rl, height=480, width=640)

    # 相机设置
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, 0.7]
    cam.distance = 1.2
    cam.elevation = 0
    cam.azimuth = 90

    # IMU Site IDs
    site_upper_id = mujoco.mj_name2id(model_math, mujoco.mjtObj.mjOBJ_SITE, "upper_arm_imu_site")
    site_fore_id = mujoco.mj_name2id(model_math, mujoco.mjtObj.mjOBJ_SITE, "forearm_imu_site")

    # --- 初始化解算器和噪声注入器 ---
    solver = IMUSolver()
    noise_injector = get_injector_by_preset(args.noise_level, seed=42)
    noise_injector.reset()

    # --- 加载 RL 策略 ---
    # 创建一个临时环境来加载策略
    temp_env = UR3eBlindIMUEnv(render_mode=None)
    agent = PPOAgent(temp_env)

    model_file = os.path.join(args.model_dir, "blind_ppo_params.msgpack")
    if os.path.exists(model_file):
        agent.load(model_file)
        print(f"Loaded RL model from {model_file}")
    else:
        print(f"Warning: No RL model found at {model_file}, using random policy")

    # RL 环境状态
    imu_stack_size = 10
    imu_history = np.zeros((imu_stack_size, 8), dtype=np.float32)

    # 视频写入
    if args.save_video:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(args.output, fourcc, 50, (1280, 480))
        print(f"Recording to {args.output}")

    print(f"Running Benchmark Arena with noise level: {args.noise_level}")
    print("Press Ctrl+C to stop")

    try:
        for step in range(args.duration):
            frame_idx = step % n_frames

            # --- 1. 生成 Ground Truth 姿态 ---
            ref = ref_qpos[frame_idx]
            target_gt = np.zeros(6)
            target_gt[0] = ref[1]
            target_gt[1] = ref[0]
            target_gt[2] = ref[3]
            target_gt[3] = -1.57 + ref[2]
            target_gt[4] = -1.57
            target_gt[5] = 0

            # --- 2. 生成虚拟 IMU 数据 (用 Robot 1 的 FK) ---
            data_math.qpos[:6] = target_gt
            mujoco.mj_kinematics(model_math, data_math)

            mat_upper = data_math.site_xmat[site_upper_id].reshape(3, 3)
            mat_fore = data_math.site_xmat[site_fore_id].reshape(3, 3)
            q_upper_clean = R.from_matrix(mat_upper).as_quat().astype(np.float32)
            q_fore_clean = R.from_matrix(mat_fore).as_quat().astype(np.float32)

            # --- 3. 注入噪声 (两个机器人接收同样的噪声数据) ---
            q_upper_noisy, q_fore_noisy = noise_injector.inject_noise_to_imu_reading(
                q_upper_clean, q_fore_clean
            )

            # --- 4. Robot 1: 数学解算器 ---
            if step == 0:
                solver.calibrate(q_upper_noisy, q_fore_noisy, offset_joints=target_gt)

            solved_joints = solver.solve(q_upper_noisy, q_fore_noisy)
            data_math.qpos[:6] = solved_joints
            mujoco.mj_forward(model_math, data_math)

            # --- 5. Robot 2: RL 策略 ---
            # 更新 IMU 历史
            imu_reading_noisy = np.concatenate([q_upper_noisy, q_fore_noisy])
            imu_history = np.roll(imu_history, -1, axis=0)
            imu_history[-1] = imu_reading_noisy

            # 构建观测
            qpos_rl = data_rl.qpos[:6].copy()
            qvel_rl = data_rl.qvel[:6].copy()
            obs = np.concatenate([qpos_rl, qvel_rl, imu_history.flatten()]).astype(np.float32)

            # 获取动作
            action, _ = agent.get_action(obs, rng=None)

            # 残差控制 (与 ur3e_blind_env 一致)
            scaled_action = action * 0.1
            data_rl.ctrl[:6] = target_gt + scaled_action

            for _ in range(10):
                mujoco.mj_step(model_rl, data_rl)

            # --- 6. 渲染 ---
            renderer_math.update_scene(data_math, camera=cam)
            img_math = renderer_math.render()
            img_math = cv2.cvtColor(img_math, cv2.COLOR_RGB2BGR)
            cv2.putText(img_math, "MATH SOLVER", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(img_math, f"Noise: {args.noise_level}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            renderer_rl.update_scene(data_rl, camera=cam)
            img_rl = renderer_rl.render()
            img_rl = cv2.cvtColor(img_rl, cv2.COLOR_RGB2BGR)
            cv2.putText(img_rl, "RL POLICY", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(img_rl, f"Noise: {args.noise_level}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 拼接
            combined = np.hstack((img_math, img_rl))

            if args.save_video:
                out.write(combined)

            if step % 100 == 0:
                # 计算与 GT 的误差
                error_math = np.mean(np.abs(data_math.qpos[:6] - target_gt))
                error_rl = np.mean(np.abs(data_rl.qpos[:6] - target_gt))
                print(f"Step {step}: Math Error = {error_math:.4f} rad, RL Error = {error_rl:.4f} rad")

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nStopped by user")

    if args.save_video:
        out.release()
        print(f"Video saved to {args.output}")

    temp_env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--noise_level", type=str, default="moderate",
                        choices=["clean", "mild", "moderate", "severe", "extreme"])
    parser.add_argument("--duration", type=int, default=1000, help="帧数")
    parser.add_argument("--model_dir", type=str, default="./models")
    parser.add_argument("--save_video", action="store_true", help="保存视频")
    parser.add_argument("--output", type=str, default="benchmark_arena.mp4")

    args = parser.parse_args()
    main(args)
