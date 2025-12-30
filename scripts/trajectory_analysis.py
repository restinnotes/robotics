"""
轨迹对比分析脚本
对比 RL 模型输出轨迹与 Ground Truth 轨迹
计算误差百分位数，验证模型在各种校准偏置下的表现
"""

import os
import sys
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R

# Add project root
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ur3e_blind_env import UR3eBlindIMUEnv
from simple_jax_ppo import PPOAgent


def compute_statistics(errors):
    """计算误差统计信息"""
    return {
        "mean": np.mean(errors),
        "std": np.std(errors),
        "min": np.min(errors),
        "max": np.max(errors),
        "p50": np.percentile(errors, 50),
        "p90": np.percentile(errors, 90),
        "p95": np.percentile(errors, 95),
        "p99": np.percentile(errors, 99),
    }


def main(args):
    print("=" * 70)
    print("轨迹对比分析 (Trajectory Comparison Analysis)")
    print("=" * 70)
    print(f"随机偏置范围: ±{args.calib_deg}°")
    print(f"Episode 数量: {args.n_episodes}")
    print(f"每个 Episode 步数: {args.episode_steps}")
    print("=" * 70)

    model_path = os.path.join(args.model_dir, "blind_ppo_params.msgpack")
    if not os.path.exists(model_path):
        print(f"未找到模型: {model_path}")
        return

    # 创建环境 (启用校准随机化)
    env = UR3eBlindIMUEnv(
        render_mode=None,  # 无渲染，纯数据收集
        imu_stack_size=10,
        calib_randomize_deg=args.calib_deg
    )
    agent = PPOAgent(env)
    agent.load(model_path)

    # 加载 Ground Truth 轨迹
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    traj_path = os.path.join(project_root, "data", "walk_arm_direct.npz")
    traj = np.load(traj_path)
    ref_qpos = traj["qpos"]
    n_frames = ref_qpos.shape[0]

    # 存储所有误差
    all_joint_errors = []  # 关节角度误差 (弧度)
    all_rewards = []
    episode_stats = []

    print("\n开始数据收集...")

    for ep in range(args.n_episodes):
        obs, _ = env.reset()
        episode_errors = []
        episode_rewards = []

        for step in range(args.episode_steps):
            # 获取当前帧的 Ground Truth
            frame_idx = env.frame_idx
            ref = ref_qpos[frame_idx]

            # Ground Truth 映射 (与环境一致)
            gt_qpos = np.zeros(6)
            gt_qpos[0] = ref[1]           # Pan
            gt_qpos[1] = ref[0]           # Lift
            gt_qpos[2] = ref[3]           # Elbow
            gt_qpos[3] = -1.57 + ref[2]   # Wrist1
            gt_qpos[4] = -1.57
            gt_qpos[5] = 0

            # RL 动作
            action, _ = agent.get_action(obs, rng=None)
            obs, reward, terminated, truncated, info = env.step(action)

            # 获取实际关节位置
            actual_qpos = env.data.qpos[:6].copy()

            # 计算关节误差 (前4个关节，因为后2个是固定的)
            joint_error = np.abs(actual_qpos[:4] - gt_qpos[:4])
            episode_errors.append(joint_error)
            episode_rewards.append(reward)

            if terminated or truncated:
                break

        # Episode 统计
        ep_errors = np.array(episode_errors)
        ep_mean_error = np.mean(ep_errors) * 180 / np.pi  # 转为度
        ep_max_error = np.max(ep_errors) * 180 / np.pi
        ep_mean_reward = np.mean(episode_rewards)

        episode_stats.append({
            "episode": ep,
            "mean_error_deg": ep_mean_error,
            "max_error_deg": ep_max_error,
            "mean_reward": ep_mean_reward,
            "steps": len(episode_errors)
        })

        all_joint_errors.extend(episode_errors)
        all_rewards.extend(episode_rewards)

        print(f"  Episode {ep+1}/{args.n_episodes}: "
              f"平均误差={ep_mean_error:.2f}°, "
              f"最大误差={ep_max_error:.2f}°, "
              f"平均奖励={ep_mean_reward:.4f}")

    env.close()

    # 汇总统计
    all_errors_deg = np.array(all_joint_errors) * 180 / np.pi  # 转为度
    total_error_per_step = np.mean(all_errors_deg, axis=1)  # 每步平均误差

    print("\n" + "=" * 70)
    print("汇总统计 (Summary Statistics)")
    print("=" * 70)

    stats = compute_statistics(total_error_per_step)
    print(f"\n关节误差统计 (度):")
    print(f"  平均值 (Mean):     {stats['mean']:.3f}°")
    print(f"  标准差 (Std):      {stats['std']:.3f}°")
    print(f"  最小值 (Min):      {stats['min']:.3f}°")
    print(f"  最大值 (Max):      {stats['max']:.3f}°")
    print(f"  50%分位 (Median):  {stats['p50']:.3f}°")
    print(f"  90%分位:           {stats['p90']:.3f}°")
    print(f"  95%分位:           {stats['p95']:.3f}°")
    print(f"  99%分位:           {stats['p99']:.3f}°")

    reward_stats = compute_statistics(all_rewards)
    print(f"\n奖励统计:")
    print(f"  平均值: {reward_stats['mean']:.4f}")
    print(f"  标准差: {reward_stats['std']:.4f}")
    print(f"  50%分位: {reward_stats['p50']:.4f}")
    print(f"  90%分位: {reward_stats['p90']:.4f}")

    # 计算"在正确范围内"的比例
    threshold_deg = args.threshold
    in_range_ratio = np.mean(total_error_per_step < threshold_deg) * 100
    print(f"\n误差 < {threshold_deg}° 的比例: {in_range_ratio:.1f}%")

    # 保存结果
    if args.save:
        result_path = os.path.join(args.model_dir, "trajectory_analysis.npz")
        np.savez(
            result_path,
            all_errors_deg=all_errors_deg,
            all_rewards=np.array(all_rewards),
            episode_stats=episode_stats,
            summary_stats=stats,
            calib_deg=args.calib_deg,
            threshold_deg=threshold_deg,
            in_range_ratio=in_range_ratio
        )
        print(f"\n结果已保存至: {result_path}")

    print("\n" + "=" * 70)
    print("分析完成!")
    print("=" * 70)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_dir", type=str, default="./models")
    parser.add_argument("--n_episodes", type=int, default=10, help="Episode 数量")
    parser.add_argument("--episode_steps", type=int, default=500, help="每个 Episode 步数")
    parser.add_argument("--calib_deg", type=float, default=30.0, help="校准随机化角度(±度)")
    parser.add_argument("--threshold", type=float, default=10.0, help="误差阈值(度)")
    parser.add_argument("--save", action="store_true", help="保存结果到文件")

    args = parser.parse_args()
    main(args)
