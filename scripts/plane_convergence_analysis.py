"""
平面回归分析 - 专门检查 shoulder_pan 关节
验证模型是否能在校准偏置下回归到正确的运动平面
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ur3e_blind_env import UR3eBlindIMUEnv
from simple_jax_ppo import PPOAgent


def main():
    print("=" * 70)
    print("平面回归分析 (Plane Convergence Analysis)")
    print("=" * 70)

    # 配置
    n_episodes = 5
    episode_steps = 500
    calib_deg = 30.0

    print(f"随机偏置范围: ±{calib_deg}°")
    print(f"Episode 数量: {n_episodes}")
    print(f"每个 Episode 步数: {episode_steps}")
    print("=" * 70)

    model_path = "./models/blind_ppo_params.msgpack"
    if not os.path.exists(model_path):
        print(f"未找到模型: {model_path}")
        return

    # 创建环境
    env = UR3eBlindIMUEnv(
        render_mode=None,
        imu_stack_size=10,
        calib_randomize_deg=calib_deg
    )
    agent = PPOAgent(env)
    agent.load(model_path)

    # 加载 Ground Truth
    traj = np.load("./data/walk_arm_direct.npz")
    ref_qpos = traj["qpos"]

    print("\n开始分析...")

    all_episode_data = []

    for ep in range(n_episodes):
        obs, _ = env.reset()

        episode_data = {
            "pan_errors": [],      # shoulder_pan 误差
            "lift_errors": [],     # shoulder_lift 误差
            "elbow_errors": [],    # elbow 误差
            "pan_actual": [],      # 实际 shoulder_pan
            "pan_gt": [],          # GT shoulder_pan
        }

        for step in range(episode_steps):
            # Ground Truth
            frame_idx = env.frame_idx
            ref = ref_qpos[frame_idx]
            gt_pan = ref[1]   # l_arm_shx -> shoulder_pan
            gt_lift = ref[0]  # l_arm_shy -> shoulder_lift
            gt_elbow = ref[3] # left_elbow -> elbow

            # RL 动作
            action, _ = agent.get_action(obs, rng=None)
            obs, reward, terminated, truncated, info = env.step(action)

            # 实际值
            actual = env.data.qpos[:6]

            # 记录误差
            episode_data["pan_errors"].append(abs(actual[0] - gt_pan) * 180 / np.pi)
            episode_data["lift_errors"].append(abs(actual[1] - gt_lift) * 180 / np.pi)
            episode_data["elbow_errors"].append(abs(actual[2] - gt_elbow) * 180 / np.pi)
            episode_data["pan_actual"].append(actual[0] * 180 / np.pi)
            episode_data["pan_gt"].append(gt_pan * 180 / np.pi)

            if terminated or truncated:
                break

        all_episode_data.append(episode_data)

        # 打印关键数据
        pan_errors = episode_data["pan_errors"]
        print(f"\nEpisode {ep+1}:")
        print(f"  Shoulder Pan 误差:")
        print(f"    前 50 步平均: {np.mean(pan_errors[:50]):.2f}°")
        print(f"    中 50 步平均 (225-275): {np.mean(pan_errors[225:275]):.2f}°")
        print(f"    后 50 步平均: {np.mean(pan_errors[-50:]):.2f}°")
        print(f"    整体平均: {np.mean(pan_errors):.2f}°")

        # 判断是否收敛
        first_half = np.mean(pan_errors[:250])
        second_half = np.mean(pan_errors[250:])
        if second_half < first_half * 0.8:
            print(f"    ✅ 趋势: 误差在下降 (自动回正)")
        elif second_half > first_half * 1.2:
            print(f"    ❌ 趋势: 误差在增加 (漂移)")
        else:
            print(f"    ➡️ 趋势: 误差稳定")

    env.close()

    # 汇总分析
    print("\n" + "=" * 70)
    print("汇总分析")
    print("=" * 70)

    all_pan_first = []
    all_pan_last = []

    for ep, data in enumerate(all_episode_data):
        pan = data["pan_errors"]
        all_pan_first.append(np.mean(pan[:100]))
        all_pan_last.append(np.mean(pan[-100:]))

    print(f"\n各 Episode 的 Shoulder Pan 误差对比:")
    print(f"  前 100 步平均: {all_pan_first}")
    print(f"  后 100 步平均: {all_pan_last}")

    avg_first = np.mean(all_pan_first)
    avg_last = np.mean(all_pan_last)

    print(f"\n总体平均:")
    print(f"  前 100 步: {avg_first:.2f}°")
    print(f"  后 100 步: {avg_last:.2f}°")

    if avg_last < avg_first * 0.7:
        print(f"\n✅✅ 结论: 模型会自动回归到正确平面 (误差下降 {(1 - avg_last/avg_first)*100:.1f}%)")
    elif avg_last < avg_first * 0.9:
        print(f"\n✅ 结论: 模型有轻微的自我纠正能力")
    elif avg_last > avg_first * 1.1:
        print(f"\n❌ 结论: 模型在错误平面上运行,未能回归")
    else:
        print(f"\n➡️ 结论: 模型保持稳定误差,既没有恶化也没有明显回归")

    print("\n" + "=" * 70)


if __name__ == "__main__":
    main()
