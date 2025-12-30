"""
校准随机化验证脚本
验证 RL 模型是否能在各种随机初始校准偏置下正确运行
- 每个 Episode 应用不同的随机偏置 (模拟用户戴歪 IMU)
- 三视图自动切换
- 观察模型是否能稳定跟踪轨迹
"""

import os
import sys
import time
import argparse
import numpy as np

# Add project root
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ur3e_blind_env import UR3eBlindIMUEnv
from simple_jax_ppo import PPOAgent


def main(args):
    print("=" * 60)
    print("校准随机化验证 (Calibration Randomization Test)")
    print("=" * 60)
    print(f"随机偏置范围: ±{args.calib_deg}°")
    print(f"每个 Episode 步数: {args.episode_steps}")
    print(f"镜头切换间隔: {args.switch_interval}s")
    print("=" * 60)

    model_path = os.path.join(args.model_dir, "blind_ppo_params.msgpack")
    if not os.path.exists(model_path):
        print(f"未找到模型: {model_path}")
        return

    # 创建环境 (启用校准随机化)
    env = UR3eBlindIMUEnv(
        render_mode="human",
        imu_stack_size=10,
        calib_randomize_deg=args.calib_deg  # 启用随机偏置
    )
    agent = PPOAgent(env)
    agent.load(model_path)

    # 三个镜头角度
    camera_angles = [
        {"name": "正面", "azimuth": 90, "elevation": 0, "distance": 1.2},
        {"name": "侧面", "azimuth": 0, "elevation": 0, "distance": 1.2},
        {"name": "俯视", "azimuth": 90, "elevation": -45, "distance": 1.5},
    ]

    obs, _ = env.reset()
    current_cam_idx = 0
    last_switch_time = time.time()
    episode_count = 0
    step_in_episode = 0
    episode_reward = 0.0

    print(f"\n[Episode {episode_count}] 开始 (随机校准偏置已应用)")
    print(f"当前镜头: {camera_angles[current_cam_idx]['name']}")

    for total_step in range(args.max_steps):
        # 执行动作
        action, _ = agent.get_action(obs, rng=None)
        obs, reward, terminated, truncated, info = env.step(action)
        episode_reward += reward
        step_in_episode += 1

        # Episode 结束 (固定步数或环境终止)
        if step_in_episode >= args.episode_steps or terminated or truncated:
            avg_reward = episode_reward / step_in_episode
            print(f"[Episode {episode_count}] 结束 | 平均奖励: {avg_reward:.4f}")

            # 重置环境 (新的随机偏置会在 reset 时自动应用)
            obs, _ = env.reset()
            episode_count += 1
            step_in_episode = 0
            episode_reward = 0.0
            print(f"\n[Episode {episode_count}] 开始 (新的随机校准偏置已应用)")

        # 检查是否需要切换镜头
        if time.time() - last_switch_time > args.switch_interval:
            current_cam_idx = (current_cam_idx + 1) % len(camera_angles)
            cam = camera_angles[current_cam_idx]

            if env.viewer is not None:
                env.viewer.cam.azimuth = cam["azimuth"]
                env.viewer.cam.elevation = cam["elevation"]
                env.viewer.cam.distance = cam["distance"]

            print(f"  >> 切换到 [{cam['name']}] 视角")
            last_switch_time = time.time()

        # 控制帧率
        time.sleep(0.02)

    env.close()
    print("\n验证完成!")
    print(f"共运行 {episode_count + 1} 个 Episode，每个 Episode 都使用了不同的随机校准偏置。")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_dir", type=str, default="./models")
    parser.add_argument("--max_steps", type=int, default=3000, help="总步数")
    parser.add_argument("--episode_steps", type=int, default=500, help="每个Episode步数")
    parser.add_argument("--switch_interval", type=float, default=5.0, help="镜头切换间隔(秒)")
    parser.add_argument("--calib_deg", type=float, default=30.0, help="校准随机化角度(±度)")

    args = parser.parse_args()
    main(args)
