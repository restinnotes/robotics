"""
UR3e 盲操 RL 训练脚本 (Blind IMU Training)
Phase 3: 训练 Agent 只用 IMU 历史来控制机器人
"""

import os
import argparse
import time
from ur3e_blind_env import UR3eBlindIMUEnv
from simple_jax_ppo import PPOAgent


def train(args):
    """训练主函数"""
    print("=" * 50)
    print("UR3e 盲操 RL 训练 (Blind IMU Mode)")
    print("=" * 50)

    # 创建输出目录
    os.makedirs(args.model_dir, exist_ok=True)

    # 创建环境
    env = UR3eBlindIMUEnv(render_mode=None, imu_stack_size=args.imu_stack)

    # 创建并运行 PPO Agent
    agent = PPOAgent(env, learning_rate=args.lr)

    print(f"开始训练 {args.total_timesteps} 步...")
    print(f"IMU Stack Size: {args.imu_stack}")
    print(f"Observation Dim: {env.observation_space.shape[0]}")

    agent.train(total_timesteps=args.total_timesteps)

    # 保存模型
    save_path = os.path.join(args.model_dir, "blind_ppo_params.msgpack")
    agent.save(save_path)
    print(f"模型已保存至: {save_path}")

    env.close()


def evaluate(args):
    """评估模型"""
    print("加载盲操模型进行评估...")
    save_path = os.path.join(args.model_dir, "blind_ppo_params.msgpack")

    if not os.path.exists(save_path):
        print(f"未找到模型文件: {save_path}")
        return

    env = UR3eBlindIMUEnv(render_mode="human", imu_stack_size=args.imu_stack)
    agent = PPOAgent(env)
    agent.load(save_path)

    obs, _ = env.reset()
    total_reward = 0

    print("运行评估... (按 ESC 退出窗口)")

    for step in range(2000):
        action, _ = agent.get_action(obs, rng=None)

        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward

        if step % 50 == 0:
            print(f"Step {step}: r={reward:.4f}")

        if terminated or truncated:
            obs, _ = env.reset()

        # 与 50 Hz 同步
        time.sleep(0.02)

    env.close()
    print(f"评估完成. Total Reward: {total_reward:.2f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", type=str, default="train", choices=["train", "eval"])
    parser.add_argument("--total_timesteps", type=int, default=200_000, help="训练总步数")
    parser.add_argument("--model_dir", type=str, default="./models", help="模型保存目录")
    parser.add_argument("--imu_stack", type=int, default=10, help="IMU 历史堆叠帧数")
    parser.add_argument("--lr", type=float, default=3e-4, help="学习率")

    args = parser.parse_args()

    if args.mode == "train":
        train(args)
    else:
        evaluate(args)
