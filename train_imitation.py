"""
UR3e 模仿学习训练脚本 (JAX Version)
使用自定义的 JAX PPO 算法，无需 Stable-Baselines3
"""

import os
import argparse
from ur3e_imitation_env import UR3eImitationEnv
from simple_jax_ppo import PPOAgent


def train(args):
    """训练主函数"""
    print("=" * 50)
    print("UR3e 模仿学习训练 (JAX Mode)")
    print("=" * 50)

    # 创建输出目录
    os.makedirs(args.model_dir, exist_ok=True)

    # 创建环境
    env = UR3eImitationEnv(render_mode=None, use_both_arms=True)

    # 创建并运行 PPO Agent
    agent = PPOAgent(env, learning_rate=3e-4)

    print(f"开始训练 {args.total_timesteps} 步...")
    agent.train(total_timesteps=args.total_timesteps)

    # 保存模型
    save_path = os.path.join(args.model_dir, "jax_ppo_params.msgpack")
    agent.save(save_path)
    print(f"模型已保存至: {save_path}")

    env.close()


def evaluate(args):
    """评估模型"""
    import time

    print("加载模型进行评估...")
    save_path = os.path.join(args.model_dir, "jax_ppo_params.msgpack")

    if not os.path.exists(save_path):
        print(f"未找到模型文件: {save_path}")
        return

    env = UR3eImitationEnv(render_mode="human", use_both_arms=True)
    agent = PPOAgent(env)
    agent.load(save_path)

    obs, _ = env.reset()
    total_reward = 0

    print(f"运行评估 (步数: {args.eval_steps})... (按 ESC 退出窗口)")

    for step in range(args.eval_steps):
        action, _ = agent.get_action(obs, rng=None)

        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward

        if step % 50 == 0:
            print(f"Step {step}: r={reward:.4f}")

        if terminated or truncated:
            obs, _ = env.reset()

        # 与参考轨迹帧率同步 (50 Hz)
        time.sleep(0.02)

    env.close()
    print(f"评估完成.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", type=str, default="train", choices=["train", "eval"])
    parser.add_argument("--total_timesteps", type=int, default=100_000, help="训练总步数")
    parser.add_argument("--eval_steps", type=int, default=1000, help="评估步数")
    parser.add_argument("--model_dir", type=str, default="./models", help="模型保存目录")

    args = parser.parse_args()

    if args.mode == "train":
        train(args)
    else:
        evaluate(args)
