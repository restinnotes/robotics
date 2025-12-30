"""
多角度评估脚本 - 隔几秒切换镜头角度
可以更清晰地观察机器人是否有姿态问题
"""

import os
import time
import argparse
import numpy as np

# 添加项目路径
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ur3e_blind_env import UR3eBlindIMUEnv
from simple_jax_ppo import PPOAgent


def main(args):
    print("多角度评估...")

    model_path = os.path.join(args.model_dir, "blind_ppo_params.msgpack")
    if not os.path.exists(model_path):
        print(f"未找到模型: {model_path}")
        return

    # 创建环境
    env = UR3eBlindIMUEnv(render_mode="human", imu_stack_size=10)
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

    print(f"运行中... 每 {args.switch_interval} 秒切换镜头")
    print(f"当前镜头: {camera_angles[current_cam_idx]['name']}")

    for step in range(args.max_steps):
        # 执行动作
        action, _ = agent.get_action(obs, rng=None)
        obs, reward, terminated, truncated, info = env.step(action)

        if terminated or truncated:
            obs, _ = env.reset()

        # 检查是否需要切换镜头
        if time.time() - last_switch_time > args.switch_interval:
            current_cam_idx = (current_cam_idx + 1) % len(camera_angles)
            cam = camera_angles[current_cam_idx]

            # 更新镜头
            if env.viewer is not None:
                env.viewer.cam.azimuth = cam["azimuth"]
                env.viewer.cam.elevation = cam["elevation"]
                env.viewer.cam.distance = cam["distance"]

            print(f"Step {step}: 切换到 [{cam['name']}] 角度")
            last_switch_time = time.time()

        # 控制帧率
        time.sleep(0.02)

    env.close()
    print("评估完成")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model_dir", type=str, default="./models")
    parser.add_argument("--max_steps", type=int, default=2000)
    parser.add_argument("--switch_interval", type=float, default=3.0, help="镜头切换间隔(秒)")

    args = parser.parse_args()
    main(args)
