"""
Record arm trajectories from loco-mujoco DEFAULT walk dataset.
This dataset has proper arm swing motion for walking.
"""

import numpy as np
import os

from loco_mujoco.task_factories import ImitationFactory, DefaultDatasetConf


def record_default_walk_arm(output_dir="data", n_steps=2000):
    """
    Record arm joint trajectories from the default walk dataset.
    """
    print("Loading default 'walk' dataset...")

    # Create environment with default walk dataset
    env = ImitationFactory.make(
        "UnitreeH1",
        default_dataset_conf=DefaultDatasetConf(["walk"]),
        n_substeps=20
    )

    print("Environment created. Recording trajectory...")

    # Get model and find arm joint indices
    model = env.model

    # Find arm joint qpos indices
    arm_joints_info = []
    for i in range(model.njnt):
        name = model.joint(i).name
        name_lower = name.lower()
        if any(x in name_lower for x in ['arm', 'shoulder', 'elbow', 'wrist']):
            qpos_addr = model.jnt_qposadr[i]
            arm_joints_info.append({
                'name': name,
                'idx': i,
                'qpos_addr': qpos_addr
            })
            print(f"  Joint [{i}] '{name}' -> qpos_addr: {qpos_addr}")

    if not arm_joints_info:
        print("No arm joints found!")
        return None

    qpos_addrs = [j['qpos_addr'] for j in arm_joints_info]
    joint_names = [j['name'] for j in arm_joints_info]

    # Reset environment
    env.reset()

    # Record trajectory
    recorded_qpos = []
    recorded_qvel = []

    print(f"Recording {n_steps} steps...")

    for step in range(n_steps):
        qpos = env.data.qpos[qpos_addrs].copy()
        qvel_size = len(env.data.qvel)
        qvel_addrs_safe = [min(a, qvel_size - 1) for a in qpos_addrs]
        qvel = env.data.qvel[qvel_addrs_safe].copy()

        recorded_qpos.append(qpos)
        recorded_qvel.append(qvel)

        try:
            action = np.zeros(env.action_dim) if hasattr(env, 'action_dim') else np.zeros(model.nu)
            obs, reward, done, truncated, info = env.step(action)
        except:
            break

        if done or truncated:
            env.reset()

    recorded_qpos = np.array(recorded_qpos)
    recorded_qvel = np.array(recorded_qvel)

    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "default_walk_arm.npz")

    frequency = 1.0 / (20 * 0.002)  # 25 Hz

    np.savez(
        output_path,
        qpos=recorded_qpos,
        qvel=recorded_qvel,
        joint_names=joint_names,
        frequency=frequency,
        n_samples=recorded_qpos.shape[0]
    )

    print(f"\nSaved arm trajectory to: {output_path}")
    print(f"  Shape: {recorded_qpos.shape}")
    print(f"  Joints: {joint_names}")
    print(f"  Duration: {recorded_qpos.shape[0] / frequency:.2f} seconds")

    return output_path


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=2000)
    parser.add_argument("--output", default="data")
    args = parser.parse_args()
    record_default_walk_arm(args.output, args.steps)
