"""
Record arm trajectories from LAFAN1 dataset by stepping through the environment.
This captures the actual qpos values for arm joints during motion playback.
"""

import numpy as np
import os

from loco_mujoco.task_factories import ImitationFactory, LAFAN1DatasetConf


def record_arm_trajectory(dataset_name="walk1_subject1", output_dir="data", n_steps=500):
    """
    Record arm joint trajectories by stepping through the LAFAN1-based environment.
    """
    print(f"Loading LAFAN1 dataset: {dataset_name}")

    # Create environment
    env = ImitationFactory.make(
        "UnitreeH1",
        lafan1_dataset_conf=LAFAN1DatasetConf([dataset_name]),
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
            # Get qpos address for this joint
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

    # Extract qpos addresses
    qpos_addrs = [j['qpos_addr'] for j in arm_joints_info]
    joint_names = [j['name'] for j in arm_joints_info]

    # Reset environment
    env.reset()

    # Record trajectory
    recorded_qpos = []
    recorded_qvel = []

    print(f"Recording {n_steps} steps...")

    for step in range(n_steps):
        # Get current arm joint positions
        qpos = env.data.qpos[qpos_addrs].copy()
        # For velocity, cap indices to qvel size (qvel has fewer elements for free joint)
        qvel_size = len(env.data.qvel)
        qvel_addrs_safe = [min(a, qvel_size - 1) for a in qpos_addrs]
        qvel = env.data.qvel[qvel_addrs_safe].copy()

        recorded_qpos.append(qpos)
        recorded_qvel.append(qvel)

        # Step environment (zero action to follow reference trajectory)
        try:
            action = np.zeros(env.action_dim) if hasattr(env, 'action_dim') else np.zeros(model.nu)
            obs, reward, done, truncated, info = env.step(action)
        except:
            # Some envs may have different step signature
            break

        if done or truncated:
            env.reset()

    # Convert to numpy arrays
    recorded_qpos = np.array(recorded_qpos)
    recorded_qvel = np.array(recorded_qvel)

    # Save data
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, f"lafan1_{dataset_name}_arm.npz")

    # Estimate frequency (assuming 20 substeps at 0.002s timestep)
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

    parser = argparse.ArgumentParser(description="Record LAFAN1 arm trajectory")
    parser.add_argument("--dataset", default="walk1_subject1", help="LAFAN1 dataset name")
    parser.add_argument("--output", default="data", help="Output directory")
    parser.add_argument("--steps", type=int, default=500, help="Number of steps to record")

    args = parser.parse_args()

    record_arm_trajectory(args.dataset, args.output, args.steps)
