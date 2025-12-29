"""
Correctly extract arm trajectory by reading directly from loco-mujoco's trajectory data structure,
NOT by stepping through the environment with zero action.
"""

import numpy as np
import os
from loco_mujoco.task_factories import ImitationFactory, DefaultDatasetConf

def extract_trajectory_directly(output_dir="data", n_frames=1000):
    """
    Extract arm joint data directly from the trajectory data structure.
    """
    print("Loading default 'walk' dataset...")

    env = ImitationFactory.make(
        "UnitreeH1",
        default_dataset_conf=DefaultDatasetConf(["walk"]),
        n_substeps=20
    )

    # Access the trajectory handler which contains the actual trajectory data
    th = env.th  # trajectory handler

    if th is None:
        print("No trajectory handler found!")
        return None

    print(f"Trajectory handler found!")
    print(f"  Number of trajectories: {th.n_trajectories}")

    # Get trajectory info
    traj = th.traj
    print(f"  Trajectory qpos shape: {traj.data.qpos.shape}")
    print(f"  Joint names: {traj.info.joint_names}")
    print(f"  Frequency: {traj.info.frequency} Hz")

    # Find arm joints in the trajectory
    arm_joints = []
    for i, name in enumerate(traj.info.joint_names):
        name_lower = name.lower()
        if any(x in name_lower for x in ['arm', 'shoulder', 'elbow', 'wrist']):
            arm_joints.append((i, name))
            print(f"  Arm joint [{i}]: {name}")

    if not arm_joints:
        print("No arm joints found!")
        return None

    # Get qpos indices for arm joints
    # The trajectory stores qpos in order of joints, but we need to handle free joints
    # which have 7 DOF (3 pos + 4 quat) vs hinge joints which have 1 DOF
    arm_qpos_indices = []
    for joint_idx, name in arm_joints:
        qpos_indices = traj.info.joint_name2ind_qpos[name]
        arm_qpos_indices.extend(qpos_indices.tolist())
        print(f"  {name}: qpos indices {qpos_indices}")

    # Extract qpos data - convert to numpy
    qpos_all = np.array(traj.data.qpos)
    n_frames = min(n_frames, qpos_all.shape[0])

    # Extract arm joint data
    arm_qpos = qpos_all[:n_frames, arm_qpos_indices]

    print(f"\nExtracted arm trajectory:")
    print(f"  Shape: {arm_qpos.shape}")
    print(f"  Frames: {n_frames}")
    print(f"  Range per joint:")
    for i, (_, name) in enumerate(arm_joints):
        print(f"    {name}: [{arm_qpos[:,i].min():.3f}, {arm_qpos[:,i].max():.3f}]")

    # Check for sudden jumps
    diff = np.abs(np.diff(arm_qpos, axis=0))
    max_jumps = diff.max(axis=0)
    print(f"  Max frame-to-frame jumps:")
    for i, (_, name) in enumerate(arm_joints):
        print(f"    {name}: {max_jumps[i]:.4f} rad")

    # Save
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, "walk_arm_direct.npz")

    np.savez(
        output_path,
        qpos=arm_qpos,
        joint_names=[name for _, name in arm_joints],
        frequency=traj.info.frequency,
        n_samples=arm_qpos.shape[0]
    )

    print(f"\nSaved to: {output_path}")
    return output_path


if __name__ == "__main__":
    extract_trajectory_directly(n_frames=2000)
