"""
Extract LAFAN1 walking arm data from loco-mujoco datasets.
This script loads the LAFAN1 walk dataset and extracts arm joint trajectories
for use with UR3e imitation learning.
"""

import numpy as np
import os

# Try to import loco-mujoco
try:
    from loco_mujoco.task_factories import ImitationFactory, LAFAN1DatasetConf
except ImportError as e:
    print(f"Error importing loco-mujoco: {e}")
    print("Please ensure loco-mujoco is installed: pip install -e ./loco-mujoco")
    exit(1)


def extract_lafan1_arm_data(dataset_name="walk1_subject1", output_dir="data"):
    """
    Extract arm joint data from LAFAN1 walking dataset.

    The humanoid model has joint names like:
    - left_shoulder, right_shoulder
    - left_elbow, right_elbow
    - left_wrist, right_wrist

    We'll extract these and map them to UR3e's 6 joints.
    """

    print(f"Loading LAFAN1 dataset: {dataset_name}")

    # Create environment to access the dataset
    env = ImitationFactory.make(
        "UnitreeH1",  # Using UnitreeH1 as it has good arm joints
        lafan1_dataset_conf=LAFAN1DatasetConf([dataset_name]),
        n_substeps=20
    )

    # Get trajectory info
    print("\nDataset loaded successfully!")

    # Try different ways to access trajectory
    traj = None
    for attr in ['_traj', 'traj', '_trajectory', 'trajectory']:
        if hasattr(env, attr):
            val = getattr(env, attr)
            if val is not None and hasattr(val, 'data') and hasattr(val, 'info'):
                traj = val
                print(f"Found trajectory via '{attr}'")
                break

    if traj is not None:
        print(f"Trajectory data shape - qpos: {traj.data.qpos.shape}")
        print(f"Joint names: {traj.info.joint_names}")
        print(f"Frequency: {traj.info.frequency} Hz")

        # Find arm-related joints
        arm_joints = []
        for i, name in enumerate(traj.info.joint_names):
            name_lower = name.lower()
            if any(x in name_lower for x in ['shoulder', 'elbow', 'wrist', 'arm']):
                arm_joints.append((i, name))
                print(f"  Found arm joint [{i}]: {name}")

        if arm_joints:
            # Extract arm joint indices
            arm_indices = [idx for idx, _ in arm_joints]

            # Get qpos data for arm joints
            qpos_all = np.array(traj.data.qpos)
            arm_qpos = qpos_all[:, arm_indices]

            # Save the extracted data
            os.makedirs(output_dir, exist_ok=True)
            output_path = os.path.join(output_dir, f"lafan1_{dataset_name}_arm.npz")

            np.savez(
                output_path,
                qpos=arm_qpos,
                joint_names=[name for _, name in arm_joints],
                frequency=traj.info.frequency,
                n_samples=arm_qpos.shape[0]
            )
            print(f"\nSaved arm data to: {output_path}")
            print(f"  Shape: {arm_qpos.shape}")
            print(f"  Duration: {arm_qpos.shape[0] / traj.info.frequency:.2f} seconds")

            return output_path
    else:
        # Fallback: Extract from model joints
        print("\nCould not access trajectory directly. Extracting from model...")
        model = env.model

        arm_joint_names = []
        arm_joint_ids = []
        for i in range(model.njnt):
            name = model.joint(i).name.lower()
            if any(x in name for x in ['shoulder', 'elbow', 'wrist', 'arm']):
                arm_joint_names.append(model.joint(i).name)
                arm_joint_ids.append(i)
                print(f"  Model arm joint [{i}]: {model.joint(i).name}")

        if arm_joint_ids:
            os.makedirs(output_dir, exist_ok=True)
            output_path = os.path.join(output_dir, f"lafan1_{dataset_name}_arm_info.npz")
            np.savez(output_path, joint_names=arm_joint_names, joint_ids=arm_joint_ids)
            print(f"\nSaved arm model info to: {output_path}")
            return output_path

    return None


def visualize_dataset(dataset_name="walk1_subject1"):
    """
    Visualize the LAFAN1 dataset to understand the motion.
    """
    print(f"\nVisualizing LAFAN1 dataset: {dataset_name}")
    print("(This will open a viewer window. Close it to continue.)")

    env = ImitationFactory.make(
        "UnitreeH1",
        lafan1_dataset_conf=LAFAN1DatasetConf([dataset_name]),
        n_substeps=20
    )

    # Play the trajectory
    env.play_trajectory(n_episodes=1, n_steps_per_episode=500, render=True)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Extract LAFAN1 arm data")
    parser.add_argument("--visualize", action="store_true", help="Visualize the dataset")
    parser.add_argument("--dataset", default="walk1_subject1", help="LAFAN1 dataset name")
    parser.add_argument("--output", default="data", help="Output directory")

    args = parser.parse_args()

    if args.visualize:
        visualize_dataset(args.dataset)
    else:
        extract_lafan1_arm_data(args.dataset, args.output)
