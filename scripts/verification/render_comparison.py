"""
Side-by-side comparison: Humanoid walk vs UR3e imitation
Updated version with correct trajectory mapping and frame rate.
"""

import mujoco
import numpy as np
import os
import cv2

# Setup paths
current_dir = os.path.dirname(os.path.abspath(__file__))
ur3e_xml = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e.xml")

# Load UR3e model
ur3e_model = mujoco.MjModel.from_xml_path(ur3e_xml)
ur3e_data = mujoco.MjData(ur3e_model)

# Load reference data (directly extracted from trajectory)
ref_path = os.path.join(current_dir, "data", "walk_arm_direct.npz")
ref = np.load(ref_path, allow_pickle=True)
ref_qpos = ref["qpos"]
ref_freq = float(ref["frequency"])  # 50 Hz

print(f"Loaded trajectory: {ref_qpos.shape[0]} frames at {ref_freq} Hz")

# Home position
home_qpos = np.array([0, 0, 0, -1.57, -1.57, 0], dtype=np.float32)

# Renderer for UR3e
width, height = 640, 480
renderer = mujoco.Renderer(ur3e_model, width=width, height=height)

# Number of frames to render (match humanoid video length)
n_frames = 500  # 10 seconds at 50 Hz

print(f"Recording {n_frames} frames...")

frames = []
for frame_idx in range(min(n_frames, len(ref_qpos))):
    # Get reference pose - direct 1:1 mapping
    target = home_qpos.copy()

    # UR3e joint 0 (shoulder_pan): l_arm_shx (外展)
    target[0] = ref_qpos[frame_idx, 1]

    # UR3e joint 1 (shoulder_lift): l_arm_shy (前后摆动)
    target[1] = ref_qpos[frame_idx, 0]

    # UR3e joint 2 (elbow): left_elbow
    target[2] = ref_qpos[frame_idx, 3]

    # UR3e joints 3-5 (wrist)
    target[3] = -1.57 + ref_qpos[frame_idx, 2]
    target[4] = -1.57
    target[5] = 0

    # Apply to model
    ur3e_data.qpos[:] = target
    mujoco.mj_forward(ur3e_model, ur3e_data)

    # Render
    renderer.update_scene(ur3e_data)
    frame = renderer.render()
    frames.append(frame)

    if frame_idx % 100 == 0:
        print(f"  Frame {frame_idx}...")

# Save as video at 50 fps to match humanoid
print("Saving video...")
output_video = os.path.join(current_dir, "ur3e_imitation_v2.mp4")
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(output_video, fourcc, int(ref_freq), (width, height))  # 50 fps

for frame in frames:
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    out.write(frame_bgr)

out.release()
print(f"Video saved to: {output_video}")
print(f"Duration: {n_frames / ref_freq:.2f} seconds at {ref_freq} fps")
