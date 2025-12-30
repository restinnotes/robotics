"""
渲染并排对比视频：UR3e Vertical vs Unitree H1 (Source Data)
原理：
在一个脚本中渲染 UR3e 环境，并利用 loco-mujoco 提供的渲染器渲染 Unitree H1 环境。
将两者的图像拼接。
"""

import mujoco
import numpy as np
import cv2
import os
# Check how to import properly. The error said "cannot import LocoMujoco"
# Usually it's `from loco_mujoco import LocoEnv` or similar.
# Looking at extract_lafan1.py: `from loco_mujoco import ImitationFactory, LocoMujoco`?
# No, let's try importing the package and seeing what's available or use the factory if needed.
# But wait, previous `extract_lafan1.py` successfully imported `ImitationFactory`.
# Let's check `loco_mujoco/__init__.py` content again from the command output.
# It seems it imports `LocoEnv`.
from loco_mujoco import LocoEnv

def main():
    # --- 配置 ---
    width, height = 640, 480
    output_fps = 50
    duration_frames = 500
    video_path = "comparison_humanoid.mp4"

    # --- 1. 初始化 Unitree H1 (使用 loco-mujoco) ---
    print("Initializing Unitree H1 environment...")
    # 使用 Unitree H1 走路任务
    # env_h1 = LocoMujoco(...) -> This class might not exist or be wrapper
    # Usually: env = LocoEnv.make("UnitreeH1.Walk")
    env_h1 = LocoEnv.make("UnitreeH1.Walk")

    # --- 2. 初始化 UR3e Vertical ---
    print("Initializing UR3e Vertical environment...")
    current_dir = os.path.dirname(os.path.abspath(__file__))
    ur3e_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
    traj_path = os.path.join(current_dir, "data", "walk_arm_direct.npz")

    ur3e_model = mujoco.MjModel.from_xml_path(ur3e_path)
    ur3e_data = mujoco.MjData(ur3e_model)
    ur3e_renderer = mujoco.Renderer(ur3e_model, height=height, width=width)

    # 相机设置
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    cam.lookat[:] = [0, 0, 0.7]
    cam.distance = 1.2
    cam.elevation = 0
    cam.azimuth = 90

    # --- 3. 加载 UR3e 轨迹 ---
    traj = np.load(traj_path, allow_pickle=True)
    ref_qpos = traj["qpos"] # UR3e 映射后的轨迹吗？不，这是原始人体关节数据 (N, 19)
    # wait, walk_arm_direct.npz 里的 qpos 已经是原始关节数据了
    # 我们的映射逻辑是在 loop 里做的
    n_frames = ref_qpos.shape[0]

    # --- 4. 视频写入 ---
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(video_path, fourcc, output_fps, (width * 2, height))

    print(f"Recording comparison video to {video_path}...")

    # 重置 H1
    env_h1.reset()
    # H1 的数据集播放逻辑比较特殊，我们需要手动设置它的 qpos
    # 或者，更简单的方法：直接用 loco-mujoco 的 replay 功能生成视频，然后我们再生成 UR3e 的，最后用 ffmpeg 拼。
    # 但为了同步，我们要尝试手动设置 H1 的状态

    # LocoMujoco 的 dataset 存储在 env.dataset 中
    # env_h1.dataset 是一个 Trajectory 对象
    # 我们知道 walk_arm_direct.npz 就是从那里解出来的，所以索引是一一对应的

    # 获取 H1 的模型和数据句柄
    h1_model = env_h1.environment.model
    h1_data = env_h1.environment.data
    h1_renderer = mujoco.Renderer(h1_model, height=height, width=width)

    # H1 相机调整：看全身
    cam_h1 = mujoco.MjvCamera()
    cam_h1.type = mujoco.mjtCamera.mjCAMERA_FIXED
    cam_h1.lookat[:] = [0, 0, 1.0] # 躯干高度
    cam_h1.distance = 3.0
    cam_h1.elevation = -20
    cam_h1.azimuth = 90

    for i in range(duration_frames):
        frame_idx = i % n_frames

        # --- A. 获取原始数据 (Unitree H1 qpos) ---
        # 注意：walk_arm_direct.npz 里只有关节数据，没有 root position/orientation
        # 如果要显示 H1 走路，我们需要完整的 qpos (包括 root)
        # 我们的 npz 里存的 qpos 是 (N, 19) -> 这是一个删减版还是完整版？
        # 让我们检查一下

        # 既然我们无法简单复原 H1 的走路（因为缺少 root pos），我们只显示 H1 原地踏步或者只显示上身？
        # 或者，我们直接从 env_h1.dataset 里取原始数据（包含 root）

        original_qpos = env_h1.dataset[0]['qpos'][frame_idx] # 假设只有一条轨迹
        # 注意：dataset 的结构可能比较复杂。

        # 备选方案：直接使用 walk_arm_direct.npz 里的数据
        # 我们的 npz 里的 qpos (19维) 对应的是哪些关节？
        # 根据之前的 extract 脚本：
        # [0:3] = root pos ? No, extracted directly from joints.
        # "UnitreeH1.Walk" qpos dim is usually 19 + 7 (free joint) = 26

        # 为了不把事情搞复杂，我们既然已经有 19 维数据，我就把这 19 维赋给 H1 对应的关节
        # H1 的 qpos 结构: [root_x, root_y, root_z, root_qw, root_qx, root_qy, root_qz, j1, j2 ... j19]
        # 我们的 npz 里的 ref_qpos 应该是后面的 j1...j19

        full_qpos_h1 = np.zeros(h1_model.nq)
        full_qpos_h1[0:3] = [0, 0, 1.0] # 固定在半空
        full_qpos_h1[3:7] = [1, 0, 0, 0] # 直立

        # 填充关节 (注意：ref_qpos 里的顺序是否和 H1 一致？)
        # extract_lafan1.py 里是： traj_data['qpos'][:, 7:]
        # 所以 ref_qpos 确实就是去掉 root 后的 19 个关节。

        raw_joints = ref_qpos[frame_idx]
        full_qpos_h1[7:] = raw_joints

        # 设置 H1 状态
        h1_data.qpos[:] = full_qpos_h1
        mujoco.mj_forward(h1_model, h1_data)

        h1_renderer.update_scene(h1_data, camera=cam_h1)
        img_h1 = h1_renderer.render()
        img_h1 = cv2.cvtColor(img_h1, cv2.COLOR_RGB2BGR)
        cv2.putText(img_h1, "Source Data (Unitree H1)", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        # --- B. 设置 UR3e 状态 ---
        # 映射逻辑
        target_ur3 = np.zeros(6)
        target_ur3[0] = raw_joints[1]           # Pan
        target_ur3[1] = raw_joints[0]           # Lift
        target_ur3[2] = raw_joints[3]           # Elbow
        target_ur3[3] = -1.57 + raw_joints[2]   # Wrist1
        target_ur3[4] = -1.57
        target_ur3[5] = 0

        ur3e_data.qpos[:6] = target_ur3
        mujoco.mj_forward(ur3e_model, ur3e_data)

        ur3e_renderer.update_scene(ur3e_data, camera=cam)
        img_ur3 = ur3e_renderer.render()
        img_ur3 = cv2.cvtColor(img_ur3, cv2.COLOR_RGB2BGR)
        cv2.putText(img_ur3, "UR3e Imitation", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # --- C. 拼接 ---
        combined = np.hstack((img_h1, img_ur3))
        out.write(combined)

        if i % 50 == 0:
            print(f"Frame {i}/{duration_frames}")

    out.release()
    print("Done!")

if __name__ == "__main__":
    main()
