"""
UR3e 竖直版模仿控制
使用竖直安装的 UR3e 模型，更接近人类手臂姿态
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# 加载竖直版 UR3e 模型
current_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Home 姿态
home_qpos = np.array([0, 0, 0, -1.57, -1.57, 0], dtype=np.float32)

# 加载参考轨迹
reference_data = None
ref_path = os.path.join(current_dir, "data", "walk_arm_direct.npz")

if os.path.exists(ref_path):
    print(f"加载参考轨迹: {ref_path}")
    ref = np.load(ref_path, allow_pickle=True)
    reference_data = {
        "qpos": ref["qpos"],
        "frequency": float(ref["frequency"]),
        "joint_names": ref["joint_names"].tolist() if isinstance(ref["joint_names"], np.ndarray) else ref["joint_names"]
    }
    print(f"  轨迹长度: {reference_data['qpos'].shape[0]} 帧")
    print(f"  采样频率: {reference_data['frequency']} Hz")
else:
    print("未找到参考轨迹，使用 sin-wave 模式")


def get_reference_pose(t, reference_data):
    """获取参考姿态"""
    target = home_qpos.copy()

    if reference_data is not None:
        ref_qpos = reference_data["qpos"]
        ref_freq = reference_data["frequency"]
        frame_idx = int((t * ref_freq) % ref_qpos.shape[0])

        # 1:1 映射
        target[0] = ref_qpos[frame_idx, 1]  # l_arm_shx -> shoulder_pan
        target[1] = ref_qpos[frame_idx, 0]  # l_arm_shy -> shoulder_lift
        target[2] = ref_qpos[frame_idx, 3]  # left_elbow -> elbow
        target[3] = -1.57 + ref_qpos[frame_idx, 2]  # l_arm_shz -> wrist_1
        target[4] = -1.57
        target[5] = 0
    else:
        # Sin-wave fallback
        s = np.sin(2 * np.pi * 0.5 * t)
        target[1] = s * 0.5
        target[2] = ((s + 1) / 2) ** 2 * 1.0
        target[3] = -1.57
        target[4] = -1.57
        target[5] = 0

    return target


print("\n启动竖直版 UR3e 摆臂模仿...")
print("(按 Esc 或关闭窗口退出)")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        t = data.time

        # 获取目标姿态
        target = get_reference_pose(t, reference_data)

        # 应用控制
        data.ctrl[:] = target

        # 仿真步进
        mujoco.mj_step(model, data)

        # 渲染
        viewer.sync()

        time.sleep(0.002)
