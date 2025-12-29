"""
Direct control with LAFAN1-based human arm motion imitation.

This script replaces the sin-wave based arm motion with real human walking
arm swing data extracted from the LAFAN1 dataset via loco-mujoco.

Simplified version: Uses joint position tracking from reference trajectory.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# 1. 加载 UR3e 模型
current_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(current_dir, "assets", "universal_robots_ur3e", "ur3e.xml")
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# 2. 定义 Home 姿态 (UR3e 的默认姿态)
home_qpos = np.array([0, 0, 0, -1.57, -1.57, 0], dtype=np.float32)

# 3. 尝试加载 LAFAN1 参考数据
reference_data = None
reference_path = os.path.join(current_dir, "data", "walk_arm_direct.npz")

if os.path.exists(reference_path):
    print(f"加载参考轨迹: {reference_path}")
    ref = np.load(reference_path, allow_pickle=True)
    reference_data = {
        "qpos": ref["qpos"],
        "frequency": float(ref["frequency"]),
        "joint_names": ref["joint_names"].tolist() if isinstance(ref["joint_names"], np.ndarray) else ref["joint_names"]
    }
    print(f"  轨迹长度: {reference_data['qpos'].shape[0]} 帧")
    print(f"  采样频率: {reference_data['frequency']} Hz")
    print(f"  关节: {reference_data['joint_names']}")
else:
    print(f"警告: 未找到参考轨迹 {reference_path}")
    print("将使用默认的 sin-wave 模式。")
    print("请先运行: python data/extract_lafan1.py")

# 4. 数据已经是干净的轨迹数据，不需要平滑处理
if reference_data is not None:
    print(f"轨迹数据就绪，频率: {reference_data['frequency']} Hz")


def get_reference_pose(t, reference_data, freq=0.8, amp_shoulder=1.0, amp_elbow=1.4):
    """
    获取参考姿态：如果有 LAFAN1 数据则使用，否则使用 sin-wave。

    LAFAN1 左臂关节 (indices 0-3):
      0: l_arm_shy - 肩膀 Y 轴 (前后摆动) -> UR3e shoulder_lift
      1: l_arm_shx - 肩膀 X 轴 (外展)    -> UR3e 可忽略或混合
      2: l_arm_shz - 肩膀 Z 轴 (旋转)    -> UR3e wrist 旋转
      3: left_elbow - 肘弯曲              -> UR3e elbow

    返回 UR3e 的 6 个关节目标角度。
    """
    target = home_qpos.copy()

    if reference_data is not None:
        ref_qpos = reference_data["qpos"]
        ref_freq = reference_data["frequency"]

        # 计算当前帧 (循环播放)
        frame_idx = int((t * ref_freq) % ref_qpos.shape[0])

        # 使用左臂数据 (indices 0-3) 直接映射到 UR3e
        # 不需要缩放，因为人形手臂关节角度范围与 UR3e 相近

        # UR3e joint 0 (shoulder_pan): 使用 l_arm_shx (外展)
        target[0] = ref_qpos[frame_idx, 1]

        # UR3e joint 1 (shoulder_lift): 使用 l_arm_shy (前后摆动)
        target[1] = ref_qpos[frame_idx, 0]

        # UR3e joint 2 (elbow): 使用 left_elbow
        target[2] = ref_qpos[frame_idx, 3]

        # UR3e joints 3-5 (wrist): 使用 l_arm_shz + 固定偏移
        target[3] = -1.57 + ref_qpos[frame_idx, 2]
        target[4] = -1.57
        target[5] = 0
    else:
        # Fallback: 使用原来的 sin-wave 模式
        s = np.sin(2 * np.pi * freq * t)
        target[1] = amp_shoulder * s
        elbow_normalized = ((s + 1) / 2) ** 2
        target[2] = amp_elbow * elbow_normalized
        target[3] = -1.57
        target[4] = -1.57
        target[5] = 0

    return target


# 4. 读取 IMU 传感器 (如果可用)
def get_imu_data(data):
    """读取 IMU 传感器数据"""
    imu_data = {}
    try:
        imu_data['wrist3_accel'] = data.sensor('wrist3_accel').data.copy()
        imu_data['wrist3_gyro'] = data.sensor('wrist3_gyro').data.copy()
        imu_data['forearm_accel'] = data.sensor('forearm_accel').data.copy()
        imu_data['forearm_gyro'] = data.sensor('forearm_gyro').data.copy()
    except KeyError as e:
        pass  # IMU sensors not available
    return imu_data


# 5. 主循环
print("\n启动 LAFAN1 imitation 模式：")
if reference_data:
    print("- 使用 LAFAN1 人类摆臂数据")
else:
    print("- 使用 sin-wave fallback")
print("- IMU 传感器已启用 (wrist3, forearm)")

with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()
    last_imu_print = 0

    while viewer.is_running():
        t = data.time

        # 获取参考姿态
        target = get_reference_pose(t, reference_data)

        # 控制信号
        data.ctrl[:] = target

        # 物理步进
        mujoco.mj_step(model, data)

        # 每秒打印一次 IMU 数据
        if t - last_imu_print >= 1.0:
            imu = get_imu_data(data)
            if imu:
                print(f"t={t:.1f}s | Wrist Accel: {imu.get('wrist3_accel', 'N/A')} | "
                      f"Forearm Accel: {imu.get('forearm_accel', 'N/A')}")
            last_imu_print = t

        # 渲染
        viewer.sync()

        # 控制帧率
        time.sleep(0.002)
