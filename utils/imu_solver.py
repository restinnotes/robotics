"""
IMU Solver: 将两个 IMU 的四元数解算为 UR3e 的关节角度
核心数学模块 - Phase 2
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

class IMUSolver:
    def __init__(self):
        # 初始校准姿态 (T-Pose 或当前姿态)
        # 存储为 Rotation 对象
        self.calib_upper = R.identity()
        self.calib_fore = R.identity()

        # 初始关节角度偏移
        self.offset_joints = np.zeros(6)

        self.calibrated = False

    def calibrate(self, q_upper, q_fore, offset_joints=None):
        """
        校准：记录当前的 IMU 姿态和对应的关节角度作为“零位”
        offset_joints: [pan, lift, elbow, wrist1, wrist2, wrist3] (弧度) 当前时刻的真实关节角
        """
        self.calib_upper = R.from_quat(q_upper).inv()
        self.calib_fore = R.from_quat(q_fore).inv()

        # 记录初始关节角度作为 Offset
        if offset_joints is not None:
            self.offset_joints = np.array(offset_joints)
        else:
            self.offset_joints = np.zeros(6)

        self.calibrated = True
        print("IMU Solver Calibrated with offset!")

    def solve(self, q_upper, q_fore):
        """
        解算关节角度
        Input:
            q_upper: 大臂 IMU 四元数 [x, y, z, w]
            q_fore:  前臂 IMU 四元数 [x, y, z, w]
        Output:
            ur3e_joints: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        """
        if not self.calibrated:
            # 如果未校准，默认以上电时刻为零位
            self.calibrate(q_upper, q_fore)

        # 1. 计算相对于校准姿态的相对旋转
        r_upper = R.from_quat(q_upper) * self.calib_upper
        r_fore = R.from_quat(q_fore) * self.calib_fore

        # 2. 将旋转转换为欧拉角 (ZYX 顺序通常对应 Yaw-Pitch-Roll)
        euler_upper = r_upper.as_euler('zyx', degrees=False) # [z, y, x]
        euler_fore = r_fore.as_euler('zyx', degrees=False)   # [z, y, x]

        # 3. 映射到 UR3e 关节 (基于 Offset 叠加增量)
        ur_joints = self.offset_joints.copy()

        # 增量映射
        # Shoulder Pan: 叠加 Yaw (Z) 增量
        ur_joints[0] += euler_upper[0]

        # Shoulder Lift: 叠加 Pitch (Y) 增量
        ur_joints[1] += euler_upper[1]

        # Elbow:
        # 计算前臂相对于大臂的旋转增量
        r_elbow_rel = r_upper.inv() * r_fore
        euler_elbow = r_elbow_rel.as_euler('zyx', degrees=False)

        # 叠加 Pitch (Y) 增量
        # 注意符号：根据实测，如果不反转符号，误差为 0.5；反转后误差为 0.9。
        # 说明正方向是对的，只是缺少 Offset。
        # 现在有了 Offset，直接叠加即可。
        ur_joints[2] += euler_elbow[1]

        # Wrist 1, 2, 3: 保持初始 Offset 不变 (因为没有手腕传感器)
        # 如果需要，可以将 Wrist1 设为固定值或某种补偿逻辑
        # 目前保持 offset 不变，即 ur_joints[3] = offset[3]

        return ur_joints
