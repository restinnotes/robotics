"""
轨迹回放器：从保存的数据文件驱动 UR3e 仿真
这是整个 pipeline 的基础模块
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os

class TrajectoryPlayer:
    """轨迹回放控制器"""

    def __init__(self, model_path, trajectory_path):
        # 加载模型
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # 加载轨迹
        traj = np.load(trajectory_path, allow_pickle=True)
        self.qpos = traj["qpos"]
        self.frequency = float(traj["frequency"])
        self.joint_names = traj["joint_names"].tolist() if isinstance(traj["joint_names"], np.ndarray) else traj["joint_names"]
        self.n_frames = self.qpos.shape[0]

        print(f"轨迹加载完成: {self.n_frames} 帧, {self.frequency} Hz")
        print(f"关节: {self.joint_names}")

        # Home 姿态
        self.home_qpos = np.array([0, 0, 0, -1.57, -1.57, 0], dtype=np.float32)

    def map_to_ur3e(self, frame_idx):
        """将人形机器人关节映射到 UR3e"""
        target = self.home_qpos.copy()
        ref = self.qpos[frame_idx]

        # 1:1 映射 (基于之前验证的逻辑)
        target[0] = ref[1]              # l_arm_shx -> shoulder_pan
        target[1] = ref[0]              # l_arm_shy -> shoulder_lift
        target[2] = ref[3]              # left_elbow -> elbow
        target[3] = -1.57 + ref[2]      # l_arm_shz -> wrist_1
        target[4] = -1.57
        target[5] = 0

        return target

    def play(self, loop=True, speed=1.0):
        """播放轨迹"""
        print(f"\n开始回放 (速度: {speed}x, 循环: {loop})")
        print("按 ESC 或关闭窗口退出")

        # DEBUG: 打印原始数据范围
        print(f"原始数据范围:")
        for i, name in enumerate(self.joint_names):
            min_val = np.min(self.qpos[:, i])
            max_val = np.max(self.qpos[:, i])
            print(f"  {name}: [{min_val:.4f}, {max_val:.4f}] Delta: {max_val - min_val:.4f}")

        # DEBUG: 打印映射后的目标范围
        print(f"\n映射后目标范围 (UR3e):")
        start_frame = self.map_to_ur3e(0)
        print(f"  Frame 0 Target: {start_frame}")

        # 相机角度配置
        camera_angles = [
            {"name": "正面", "azimuth": 90, "elevation": 0, "distance": 1.2},
            {"name": "侧面", "azimuth": 0, "elevation": 0, "distance": 1.2},
            {"name": "俯视", "azimuth": 90, "elevation": -45, "distance": 1.5},
        ]
        cam_switch_interval = 250 # 5 seconds at 50Hz (approx, simulation time based)

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # 初始相机设置
            viewer.cam.lookat[:] = [0, 0, 0.7]
            viewer.cam.distance = 1.2
            viewer.cam.elevation = 0
            viewer.cam.azimuth = 90

            step_count = 0

            while viewer.is_running():
                # 自动切换相机
                if step_count % cam_switch_interval == 0:
                    idx = (step_count // cam_switch_interval) % 3
                    cam = camera_angles[idx]
                    viewer.cam.azimuth = cam["azimuth"]
                    viewer.cam.elevation = cam["elevation"]
                    viewer.cam.distance = cam["distance"]
                    print(f"Switching to [{cam['name']}] view")

                # 用仿真时间来索引帧 (与 direct_control_vertical 一致)
                t = self.data.time
                frame_idx = int((t * self.frequency) % self.n_frames)

                # 获取目标姿态
                target = self.map_to_ur3e(frame_idx)

                # 应用控制
                self.data.ctrl[:] = target

                # 仿真步进
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

                # 与 direct_control_vertical 一致的帧率
                time.sleep(0.002)
                step_count += 1


def main():
    # 路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(current_dir))
    model_path = os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e_vertical.xml")
    traj_path = os.path.join(project_root, "data", "walk_arm_direct.npz")

    # 创建播放器并运行
    player = TrajectoryPlayer(model_path, traj_path)
    player.play(loop=True, speed=1.0)


if __name__ == "__main__":
    main()
