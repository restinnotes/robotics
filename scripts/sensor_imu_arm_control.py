"""
Arm Control Logic with Recording
================================
Encapsulates the arm control logic, IMU connection, and recording into a class
for easier integration with UI.
"""

import time
import numpy as np
import mujoco
import mujoco.viewer
import sys
import os
import platform

# --- Linux OpenGL/GLX Fix ---
if platform.system() == "Linux":
    lib_path = "/usr/lib/x86_64-linux-gnu/libstdc++.so.6"
    if os.path.exists(lib_path):
        os.environ["LD_PRELOAD"] = lib_path
# ----------------------------

from scipy.spatial.transform import Rotation as R

# Ensure project root is in path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

from utils.bhy2cli_receiver import BHy2CLIReceiver
from scripts.verification.motion_recorder import MotionRecorder

class ArmImuController:
    def __init__(self, scale=1.5, enable_drift_compensation=False, no_viewer=False):
        self.scale = scale
        self.enable_drift_compensation = enable_drift_compensation
        self.no_viewer = no_viewer

        self.model = None
        self.data = None
        self.viewer = None
        self.receiver = None
        self.recorder = None
        self.is_running = False

        # Initial positions
        self.target_pan = 0.0
        self.target_lift = -1.57

        # Load Model
        self._load_model()

        # Initialize Recorder
        self.recorder = MotionRecorder(output_dir=os.path.join(project_root, "data", "recordings"))

    def _load_model(self):
        possible_paths = [
            os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e_vertical.xml"),
            os.path.join(project_root, "assets", "universal_robots_ur3e", "ur3e.xml"),
        ]
        model_path = None
        for path in possible_paths:
            if os.path.exists(path):
                model_path = path
                break

        if not model_path:
            raise FileNotFoundError("Could not find UR3e model file.")

        print(f"Loading model: {model_path}")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Set initial pose
        initial_qpos = np.array([0, -1.57, -1.57, -1.57, -1.57, 0])
        self.data.qpos[:] = initial_qpos
        mujoco.mj_step(self.model, self.data)

    def connect_sensor(self):
        print("Connecting to sensor (ID 37)...")
        self.receiver = BHy2CLIReceiver(
            sensor_id=37,
            sample_rate=50,
            enable_drift_compensation=self.enable_drift_compensation
        )

        # Note: BHy2CLIReceiver handles connection
        if not self.receiver.connect(perform_gyro_foc=True):
            print("Failed to connect to sensor!")
            return False

        print("Sensor connected.")
        return True

    def calibrate_sensor(self):
        if self.receiver:
            self.receiver.calibrate()

    def start_recording(self):
        self.recorder.start()

    def stop_recording(self):
        stats = self.recorder.stop()
        self.recorder.save() # Auto save with timestamp
        return stats

    def step(self):
        """
        Execute one control step.
        Returns current sensor data stats for UI.
        """
        if not self.receiver:
            return None

        # 1. Get Orientation
        orientation = self.receiver.get_orientation()
        yaw, pitch, roll = 0, 0, 0

        if orientation:
            # 使用矢量投影法代替欧拉角，解决 Gimbal Lock 和翻转问题

            # 1. 获取传感器前向矢量 (假设 X 轴为指向)
            v = orientation.apply([1, 0, 0])

            # 2. 计算球坐标候选解
            # 候选解 1: 标准解，Lift 在 [-90, 90]
            # Pan = atan2(y, x), Lift = asin(z)
            p1 = np.arctan2(v[1], v[0])
            l1 = np.arcsin(np.clip(v[2], -1.0, 1.0))

            # 候选解 2: "翻转"解，Lift 在 [90, 270] (或 [-270, -90])
            # 对应于 Pitch 越过 90 度的情况
            p2 = p1 + np.pi
            l2 = np.pi - l1

            # 初始化状态
            if not hasattr(self, '_last_raw_pan'):
                self._last_raw_pan = p1
                self._last_raw_lift = l1
                self._accum_pan = 0.0
                self._accum_lift = -1.57 # 初始 Lift Down

            # 3. 连续性解算 (Unwrap & Candidate Selection)
            # 对两个候选 Pan 进行解绕，使其接近上一帧
            p1_u = np.unwrap([self._last_raw_pan, p1])[1]
            p2_u = np.unwrap([self._last_raw_pan, p2])[1]

            # 计算主要候选项与上一帧的距离 (欧氏距离)
            dist1 = (p1_u - self._last_raw_pan)**2 + (l1 - self._last_raw_lift)**2
            dist2 = (p2_u - self._last_raw_pan)**2 + (l2 - self._last_raw_lift)**2

            # 选择距离最小的解，保证运动平滑连续
            if dist1 < dist2:
                chosen_p = p1_u
                chosen_l = l1
            else:
                chosen_p = p2_u
                chosen_l = l2

            # 4. Calculate delta and apply sensitivity
            delta_pan = chosen_p - self._last_raw_pan
            delta_lift = chosen_l - self._last_raw_lift

            # --- Singularity Handling (Gimbal Lock Prevention) ---
            # When lift approaches ±90°, pan becomes degenerate (undefined).
            # Freeze pan updates in singularity zone to prevent wild swings.
            SINGULARITY_THRESHOLD = 1.4  # ~80 degrees, close to ±90
            in_singularity = abs(chosen_l) > SINGULARITY_THRESHOLD

            if in_singularity:
                # Only update lift, keep pan frozen
                delta_pan = 0.0
            # ----------------------------------

            self._last_raw_pan = chosen_p
            self._last_raw_lift = chosen_l

            # 累积控制量
            # 初始状态下的 _accum_pan 对应 Robot 0 (Forward)
            # 初始状态下的 _accum_lift 对应 Robot -1.57 (Down)
            # 如果一开始 sensor 是 Forward (l=0), delta=0.
            # 此时 target_lift 应该是 -1.57 + 0? 否。
            # 如果 sensor 是 Forward，Robot 应该是 Horizontal (0)。
            # 我们逻辑：Sensor Forward (Identity) -> v=[1,0,0] -> p=0, l=0.
            # 此时我们希望 Robot Horizontal。
            # 所以 _accum_lift 初始值设为 0 比较好？
            # 不，_accum_lift 代表 "目标角度"。
            # 我们的逻辑：Align Sensor Frame with Robot Frame.
            # Sensor Forward = Robot Forward (Horizontal).
            # Sensor Down = Robot Down (-1.57).
            # 所以 _accum_lift 初始应该跟随 Sensor 的初始状态相对值。

            # 修正：我们不仅需要积攒 Delta，还需要一个 Base Offset。
            # 简化方案：直接使用 Scale 后的 Delta 累加
            if not hasattr(self, '_target_pan'):
                self._target_pan = 0.0
                # 如果初始 Sensor 是 Down (l=-1.57)，我们希望 Robot 是 Down (-1.57)
                # 如果初始 Sensor 是 Forward (l=0)，我们希望 Robot 是 Forward (0)
                # 所以 Offset = 0
                self._target_lift = chosen_l # 初始直接同步绝对值

                # 特殊处理：如果是 Down 启动，ensure it matches
                # 我们的 chosen_l 是绝对角度。
                # 如果用户希望 scale > 1，则绝对角度会放大。
                # 我们采用 "绝对跟随 + 增量放大" 混合？
                # 不，标准做法是：初始绝对对齐，之后只认增量。
                self._target_pan = chosen_p
                self._target_lift = chosen_l
            else:
                self._target_pan += delta_pan * self.scale
                self._target_lift += delta_lift * self.scale

            self.target_pan = self._target_pan
            self.target_lift = self._target_lift

            # 3. Apply Control
            self.data.qpos[0] = self.target_pan
            self.data.qpos[1] = self.target_lift  # 移除限制，允许全球面运动

            # 锁定小臂和手腕（防止受重力/惯性摆动）
            # Initial pose: [0, -1.57, -1.57, -1.57, -1.57, 0]
            # 关节2 (Elbow), 3 (Wrist 1), 4 (Wrist 2) 固定在 -1.57 (-90度)
            # 关节5 (Wrist 3) 固定在 0
            self.data.qpos[2] = -1.57
            self.data.qpos[3] = -1.57
            self.data.qpos[4] = -1.57
            self.data.qpos[5] = 0.0

            # 用于 UI 显示
            yaw = chosen_p
            pitch = chosen_l

            # 4. Record if enabled
            if self.recorder and self.recorder.is_recording:
                # We record the mapping source (IMU) and the result (qpos)
                # Currently MotionRecorder expects specific format, let's adapt
                # We'll save the raw quat as imu_upper
                quat = orientation.as_quat() # x,y,z,w
                self.recorder.record_frame(
                    qpos=self.data.qpos,
                    qvel=self.data.qvel, # Not strictly controlled but good for checking
                    imu_upper=quat
                )

        # 5. Physics Step
        mujoco.mj_step(self.model, self.data)

        # 6. Viewer Sync (with error handling)
        if self.viewer:
            try:
                if self.viewer.is_running():
                    self.viewer.sync()
            except Exception as e:
                # Viewer may have been closed unexpectedly
                print(f"Viewer sync error: {e}")
                self.viewer = None

        return {
            "yaw": yaw,
            "pitch": pitch,
            "roll": roll,
            "pan": self.target_pan,
            "lift": self.target_lift,
            "is_recording": self.recorder.is_recording,
            "recording_frames": len(self.recorder.data["timestamps"])
        }

    def launch_viewer(self):
        if not self.no_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.lookat[:] = [0, 0, 0.7]
            self.viewer.cam.distance = 1.2
            self.viewer.cam.azimuth = 90

    def close(self):
        if self.viewer:
            try:
                if self.viewer.is_running():
                    self.viewer.close()
            except Exception as e:
                print(f"Error closing viewer: {e}")
            finally:
                self.viewer = None
        if self.receiver:
            self.receiver.disconnect()
        if self.recorder and self.recorder.is_recording:
            self.stop_recording()

# CLI Main for testing
def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-viewer", action="store_true")
    args = parser.parse_args()

    controller = ArmImuController(no_viewer=args.no_viewer)

    if not controller.connect_sensor():
        return

    print("Calibrating in 2 seconds...")
    time.sleep(2)
    controller.calibrate_sensor()
    print("Go!")

    controller.launch_viewer()

    try:
        while True:
            controller.step()
            time.sleep(0.01)
            if controller.viewer and not controller.viewer.is_running():
                break
    except KeyboardInterrupt:
        pass
    finally:
        controller.close()

if __name__ == "__main__":
    main()
