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
            # --- Full Sphere Smooth Control ---
            # 1. Extract pointing direction from quaternion (avoids Euler gimbal lock)
            forward = orientation.apply([1, 0, 0])  # Sensor X-axis = pointing direction

            # 2. Convert to spherical coordinates
            # Pan (azimuth): rotation around vertical axis
            raw_pan = np.arctan2(forward[1], forward[0])
            # Lift (elevation): angle from horizontal plane
            raw_lift = np.arcsin(np.clip(forward[2], -1.0, 1.0))

            # 3. Apply scale
            scaled_pan = raw_pan * self.scale
            scaled_lift = raw_lift * self.scale

            # 4. EMA (Exponential Moving Average) smoothing for continuity
            # This prevents sudden jumps, especially when Pan wraps around ±π
            if not hasattr(self, '_smooth_pan'):
                self._smooth_pan = scaled_pan
                self._smooth_lift = scaled_lift

            # Handle Pan wrap-around: if delta > π, it's a wrap
            delta_pan = scaled_pan - self._smooth_pan
            if delta_pan > np.pi:
                delta_pan -= 2 * np.pi
            elif delta_pan < -np.pi:
                delta_pan += 2 * np.pi

            # EMA filter (alpha controls smoothness: lower = smoother)
            ALPHA = 0.3  # 0.3 = fairly responsive, 0.1 = very smooth
            self._smooth_pan += ALPHA * delta_pan
            self._smooth_lift += ALPHA * (scaled_lift - self._smooth_lift)

            # Normalize Pan to [-π, π]
            if self._smooth_pan > np.pi:
                self._smooth_pan -= 2 * np.pi
            elif self._smooth_pan < -np.pi:
                self._smooth_pan += 2 * np.pi

            # 5. Apply to robot
            self.target_pan = self._smooth_pan
            self.target_lift = self._smooth_lift  # No clamping - full sphere

            self.data.qpos[0] = self.target_pan
            self.data.qpos[1] = self.target_lift

            # Lock forearm and wrist joints
            self.data.qpos[2] = -1.57
            self.data.qpos[3] = -1.57
            self.data.qpos[4] = -1.57
            self.data.qpos[5] = 0.0

            # For UI display
            yaw = raw_pan
            pitch = raw_lift

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
