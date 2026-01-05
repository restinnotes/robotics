"""
Arm Trajectory Player
======================
Generic player for recorded .npz trajectories.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import sys

# Ensure project root is in path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

class ArmTrajectoryPlayer:
    def __init__(self, filename=None, loop=True, speed=1.0, no_viewer=False):
        self.filename = filename
        self.loop = loop
        self.speed = speed
        self.no_viewer = no_viewer

        self.model = None
        self.data = None
        self.viewer = None

        self.qpos_traj = None
        self.timestamps = None
        self.n_frames = 0
        self.frequency = 50.0

        self.is_playing = False
        self.current_frame = 0
        self.start_time = 0

        self._load_model()
        if filename:
            self.load_trajectory(filename)

    def _load_model(self):
        # Use the same model logic as controller
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

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Home Pose
        self.home_qpos = np.array([0, -1.57, -1.57, -1.57, -1.57, 0])
        self.data.qpos[:] = self.home_qpos
        mujoco.mj_step(self.model, self.data)

    def load_trajectory(self, filename):
        if not os.path.exists(filename):
            print(f"File not found: {filename}")
            return False

        try:
            data = np.load(filename, allow_pickle=True)
            if "qpos" in data:
                self.qpos_traj = data["qpos"]
                self.n_frames = self.qpos_traj.shape[0]

                # Determine frequency
                if "frequency" in data:
                    self.frequency = float(data["frequency"])
                elif "timestamps" in data:
                    ts = data["timestamps"]
                    if len(ts) > 1:
                        avg_dt = (ts[-1] - ts[0]) / (len(ts) - 1)
                        self.frequency = 1.0 / avg_dt if avg_dt > 0 else 50.0

                self.filename = filename
                self.current_frame = 0
                print(f"Loaded {filename}: {self.n_frames} frames, {self.frequency:.1f} Hz")
                return True
            else:
                print("Invalid file format: missing 'qpos'")
                return False
        except Exception as e:
            print(f"Error loading trajectory: {e}")
            return False

    def play(self):
        self.is_playing = True
        self.start_time = time.time()

    def stop(self):
        self.is_playing = False

    def step(self):
        """
        Advance simulation by one step based on time.
        """
        if not self.is_playing or self.qpos_traj is None:
            return None

        # Calculate current frame based on time
        # This handles speed variation and skipped frames
        elapsed = (time.time() - self.start_time) * self.speed
        frame_idx = int(elapsed * self.frequency)

        if self.loop:
            frame_idx = frame_idx % self.n_frames
        else:
            if frame_idx >= self.n_frames:
                frame_idx = self.n_frames - 1
                self.is_playing = False # End of playback

        self.current_frame = frame_idx

        # Set qpos
        target_qpos = self.qpos_traj[frame_idx]

        # We might need to handle dimension mismatch if checking older files
        # but MotionRecorder output should match model qpos size
        d = len(target_qpos)
        m = len(self.data.qpos)
        self.data.qpos[:min(d,m)] = target_qpos[:min(d,m)]

        mujoco.mj_step(self.model, self.data)

        if self.viewer and self.viewer.is_running():
            self.viewer.sync()

        return {
            "frame": frame_idx,
            "total_frames": self.n_frames,
            "is_playing": self.is_playing,
            "time": elapsed
        }

    def launch_viewer(self):
        if not self.no_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.viewer.cam.lookat[:] = [0, 0, 0.7]
            self.viewer.cam.distance = 1.2
            self.viewer.cam.azimuth = 90

    def close(self):
        if self.viewer:
            self.viewer.close()

# CLI Main
def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("file", nargs="?", help="Path to npz file")
    parser.add_argument("--speed", type=float, default=1.0)
    args = parser.parse_args()

    player = ArmTrajectoryPlayer(args.file)
    if player.qpos_traj is not None:
        player.launch_viewer()
        player.play()
        try:
            while player.viewer.is_running():
                player.step()
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        player.close()
    else:
        print("Please provide a valid trajectory file.")

if __name__ == "__main__":
    main()
