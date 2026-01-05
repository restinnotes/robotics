import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import os
import sys
import glob
import numpy as np

# Ensure project root is in path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

# Import logic classes
# Note: Assuming files are in the same directory (scripts/)
from sensor_imu_arm_control import ArmImuController
from arm_trajectory_player import ArmTrajectoryPlayer

class ArmControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("UR3e Arm Control & Playback")
        self.root.geometry("600x500")

        # Style
        style = ttk.Style()
        style.configure("TButton", padding=6)
        style.configure("TLabel", padding=5)

        # Tabs
        self.notebook = ttk.Notebook(root)
        self.notebook.pack(expand=True, fill='both', padx=10, pady=10)

        self.tab_control = ttk.Frame(self.notebook)
        self.tab_playback = ttk.Frame(self.notebook)

        self.notebook.add(self.tab_control, text='  Remote Control  ')
        self.notebook.add(self.tab_playback, text='  Playback  ')

        # --- Control Tab ---
        self.controller = None
        self._init_control_tab()

        # --- Playback Tab ---
        self.player = None
        self._init_playback_tab()

        # Loop
        self.update_interval = 20 # ms
        self.root.after(self.update_interval, self.update_loop)

        # Handle Exit
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _init_control_tab(self):
        frame = ttk.LabelFrame(self.tab_control, text="Sensor Control (ID 37)")
        frame.pack(fill='x', padx=10, pady=10)

        # Connection
        btn_connect = ttk.Button(frame, text="Connect Sensor & Start", command=self.cmd_connect)
        btn_connect.grid(row=0, column=0, padx=5, pady=5)

        btn_calibrate = ttk.Button(frame, text="Calibrate (Zero)", command=self.cmd_calibrate)
        btn_calibrate.grid(row=0, column=1, padx=5, pady=5)

        # Status
        self.lbl_status = ttk.Label(frame, text="Status: Disconnected", foreground="red")
        self.lbl_status.grid(row=1, column=0, columnspan=2, sticky="w")

        # Data Display
        data_frame = ttk.LabelFrame(self.tab_control, text="Live Data")
        data_frame.pack(fill='x', padx=10, pady=10)

        self.lbl_yaw = ttk.Label(data_frame, text="Yaw: 0.0°")
        self.lbl_yaw.pack(anchor="w")
        self.lbl_pitch = ttk.Label(data_frame, text="Pitch: 0.0°")
        self.lbl_pitch.pack(anchor="w")
        self.lbl_target = ttk.Label(data_frame, text="Target Pan: 0.0° | Lift: 0.0°")
        self.lbl_target.pack(anchor="w")

        # Recording
        rec_frame = ttk.LabelFrame(self.tab_control, text="Recording")
        rec_frame.pack(fill='x', padx=10, pady=10)

        self.btn_record = ttk.Button(rec_frame, text="Start Recording", command=self.cmd_toggle_record)
        self.btn_record.pack(side="left", padx=5, pady=5)

        self.lbl_rec_status = ttk.Label(rec_frame, text="Ready")
        self.lbl_rec_status.pack(side="left", padx=5)

    def _init_playback_tab(self):
        # File Selection
        frame_file = ttk.Frame(self.tab_playback)
        frame_file.pack(fill='x', padx=10, pady=5)

        ttk.Button(frame_file, text="Refresh List", command=self.refresh_file_list).pack(side="right")
        ttk.Label(frame_file, text="Recordings:").pack(side="left")

        # Listbox with Scrollbar
        list_frame = ttk.Frame(self.tab_playback)
        list_frame.pack(fill='both', expand=True, padx=10)

        self.listbox = tk.Listbox(list_frame, height=8)
        self.listbox.pack(side="left", fill="both", expand=True)

        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.listbox.yview)
        scrollbar.pack(side="right", fill="y")
        self.listbox.config(yscrollcommand=scrollbar.set)

        self.refresh_file_list()

        # Controls
        ctrl_frame = ttk.Frame(self.tab_playback)
        ctrl_frame.pack(fill='x', padx=10, pady=10)

        ttk.Button(ctrl_frame, text="Msg: Load & Play", command=self.cmd_play_selected).pack(side="left", padx=5)
        ttk.Button(ctrl_frame, text="Stop", command=self.cmd_stop_playback).pack(side="left", padx=5)

        self.chk_loop_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(ctrl_frame, text="Loop", variable=self.chk_loop_var).pack(side="left", padx=10)

        # Info
        self.lbl_play_info = ttk.Label(self.tab_playback, text="Ready")
        self.lbl_play_info.pack(padx=10)

    def refresh_file_list(self):
        self.listbox.delete(0, tk.END)
        rec_dir = os.path.join(project_root, "data", "recordings")
        if not os.path.exists(rec_dir):
            os.makedirs(rec_dir)

        files = glob.glob(os.path.join(rec_dir, "*.npz"))
        # Also check data/
        data_files = glob.glob(os.path.join(project_root, "data", "*.npz"))

        all_files = sorted(files + data_files, key=os.path.getmtime, reverse=True)

        for f in all_files:
            # Show relative path for brevity
            try:
                rel = os.path.relpath(f, project_root)
            except:
                rel = f
            self.listbox.insert(tk.END, rel)

    # --- Actions: Control ---
    def cmd_connect(self):
        if self.controller:
            return

        # Close playback if running to avoid conflicts
        if self.player:
            self.cmd_stop_playback()

        try:
            self.controller = ArmImuController(no_viewer=False) # Separate thread/process for viewer?
            # Note: launch_passive runs in same process, so update_loop needs to call sync()
            # If we used launch_passive, the window is created and we must call sync in loop.

            if self.controller.connect_sensor():
                self.controller.launch_viewer()
                self.lbl_status.config(text="Status: Connected (MuJoCo running)", foreground="green")
            else:
                messagebox.showerror("Error", "Failed to connect to sensor")
                self.controller = None
        except Exception as e:
            messagebox.showerror("Error", str(e))
            print(e)

    def cmd_calibrate(self):
        if self.controller:
            self.controller.calibrate_sensor()
            messagebox.showinfo("Info", "Calibration sent. Ensure sensor is at zero position.")

    def cmd_toggle_record(self):
        if not self.controller:
            return

        if self.controller.recorder.is_recording:
            # Stop
            stats = self.controller.stop_recording()
            self.btn_record.config(text="Start Recording")
            self.lbl_rec_status.config(text=f"Saved: {stats['n_frames']} frames")
            self.refresh_file_list()
        else:
            # Start
            self.controller.start_recording()
            self.btn_record.config(text="Stop Recording")
            self.lbl_rec_status.config(text="Recording... 0s")

    # --- Actions: Playback ---
    def cmd_play_selected(self):
        sel = self.listbox.curselection()
        if not sel:
            return

        # Close controller viewer if running to avoid conflicts
        if self.controller and self.controller.viewer:
            try:
                if self.controller.viewer.is_running():
                    # Ask user or auto-close controller viewer
                    # For now, we'll just close it automatically
                    self.controller.viewer.close()
            except:
                pass

        filename = self.listbox.get(sel[0])
        abs_path = os.path.join(project_root, filename)

        if self.player:
            self.player.close()

        try:
            self.player = ArmTrajectoryPlayer(abs_path, loop=self.chk_loop_var.get())
            if self.player.qpos_traj is not None:
                self.player.launch_viewer()
                self.player.play()
                self.lbl_play_info.config(text=f"Playing: {filename}")
            else:
                self.lbl_play_info.config(text="Failed to load file")
        except Exception as e:
            messagebox.showerror("Error", str(e))
            import traceback
            traceback.print_exc()

    def cmd_stop_playback(self):
        if self.player:
            self.player.stop()
            self.player.close()
            self.player = None
            self.lbl_play_info.config(text="Stopped")

    def update_loop(self):
        # Control Loop
        if self.controller:
            try:
                stats = self.controller.step()
                if stats and self.controller.viewer.is_running():
                    # Update UI
                    self.lbl_yaw.config(text=f"Yaw: {np.degrees(stats['yaw']):.1f}°")
                    self.lbl_pitch.config(text=f"Pitch: {np.degrees(stats['pitch']):.1f}°")
                    self.lbl_target.config(text=f"Pan: {np.degrees(stats['pan']):.1f}° | Lift: {np.degrees(stats['lift']):.1f}°")

                    if stats['is_recording']:
                        self.lbl_rec_status.config(text=f"Recording... {stats['recording_frames']} frames")

                if self.controller.viewer and not self.controller.viewer.is_running():
                    # Viewer closed
                    self.controller.close()
                    self.controller = None
                    self.lbl_status.config(text="Status: Disconnected", foreground="red")
            except Exception as e:
                print(e)
                self.controller = None

        # Playback Loop
        if self.player and self.player.is_playing:
            try:
                p_stats = self.player.step()
                if p_stats:
                    self.lbl_play_info.config(text=f"Frame: {p_stats['frame']}/{p_stats['total_frames']}")
                elif not self.player.is_playing:
                    # Playback ended naturally
                    self.lbl_play_info.config(text="Playback finished")

                # Check if viewer was closed
                if self.player.viewer:
                    try:
                        if not self.player.viewer.is_running():
                            self.cmd_stop_playback()
                    except:
                        # Viewer may have been closed unexpectedly
                        self.cmd_stop_playback()
            except Exception as e:
                print(f"Playback error: {e}")
                import traceback
                traceback.print_exc()
                self.cmd_stop_playback()

        # Schedule next
        self.root.after(self.update_interval, self.update_loop)

    def on_close(self):
        if self.controller:
            self.controller.close()
        if self.player:
            self.player.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ArmControlApp(root)
    root.mainloop()
