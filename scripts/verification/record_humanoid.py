"""
Record humanoid walking and save as video
"""
from loco_mujoco.task_factories import ImitationFactory, DefaultDatasetConf
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
output_path = os.path.join(current_dir, "humanoid_walk.mp4")

print("Loading humanoid walk environment...")
env = ImitationFactory.make(
    'UnitreeH1',
    default_dataset_conf=DefaultDatasetConf(['walk']),
    n_substeps=20
)

print(f"Recording to: {output_path}")
env.play_trajectory(
    n_episodes=1,
    n_steps_per_episode=500,
    render=True,
    record=True,
    recorder_params={'path': output_path}
)
print("Done!")
