from simul_files.quadPlot import plot_quad_3d, reset_history, save_plots_as_jpeg
from stable_baselines3 import PPO
from rl_env_scaledObs import WaypointQuadEnv
import numpy as np
import matplotlib.pyplot as plt



from stable_baselines3 import PPO
from rl_env_scaledObs import WaypointQuadEnv
import numpy as np

NUM_EPISODES = 100
MAX_STEPS = 600

all_actual = []
all_desired = []

try:
    model = PPO.load("checkpoints_from_8_6M/ppo_model_2300000_steps.zip", custom_objects={
        "clip_range": 0.2,
        "lr_schedule": 0.0003
    })
    print("Loaded trained model successfully!")
except:
    print("No trained model found. Please train first or use random actions.")
    model = None


# Use the same random pairs as PID controller
import os
random_pairs_file = "data/random_start_end_pairs.npy"
if not os.path.exists(random_pairs_file):
    raise FileNotFoundError("data/random_start_end_pairs.npy not found. Run the PID script first to generate it.")
random_pairs = np.load(random_pairs_file, allow_pickle=True)

for ep in range(NUM_EPISODES):
    start_pos, end_pos = random_pairs[ep]
    env = WaypointQuadEnv(start_pos=np.array(start_pos), target_pos=np.array(end_pos))
    obs, _ = env.reset()
    actual_positions = []
    desired_positions = []
    for step in range(MAX_STEPS):
        if model is not None:
            action, _ = model.predict(obs, deterministic=True)
        else:
            action = env.action_space.sample()
        pos = env.quadcopter.position().copy()
        actual_positions.append(pos)
        desired_positions.append(env.current_waypoint.copy())
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            break
    all_actual.append(np.array(actual_positions))
    all_desired.append(np.array(desired_positions))
    print(f"Episode {ep+1}/{NUM_EPISODES} finished, steps: {len(actual_positions)}")

np.save("data/rl_actual_positions_episodes.npy", np.array(all_actual, dtype=object))
np.save("data/rl_desired_positions_episodes.npy", np.array(all_desired, dtype=object))
print("Saved RL data for all episodes in data/ folder.")