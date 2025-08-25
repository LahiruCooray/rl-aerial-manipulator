"""
author: Peter Huang, Antonio Cuni
email: hbd730@gmail.com, anto.cuni@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!


Modified by Methsara Shamika in year 2025

"""

from quadPlot_pid import plot_quad_3d
import pid_controller as pid  


import trajGen
import trajGen3D
from simul_files.model.quadcopter import Quadcopter
import numpy as np
import os

# Simulation settings
animation_frequency = 50  # Hz
control_frequency = 200   # Hz (inner control loop)
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency
time = [0.0]

def attitudeControl(quad, time, waypoints, coeff_x, coeff_y, coeff_z):
    desired_state = trajGen3D.generate_trajectory(time[0], 1.2, waypoints, coeff_x, coeff_y, coeff_z)
    dt = 0.01  # Time step
    F, M = pid.run(quad, desired_state, dt)  
    quad.update(dt, F, M)
    time[0] += dt


NUM_EPISODES = 100
MAX_STEPS = 600


# Generate or load random pairs for all episodes
random_pairs_file = "data/random_start_end_pairs.npy"
if not os.path.exists(random_pairs_file):
    random_pairs = []
    for _ in range(NUM_EPISODES):
        start_pos = [
            np.random.uniform(-1, 1),
            np.random.uniform(-1, 1),
            np.random.uniform(0, 8)
        ]
        end_pos = [
            np.random.uniform(-1, 1),
            np.random.uniform(-1, 1),
            np.random.uniform(0, 8)
        ]
        random_pairs.append((start_pos, end_pos))
    np.save(random_pairs_file, np.array(random_pairs, dtype=object))
else:
    random_pairs = np.load(random_pairs_file, allow_pickle=True)

all_actual = []
all_desired = []


for ep in range(NUM_EPISODES):
    start_pos, end_pos = random_pairs[ep]
    waypoints = np.stack([np.array(start_pos, dtype=float), np.array(end_pos, dtype=float)])
    pos = tuple(waypoints[0])
    attitude = (0, 0, 0)
    quadcopter = Quadcopter(pos, attitude)
    coeff_x, coeff_y, coeff_z = trajGen3D.get_MST_coefficients(waypoints)
    time = [0.0]
    actual_positions = []
    desired_positions = []
    for step in range(MAX_STEPS):
        desired_state = trajGen3D.generate_trajectory(time[0], 1.2, waypoints, coeff_x, coeff_y, coeff_z)
        F, M = pid.run(quadcopter, desired_state, dt)
        quadcopter.update(dt, F, M)
        time[0] += 0.01
        pos = quadcopter.position().copy()
        actual_positions.append(pos)
        desired_positions.append(desired_state.pos.copy())
        target = np.array(waypoints[-1])
        dist = np.linalg.norm(pos - target)
        if dist < 0.1:
            break
        if not (-1 <= pos[0] <= 1 and -1 <= pos[1] <= 1 and 0 <= pos[2] <= 8):
            break
    all_actual.append(np.array(actual_positions))
    all_desired.append(np.array(desired_positions))
    print(f"Episode {ep+1}/{NUM_EPISODES} finished, steps: {len(actual_positions)}")

np.save("data/pid_actual_positions_episodes.npy", np.array(all_actual, dtype=object))
np.save("data/pid_desired_positions_episodes.npy", np.array(all_desired, dtype=object))
print("Saved PID data for all episodes in data/ folder.")