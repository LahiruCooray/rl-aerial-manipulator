def mean_successful_episode_length(actual_episodes, desired_episodes, tolerance=0.1):
	lengths = []
	for actual, desired in zip(actual_episodes, desired_episodes):
		actual = np.array(actual)
		desired = np.array(desired)
		min_len = min(len(actual), len(desired))
		if min_len == 0:
			continue
		final_actual = actual[min_len-1]
		final_desired = desired[min_len-1]
		dist = np.linalg.norm(final_actual - final_desired)
		if dist <= tolerance:
			lengths.append(min_len)
	return np.mean(lengths) if lengths else 0.0
def calculate_success_rate(actual_episodes, desired_episodes, tolerance=0.05):
	"""
	Success: Final position within tolerance of desired position for each episode.
	Returns: success_rate (float, percent)
	"""
	successes = 0
	total = len(actual_episodes)
	for actual, desired in zip(actual_episodes, desired_episodes):
		actual = np.array(actual)
		desired = np.array(desired)
		min_len = min(len(actual), len(desired))
		if min_len == 0:
			continue
		final_actual = actual[min_len-1]
		final_desired = desired[min_len-1]
		dist = np.linalg.norm(final_actual - final_desired)
		if dist <= tolerance:
			successes += 1
	return 100.0 * successes / total if total > 0 else 0.0
 
def plot_current_distance(pid_actual, rl_actual, episode_idx=0):
	import os
	save_dir = "saved_plots"
	os.makedirs(save_dir, exist_ok=True)
	pid_a = np.array(pid_actual[episode_idx])
	rl_a = np.array(rl_actual[episode_idx])
	# Compute distance from origin for each step
	pid_dist = np.linalg.norm(pid_a, axis=1)
	rl_dist = np.linalg.norm(rl_a, axis=1)
	steps_pid = np.arange(len(pid_dist))
	steps_rl = np.arange(len(rl_dist))
	plt.figure(figsize=(10,5))
	plt.plot(steps_pid, pid_dist, label='PID Distance from Origin', color='tab:blue')
	plt.plot(steps_rl, rl_dist, label='RL Distance from Origin', color='tab:orange')
	plt.title(f'Distance from Origin - Episode {episode_idx+1}')
	plt.xlabel('Step')
	plt.ylabel('Distance (m)')
	plt.legend()
	plt.grid(True)
	plt.tight_layout()
	filename = os.path.join(save_dir, f"current_distance_episode_{episode_idx+1}.png")
	plt.savefig(filename)
	print(f"Saved plot: {filename}")
	plt.show()
	plt.close()

def plot_xyz_positions(pid_actual, pid_desired, rl_actual, rl_desired, episode_idx=0):
	import os
	save_dir = "saved_plots"
	os.makedirs(save_dir, exist_ok=True)
	labels = ['x', 'y', 'z']
	# Select episode
	pid_a = np.array(pid_actual[episode_idx])
	pid_d = np.array(pid_desired[episode_idx])
	rl_a = np.array(rl_actual[episode_idx])
	rl_d = np.array(rl_desired[episode_idx])
	steps = np.arange(1200)
	fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
	for i, label in enumerate(labels):
		# Prepare y-data for 1200 timesteps, fill missing with np.nan
		pid_a_y = np.full(1200, np.nan)
		pid_d_y = np.full(1200, np.nan)
		rl_a_y = np.full(1200, np.nan)
		rl_d_y = np.full(1200, np.nan)
		if pid_a.shape[0] > 0:
			pid_a_y[:min(1200, pid_a.shape[0])] = pid_a[:min(1200, pid_a.shape[0]), i]
		if pid_d.shape[0] > 0:
			pid_d_y[:min(1200, pid_d.shape[0])] = pid_d[:min(1200, pid_d.shape[0]), i]
		if rl_a.shape[0] > 0:
			rl_a_y[:min(1200, rl_a.shape[0])] = rl_a[:min(1200, rl_a.shape[0]), i]
		if rl_d.shape[0] > 0:
			rl_d_y[:min(1200, rl_d.shape[0])] = rl_d[:min(1200, rl_d.shape[0]), i]
		ax = axs[i]
		ax.plot(steps, pid_a_y, label='PID Actual', color='tab:blue')
		ax.plot(steps, pid_d_y, '--', label='PID Desired', color='tab:cyan')
		ax.plot(steps, rl_a_y, label='RL Actual', color='tab:orange')
		ax.plot(steps, rl_d_y, '--', label='RL Desired', color='tab:red')
		ax.set_ylabel(f'{label} Position (m)')
		ax.set_title(f'{label.upper()} Position - Episode {episode_idx+1}')
		ax.legend()
		ax.grid(True)
	axs[-1].set_xlabel('Step')
	plt.tight_layout()
	filename = os.path.join(save_dir, f"xyz_positions_episode_{episode_idx+1}.png")
	plt.savefig(filename)
	print(f"Saved plot: {filename}")
	plt.show()
	plt.close()
import numpy as np
import matplotlib.pyplot as plt

def compute_rmse(actual, desired):
	# Ensure arrays are float and at least 2D
	actual = np.asarray(actual, dtype=float)
	desired = np.asarray(desired, dtype=float)
	if actual.ndim == 1:
		actual = actual.reshape(-1, 1)
	if desired.ndim == 1:
		desired = desired.reshape(-1, 1)
	return np.sqrt(np.mean((actual - desired) ** 2, axis=0))

def compute_rise_time(actual, desired, threshold=0.9):
	# Returns the step index where actual first reaches threshold*final value
	final = desired[-1]
	start = actual[0]
	delta = final - start
	if np.allclose(delta, 0):
		return 0
	rise_idx = 0
	for i, a in enumerate(actual):
		if np.all(np.abs(a - start) >= threshold * np.abs(delta)):
			rise_idx = i
			break
	return rise_idx

def compute_settling_time(actual, desired, tol=0.05):
	# Returns the last step index where actual leaves a band of tol*final value
	final = desired[-1]
	band = tol * np.abs(final)
	for i in range(len(actual)-1, -1, -1):
		if np.any(np.abs(actual[i] - final) > band):
			return i+1
	return 0

def analyze_episodes(actual_episodes, desired_episodes):
	rmses = []
	rise_times = []
	settling_times = []
	for actual, desired in zip(actual_episodes, desired_episodes):
		min_len = min(len(actual), len(desired))
		actual = np.array(actual[:min_len])
		desired = np.array(desired[:min_len])
		rmses.append(compute_rmse(actual, desired))
		rise_times.append(compute_rise_time(actual, desired))
		settling_times.append(compute_settling_time(actual, desired))
	return np.array(rmses), np.array(rise_times), np.array(settling_times)

def plot_metric(metric_pid, metric_rl, name, ylabel):
	import os
	save_dir = "saved_plots"
	os.makedirs(save_dir, exist_ok=True)
	plt.figure(figsize=(10,5))
	plt.plot(metric_pid, label='PID', alpha=0.7)
	plt.plot(metric_rl, label='RL', alpha=0.7)
	plt.title(f'{name} per Episode')
	plt.xlabel('Episode')
	plt.ylabel(ylabel)
	plt.legend()
	plt.grid(True)
	plt.tight_layout()
	filename = os.path.join(save_dir, f"{name.replace(' ', '_').lower()}.png")
	plt.savefig(filename)
	print(f"Saved plot: {filename}")
	plt.show()
	plt.close()

def main():
	pid_actual = np.load('Compare PID and RL/data/pid_actual_positions_episodes.npy', allow_pickle=True)
	pid_desired = np.load('Compare PID and RL/data/pid_desired_positions_episodes.npy', allow_pickle=True)
	rl_actual = np.load('Compare PID and RL/data/rl_actual_positions_episodes.npy', allow_pickle=True)
	rl_desired = np.load('Compare PID and RL/data/rl_desired_positions_episodes.npy', allow_pickle=True)

	pid_rmse, pid_rise, pid_settle = analyze_episodes(pid_actual, pid_desired)
	rl_rmse, rl_rise, rl_settle = analyze_episodes(rl_actual, rl_desired)

	# Plot RMSE (Euclidean norm)
	#plot_metric([np.linalg.norm(r) for r in pid_rmse], [np.linalg.norm(r) for r in rl_rmse], 'RMSE', 'RMSE (m)')
	# Plot rise time
	#plot_metric(pid_rise, rl_rise, 'Rise Time', 'Steps to 90%')
	# Plot settling time
	#plot_metric(pid_settle, rl_settle, 'Settling Time', 'Steps to Settle')
	# Plot x, y, z positions for first episode
	plot_xyz_positions(pid_actual, pid_desired, rl_actual, rl_desired, episode_idx=2)

	plot_current_distance(pid_actual, rl_actual, episode_idx=2)

	# Print summary statistics
	print('PID Mean RMSE:', np.mean([np.linalg.norm(r) for r in pid_rmse]))
	print('RL Mean RMSE:', np.mean([np.linalg.norm(r) for r in rl_rmse]))
	print('PID Mean Rise Time:', np.mean(pid_rise))
	print('RL Mean Rise Time:', np.mean(rl_rise))
	print('PID Mean Settling Time:', np.mean(pid_settle))
	print('RL Mean Settling Time:', np.mean(rl_settle))

	# Calculate and print success rates
	pid_success = calculate_success_rate(pid_actual, pid_desired, tolerance=0.1)
	rl_success = calculate_success_rate(rl_actual, rl_desired, tolerance=0.1)
	print(f'PID Success Rate: {pid_success:.2f}%')
	print(f'RL Success Rate: {rl_success:.2f}%')

	pid_mean_len = mean_successful_episode_length(pid_actual, pid_desired, tolerance=0.1)
	rl_mean_len = mean_successful_episode_length(rl_actual, rl_desired, tolerance=0.1)
	print(f'PID Mean Episode Length (successful): {pid_mean_len:.2f}')
	print(f'RL Mean Episode Length (successful): {rl_mean_len:.2f}')

if __name__ == "__main__":
	main()
