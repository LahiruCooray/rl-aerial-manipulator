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
	pid_actual = np.load('pid_actual_positions_episodes.npy', allow_pickle=True)
	pid_desired = np.load('pid_desired_positions_episodes.npy', allow_pickle=True)
	rl_actual = np.load('rl_actual_positions_episodes.npy', allow_pickle=True)
	rl_desired = np.load('rl_desired_positions_episodes.npy', allow_pickle=True)

	pid_rmse, pid_rise, pid_settle = analyze_episodes(pid_actual, pid_desired)
	rl_rmse, rl_rise, rl_settle = analyze_episodes(rl_actual, rl_desired)

	# Plot RMSE (Euclidean norm)
	plot_metric([np.linalg.norm(r) for r in pid_rmse], [np.linalg.norm(r) for r in rl_rmse], 'RMSE', 'RMSE (m)')
	# Plot rise time
	plot_metric(pid_rise, rl_rise, 'Rise Time', 'Steps to 90%')
	# Plot settling time
	plot_metric(pid_settle, rl_settle, 'Settling Time', 'Steps to Settle')

	# Print summary statistics
	print('PID Mean RMSE:', np.mean([np.linalg.norm(r) for r in pid_rmse]))
	print('RL Mean RMSE:', np.mean([np.linalg.norm(r) for r in rl_rmse]))
	print('PID Mean Rise Time:', np.mean(pid_rise))
	print('RL Mean Rise Time:', np.mean(rl_rise))
	print('PID Mean Settling Time:', np.mean(pid_settle))
	print('RL Mean Settling Time:', np.mean(rl_settle))

if __name__ == "__main__":
	main()
