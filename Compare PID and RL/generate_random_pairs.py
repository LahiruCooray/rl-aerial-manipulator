import numpy as np

NUM_EPISODES = 100
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
import os
os.makedirs("data", exist_ok=True)
np.save("data/random_start_end_pairs.npy", np.array(random_pairs, dtype=object))
print("random_start_end_pairs.npy generated for 100 episodes in data/ folder.")
