import os
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env import VecNormalize
from rl_env import WaypointQuadEnv

CHECKPOINT_PATH = "waypoint_controller4"
SAVE_PATH = "waypoint_controller4_final"
TOTAL_TIMESTEPS = 1_000_000  # Can incrementally increase if resuming

def train():
    # Create environment (VecEnv for PPO)
    env = make_vec_env(WaypointQuadEnv, n_envs=4)

    # Load checkpoint if exists
    if os.path.exists(CHECKPOINT_PATH + ".zip"):
        print(f"Loading model from {CHECKPOINT_PATH}...")
        model = PPO.load(CHECKPOINT_PATH, env=env, device="cuda")
    else:
        print("Starting new training...")
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=128,
            n_epochs=10,
            device="cuda",
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,
            verbose=1,
            tensorboard_log="./tensorboard_logs/"
        )

    # Train model
    model.learn(total_timesteps=TOTAL_TIMESTEPS, progress_bar=True)

    # Evaluate performance
    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
    print(f"Mean reward: {mean_reward:.2f} ± {std_reward:.2f}")

    # Final save with descriptive name

    model.save(SAVE_PATH)
    print(f"Final model saved as {SAVE_PATH}.zip")

    return model

if __name__ == "__main__":
    train()
