import os
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from rl_env_scaledObs import WaypointQuadEnv
from torch import nn
import torch

CHECKPOINT_PATH = "D:/Final Year/FYP/rl-aerial-manipulator/initial-implementation_Stabalize/waypoint_controller_unnormalized.zip"
SAVE_PATH = "D:/Final Year/FYP/rl-aerial-manipulator/initial-implementation_Stabalize/waypoint_controller_unnormalized_stable_5M"
TOTAL_TIMESTEPS = 3  # Can incrementally increase if resuming

def train():
    # Create environment (VecEnv for PPO)
    env = make_vec_env(WaypointQuadEnv, n_envs=8)

    # Load checkpoint if exists
    policy_kwargs = dict(
        net_arch=[128, 64, 64],
        activation_fn=nn.Tanh  # Using Tanh for better exploration in continuous action spaces
    )

    # Load PPO checkpoint if available, else new model
    if os.path.exists(CHECKPOINT_PATH + ".zip"):
        print(f"Loading model from {CHECKPOINT_PATH}...")
        model = PPO.load(CHECKPOINT_PATH, env=env,device="cuda" if torch.cuda.is_available() else "cpu" , policy_kwargs=policy_kwargs)
    else:
        print("Starting new training...")
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=2e-4,
            n_steps=2048,
            batch_size=128,
            n_epochs=10,
            device="cuda" if torch.cuda.is_available() else "cpu",
            gamma=0.995,
            gae_lambda=0.9,
            clip_range=0.2,  # Controls how far new policy can deviate from the old one
            ent_coef=0.01,   # Encourages exploration
            verbose=1,
            policy_kwargs=policy_kwargs,
            tensorboard_log="./tensorboard_logs/"
        )

    # Train model
    model.learn(total_timesteps=TOTAL_TIMESTEPS, progress_bar=True)
    
    model.save(SAVE_PATH)
    print(f"Final model saved as {SAVE_PATH}.zip")

    # Evaluate performance
    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
    print(f"Mean reward: {mean_reward:.2f} ± {std_reward:.2f}")


    return model

if __name__ == "__main__":
    train()
