from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from rl_env_scaledObs import WaypointQuadEnv
from torch import nn
import torch
import os
from stable_baselines3.common.callbacks import CheckpointCallback
import gc
gc.collect()
torch.cuda.empty_cache()

# Save every 1 million steps
checkpoint_callback = CheckpointCallback(
    save_freq=12500, 
    save_path="./checkpoints_from_10_9M/",
    name_prefix="ppo_model"
)

CHECKPOINT_PATH = "checkpoints_from_8_6M/ppo_model_2300000_steps"

def train():
    # Create environment
    env = make_vec_env(WaypointQuadEnv, n_envs=8)
   # env = VecNormalize(env, norm_obs=True, norm_reward=False)

    policy_kwargs = dict(
        net_arch=[128, 64, 64],
        activation_fn=nn.Tanh  # Using Tanh for better exploration in continuous action spaces
    )

    # Load PPO checkpoint if available, else new model
    if os.path.exists(CHECKPOINT_PATH + ".zip"):
        print(f"Loading model from {CHECKPOINT_PATH}...")
        model = PPO.load(CHECKPOINT_PATH, env=env,device="cuda" if torch.cuda.is_available() else "cpu" , policy_kwargs=policy_kwargs, ent_coef=0.0001)
    else:
        print("Starting new training...")    
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=2e-4,
            n_steps= 2048,  # Number of steps to run for each environment per update
            batch_size=128,
            n_epochs=12,
            device="cuda" if torch.cuda.is_available() else "cpu",
            gamma=0.995,
            gae_lambda=0.9,
            clip_range=0.2,  # Controls how far new policy can deviate from the old one
            ent_coef=0.0005,   # Encourages exploration
            verbose=1,
            policy_kwargs=policy_kwargs,
            tensorboard_log="./tensorboard_logs_v4/"
        )

    # Train the model
    model.learn(total_timesteps=4100000, callback=checkpoint_callback, progress_bar=True)
    model.save("waypoint_controller_v4_b_15M")  # Save the final model
    print("Model saved successfully!")

    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
    print(f"Mean reward: {mean_reward} +/- {std_reward}")
    
    
    return model

if __name__ == "__main__":
    train()