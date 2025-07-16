from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from rl_env_scaledObs import WaypointQuadEnv
from torch import nn
import torch

def train():
    # Create environment
    env = make_vec_env(WaypointQuadEnv, n_envs=8)
   # env = VecNormalize(env, norm_obs=True, norm_reward=False)

    policy_kwargs = dict(
        net_arch=[128, 64, 64],
        activation_fn=nn.Tanh  # Using Tanh for better exploration in continuous action spaces
    )
    # Initialize RL algorithm
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

    # Train the model
    model.learn(total_timesteps=1000000, progress_bar=True)
    model.save("waypoint_controller_unnormalized")
    print("Model saved successfully!")

    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
    print(f"Mean reward: {mean_reward} +/- {std_reward}")
    
    
    return model

if __name__ == "__main__":
    train()