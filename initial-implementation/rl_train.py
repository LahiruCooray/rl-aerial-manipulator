import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from rl_env import WaypointQuadEnv

def train():
    # Create environment
    env = make_vec_env(WaypointQuadEnv, n_envs=4)
    
    # Initialize RL algorithm
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
        clip_range=0.2,  # Controls how far new policy can deviate from the old one
        ent_coef=0.01,   # Encourages exploration
        verbose=1,
        tensorboard_log="./tensorboard_logs/"
    )

    # Train the model
    model.learn(total_timesteps=1000000, progress_bar=True)
    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
    print(f"Mean reward: {mean_reward} +/- {std_reward}")
    
    # Save the model
    model.save("waypoint_controller4")
    print("Model saved successfully!")
    
    return model

if __name__ == "__main__":
    train()