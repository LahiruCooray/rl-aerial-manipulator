import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
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
        batch_size=64,
        n_epochs=10,
        verbose=1,
        tensorboard_log="./tensorboard_logs/"
    )
    
    # Train the model
    model.learn(total_timesteps=100000)  # Reduced for testing
    
    # Save the model
    model.save("waypoint_controller")
    print("Model saved successfully!")
    
    return model

if __name__ == "__main__":
    train()