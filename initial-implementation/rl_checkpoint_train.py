import os
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.evaluation import evaluate_policy
from rl_env import WaypointQuadEnv
from torch import nn

CHECKPOINT_PATH = "waypoint_controller6"
VECNORM_PATH = "vec_normalize.pkl"
SAVE_PATH = "waypoint_controller6_final"
TOTAL_TIMESTEPS = 1_000_000

def make_env():
    return Monitor(WaypointQuadEnv())

def train():
    # Vectorized env with Monitor wrapper
    dummy_env = DummyVecEnv([make_env for _ in range(4)])

    # Load VecNormalize stats or create new VecNormalize wrapper
    if os.path.exists(VECNORM_PATH):
        env = VecNormalize.load(VECNORM_PATH, dummy_env)
        env.training = True  # Enable normalization updates during training
        env.norm_reward = False
        print("Loaded VecNormalize stats.")
    else:
        env = VecNormalize(dummy_env, norm_obs=True, norm_reward=False)
        print("Created new VecNormalize environment.")

    policy_kwargs = dict(
        net_arch=[128, 64, 64],
        activation_fn=nn.ReLU
    )

    # Load PPO checkpoint if available, else new model
    if os.path.exists(CHECKPOINT_PATH + ".zip"):
        print(f"Loading model from {CHECKPOINT_PATH}...")
        model = PPO.load(CHECKPOINT_PATH, env=env, device="cuda", policy_kwargs=policy_kwargs)
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
            policy_kwargs=policy_kwargs,
            tensorboard_log="./tensorboard_logs/"
        )

    # Train
    model.learn(total_timesteps=TOTAL_TIMESTEPS, progress_bar=True)

    # Save model and normalization stats
    model.save(SAVE_PATH)
    env.save(VECNORM_PATH)
    print(f"Model saved as {SAVE_PATH}.zip and VecNormalize saved as {VECNORM_PATH}")

    # Evaluate
    mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=10)
    print(f"Mean reward: {mean_reward:.2f} ± {std_reward:.2f}")

    return model

if __name__ == "__main__":
    train()
