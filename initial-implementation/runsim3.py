from quadPlot import plot_quad_3d
from stable_baselines3 import PPO
from rl_env import WaypointQuadEnv
import numpy as np
import matplotlib.pyplot as plt
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize


def main():
    # Load trained RL model
    try:
        model = PPO.load("waypoint_controller6_final6", custom_objects={
    "clip_range": 0.2,        
    "lr_schedule": 0.0003     
})
        print("Loaded trained model successfully!")
    except:
        print("No trained model found. Please train first or use random actions.")
        model = None
    
    try:
        # Wrap environment
        dummy_env = DummyVecEnv([lambda: WaypointQuadEnv()])
        env = VecNormalize.load("vec_normalize.pkl", dummy_env)
        env.training = False      # Disable normalization updates
        env.norm_reward = False   # Don’t normalize reward during eval
        print("✅ Loaded VecNormalize stats.")
    except Exception as e:
        print(f"❌ Failed to load VecNormalize: {e}")
        return
    obs,_ = env.envs[0].reset()
    
    # Get waypoints for visualization
    waypoints = np.array(env.envs[0].waypoint_list)

    print(f"Waypoints to visit: {waypoints}")
    
    # Control loop for animation
    step_count = 0
    max_steps = 5000  # Prevent infinite loops
    
    def control_loop(i):
        nonlocal obs, step_count
        
        # Run multiple control steps per animation frame
        for _ in range(4):  # Same as original runsim.py
            if step_count >= max_steps:
                obs,_= env.envs[0].reset()
                waypoints = env.envs[0].waypoint_list  # Update waypoints for plot
                step_count = 0
                break
            
                
            if model is not None:
                # Use trained RL policy
                action, _ = model.predict(obs, deterministic=True)
            else:
                # Random actions for testing
                action = env.envs[0].action_space.sample()
            
            # Step environment
            #print(env.step(action))
            obs, reward, terminated, truncated, info = env.envs[0].step(action)
            step_count += 1
            
            # Print progress
            if step_count % 100 == 0:
                pos = env.envs[0].quadcopter.position()
                target = env.envs[0].current_waypoint
                dist = np.linalg.norm(pos - target)
                print(f"Step {step_count}: Distance to waypoint {env.envs[0].waypoint_index}: {dist:.2f}")
            
            # Check if episode ended
            if terminated or truncated:
                if ('success' in info) and info['success']:
                    print("🎉 All waypoints reached successfully!")
                elif ('success' in info ) and ("crashed" in info) and not info['success'] and info["crashed"]:
                    print("🚨 Episode terminated (crashed): Quadcopter hit the ground!")
                elif ('success' in info) and ("out_of_bounds" in info) and not info['success'] and info["out_of_bounds"]:
                    print("🚨 Episode terminated (out of bounds): Quadcopter flew too far!")
                else:                   
                    print("🚨 Episode terminated: Quadcopter stopped moving!")
                 
                # Reset for continuous visualization
                obs, _ = env.envs[0].reset()
                waypoints = env.envs[0].waypoint_list  # Update waypoints for plot
                step_count = 0
                break
        
        return env.envs[0].quadcopter.world_frame()
    
    # Start 3D visualization
    plot_quad_3d(waypoints, control_loop)

if __name__ == "__main__":
    main()