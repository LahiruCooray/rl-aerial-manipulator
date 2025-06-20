from quadPlot import plot_quad_3d
from stable_baselines3 import PPO
from rl_env import WaypointQuadEnv
import numpy as np
import matplotlib.pyplot as plt

def main():
    # Load trained RL model
    try:
        model = PPO.load("waypoint_controller")
        print("Loaded trained model successfully!")
    except:
        print("No trained model found. Please train first or use random actions.")
        model = None
    
    # Initialize environment
    env = WaypointQuadEnv()
    obs, _ = env.reset()
    
    # Get waypoints for visualization
    waypoints = np.array(env.waypoint_list)
    print(f"Waypoints to visit: {waypoints}")
    
    # Control loop for animation
    step_count = 0
    max_steps = 2000  # Prevent infinite loops
    
    def control_loop(i):
        nonlocal obs, step_count
        
        # Run multiple control steps per animation frame
        for _ in range(4):  # Same as original runsim.py
            if step_count >= max_steps:
                break
                
            if model is not None:
                # Use trained RL policy
                action, _ = model.predict(obs, deterministic=True)
            else:
                # Random actions for testing
                action = env.action_space.sample()
            
            # Step environment
            obs, reward, terminated, truncated, info = env.step(action)
            step_count += 1
            
            # Print progress
            if step_count % 100 == 0:
                pos = env.quadcopter.position()
                target = env.current_waypoint
                dist = np.linalg.norm(pos - target)
                print(f"Step {step_count}: Distance to waypoint {env.waypoint_index}: {dist:.2f}")
            
            # Check if episode ended
            if terminated or truncated:
                if 'success' in info and info['success']:
                    print("🎉 All waypoints reached successfully!")
                else:
                    print("❌ Episode terminated (crash or timeout)")
                # Reset for continuous visualization
                obs, _ = env.reset()
                waypoints[:] = env.waypoint_list  # Update waypoints for plot
                step_count = 0
                break
        
        return env.quadcopter.world_frame()
    
    # Start 3D visualization
    plot_quad_3d(waypoints, control_loop)

if __name__ == "__main__":
    main()