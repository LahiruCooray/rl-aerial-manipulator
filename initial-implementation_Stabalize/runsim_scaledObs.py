from simul_files.quadPlot import plot_quad_3d
from stable_baselines3 import PPO
from rl_env_scaledObs import WaypointQuadEnv
import numpy as np
import matplotlib.pyplot as plt


def main():
    # Load trained RL model
    try:
        model = PPO.load("D:/Final Year/FYP/rl-aerial-manipulator/initial-implementation_Stabalize/waypoint_controller_unnormalized_4_10M.zip", custom_objects={
    "clip_range": 0.2,        
    "lr_schedule": 0.0003     
})
        print("Loaded trained model successfully!")
    except:
        print("No trained model found. Please train first or use random actions.")
        model = None
    
    # Initialize environment
    env = WaypointQuadEnv()

    # Configure wind parameters for training
    env.set_wind_parameters(
        enabled=True,
        strength=2.0,           # Moderate wind for training
        turbulence=0.5,         # Medium turbulence
        direction_change_rate=0.1  # Gradual direction changes
    )

    obs, _ = env.reset()
    
    # Get waypoints for visualization
    waypoints_holder = [np.array(env.waypoint_list)]
    print(f"Waypoints to visit: {waypoints_holder[0]}")
    
    # Control loop for animation
    step_count = 0
    max_steps = 5000  # Prevent infinite loops
    
    def control_loop(i):
        nonlocal obs, step_count, waypoints_holder
        
        # Run multiple control steps per animation frame
        for _ in range(4):  # Same as original runsim.py
            if step_count >= max_steps:
                obs, _ = env.reset()
                waypoints_holder[0] = np.array(env.waypoint_list)  # Update waypoints for plot
                step_count = 0
                print("Resetting environment after reaching max steps.")
                print(f"Waypoints to visit: {waypoints_holder[0]}")
            
                
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
                if ('success' in info) and info['success']:
                    print("🎉 All waypoints reached successfully!")
                elif ('success' in info ) and ("crashed" in info) and not info['success'] and info["crashed"]:
                    print("🚨 Episode terminated (crashed): Quadcopter hit the ground!")
                elif ('success' in info) and ("out_of_bounds" in info) and not info['success'] and info["out_of_bounds"]:
                    print("🚨 Episode terminated (out of bounds): Quadcopter flew too far!")
                else:                   
                    print("🚨 Episode terminated: Quadcopter stopped moving!")
                 
                # Reset for continuous visualization
                obs, _ = env.reset()
                waypoints_holder[0] = np.array(env.waypoint_list)  # Update waypoints for plot
                print(f"Waypoints to visit: {waypoints_holder[0]}")
                step_count = 0
                break
        
        return env.quadcopter.world_frame()
    
    # Start 3D visualization
    plot_quad_3d(waypoints_holder, control_loop)

if __name__ == "__main__":
    main()