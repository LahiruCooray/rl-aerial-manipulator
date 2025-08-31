"""
author: Peter Huang, Antonio Cuni
email: hbd730@gmail.com, anto.cuni@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!


Modified by Methsara Shamika in year 2025

"""

from quadPlot import plot_quad_3d
import pid_controller as pid  

import trajGen3D
from model.quadcopter import Quadcopter
import numpy as np

# Simulation settings
animation_frequency = 50  # Hz
control_frequency = 200   # Hz (inner control loop)
control_iterations = control_frequency / animation_frequency
dt = 1.0 / control_frequency
time = [0.0]

def attitudeControl(quad, time, waypoints, coeff_x, coeff_y, coeff_z):
    desired_state = trajGen3D.generate_trajectory(time[0], 1.2, waypoints, coeff_x, coeff_y, coeff_z)
    dt = 0.01  # Time step
    F, M = pid.run(quad, desired_state, dt)  
    quad.update(dt, F, M)
    time[0] += dt

def main():
    # Initial position and orientation
    pos = (0.5, 0, 0)
    attitude = (0, 0, 0)
    quadcopter = Quadcopter(pos, attitude)

    # Generate 3D helical waypoints and minimum snap trajectory coefficients
    waypoints = trajGen3D.get_helix_waypoints(10, 5)            # radius and height of the helix   6,9         10,5
    
    # Example: straight line along X axis
    #waypoints = [[x, 0.0, 0.0] for x in np.linspace(0, 10, 10)]
    
    coeff_x, coeff_y, coeff_z = trajGen3D.get_MST_coefficients(waypoints)
    
    print("All waypoints:")
    for i, point in enumerate(waypoints):
        print(f"Point {i+1}: x={point[0]:.3f}, y={point[1]:.3f}, z={point[2]:.3f}")

    # Print debugging info about waypoints
    print("DEBUG waypoints shape:", np.shape(waypoints))
    print("DEBUG waypoints dtype:", type(waypoints))
    print("DEBUG waypoints sample:", waypoints[:2])

    # Define the function to call on each frame
    def control_loop(i):
        for _ in range(int(control_iterations)):
            attitudeControl(quadcopter, time, waypoints, coeff_x, coeff_y, coeff_z)
        return quadcopter.world_frame()

    # Call the 3D plot and run animation
    plot_quad_3d(waypoints, control_loop)

if __name__ == "__main__":
    main()
