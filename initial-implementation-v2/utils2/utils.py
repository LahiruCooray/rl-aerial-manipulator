from scipy.spatial.transform import Rotation as R
import numpy as np

def quaternion_to_rpy(w, x, y, z, degrees=True):
    # scipy expects [x, y, z, w] order
    quat = [x, y, z, w]
    r = R.from_quat(quat)
    # 'xyz' = roll (x), pitch (y), yaw (z)
    return r.as_euler('xyz', degrees=degrees)


def linear_trajectory(start, num_waypoints):
    """Generate a linear trajectory with specified number of waypoints."""
    waypoints = []
    print('Linear')
    random_end = np.random.uniform(-1, 1, size=3)  # Random end point
    random_end[2] = np.random.uniform(0.5, 3)

    for i in range(1, num_waypoints+1):
        t = i / (num_waypoints)
        waypoint = start + t * (random_end - start)
        waypoints.append(waypoint)
    return waypoints
        


def curved_trajectory(start, num_waypoints):
    """Generate a curved trajectory with specified number of waypoints."""
    
    waypoints = []
    random_end = np.random.uniform(-1, 1, size=3)  # Random end point
    random_end[2] = np.random.uniform(0.5, 3)
    temp = np.random.randint(0, 3)
    #print(f"Random end point: {random_end}")
    if temp == 0:
        print("Curved-Z")
        for i in range(1, num_waypoints+1):
            t = i / (num_waypoints)
            waypoint = start + t * (random_end - start) + np.array([0, 0, np.sin(2 * t * np.pi)])
            waypoint[2] = max(waypoint[2], 0.2)
            waypoints.append(waypoint)
    elif temp == 1:
        print("Curved-Y")
        for i in range(1, num_waypoints+1):
            t = i / (num_waypoints)
            waypoint = start + t * (random_end - start) + np.array([0, np.sin(2 * t * np.pi), 0])
            waypoint[2] = max(waypoint[2], 0.2)
            waypoints.append(waypoint)
    else:
        print("Curved-X")
        for i in range(1, num_waypoints+1):
            t = i / (num_waypoints)
            waypoint = start + t * (random_end - start) + np.array([np.sin(2 * t * np.pi), 0, 0])
            waypoint[2] = max(waypoint[2], 0.2)
            waypoints.append(waypoint)

    return waypoints



def helical_trajectory(start, num_waypoints, radius=0.8, height_step=0.4, turns=1):
    """
    Generate a helical (spiral) trajectory with specified parameters.
    
    Args:
        start: Starting position [x, y, z]
        num_waypoints: Number of waypoints to generate
        radius: Radius of the helix (default: 0.5m)
        height_step: Vertical distance between each waypoint (default: 0.1m)
        turns: Number of complete turns in the helix (default: 2)
    
    Returns:
        List of waypoints forming a helical trajectory
    """
    waypoints = []
    print('Helical')
    
    # Total angle to cover for the specified number of turns
    total_angle = 2 * np.pi * turns
    
    for i in range(1, num_waypoints + 1):
        # Current angle position
        angle = (i / num_waypoints) * total_angle
        
        # Calculate position on helix
        x = start[0] + radius * np.cos(angle)
        y = start[1] + radius * np.sin(angle)
        z = start[2] + i * height_step
        
        waypoint = np.array([x, y, z])
        # Ensure minimum height of 0.2m
        waypoint[2] = max(waypoint[2], 0.2)
        waypoints.append(waypoint)
    
    return waypoints