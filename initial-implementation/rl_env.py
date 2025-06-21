import numpy as np
import gymnasium as gym
from gymnasium import spaces
from model.quadcopter import Quadcopter
import model.params as params  # Import params module
from scipy.spatial.transform import Rotation

class WaypointQuadEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        # State: [pos(3), vel(3), quat(4), omega(3)] = 13D
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, 
            shape=(13,), dtype=np.float32
        )
        
        # Action: [thrust, moment_x, moment_y, moment_z]
        self.action_space = spaces.Box(
            low=np.array([0, -1, -1, -1]), 
            high=np.array([2.0, 1, 1, 1]), 
            dtype=np.float32
        )
        
        self.quadcopter = None
        self.current_waypoint = None
        self.waypoint_list = []
        self.waypoint_index = 0
        self.dt = 1.0/200.0
        self.last_distance = None
        
    def reset(self, seed=None):
        super().reset(seed=seed)
        
        # Random start position
        start_pos = np.random.uniform(-2, 2, 3)
        start_pos[2] = np.random.uniform(0.5, 3)  # Keep above ground
        
        self.quadcopter = Quadcopter(start_pos, (0,0,0))
        
        # Generate random waypoints
        self.waypoint_list = self._generate_waypoints()
        self.waypoint_index = 0
        self.current_waypoint = self.waypoint_list[0]
        self.last_distance = None
        
        return self._get_observation(), {}
    
    def _generate_waypoints(self, num_waypoints=5):
        """Generate random waypoints in 3D space"""
        waypoints = []
        for _ in range(num_waypoints):
            wp = np.array([
                np.random.uniform(-3, 3),    # x
                np.random.uniform(-3, 3),    # y  
                np.random.uniform(1, 5)      # z (stay above ground)
            ])
            waypoints.append(wp)
        return waypoints
    
    def _get_observation(self):
        """13D state: [pos(3), vel(3), quat(4), omega(3)]"""
        pos = self.quadcopter.position()
        vel = self.quadcopter.velocity()
        
        # Get quaternion directly from state (already stored as quaternion)
        quat = self.quadcopter.state[6:10]  # [qw, qx, qy, qz]
        
        omega = self.quadcopter.omega()
        
        return np.concatenate([pos, vel, quat, omega]).astype(np.float32)
    
    def step(self, action):
        # Apply control - use params.mass instead of self.quadcopter.mass
        F = action[0] * params.mass * params.g  # Fixed: use params.mass
        M = action[1:4] * 0.1  # Scale moments
        
        self.quadcopter.update(self.dt, F, M.reshape(-1, 1))  # M needs to be column vector
        
        # Calculate reward
        reward = self._calculate_reward()
        
        # Check if reached current waypoint
        pos = self.quadcopter.position()
        vel = self.quadcopter.velocity()
        rot_vel = self.quadcopter.omega()
        distance_to_waypoint = np.linalg.norm(pos - self.current_waypoint)
        
        if distance_to_waypoint < 0.2:  # Reached waypoint
            reward += 100.0  # Big bonus for reaching waypoint
            self.waypoint_index += 1
            
            if self.waypoint_index < len(self.waypoint_list):
                self.current_waypoint = self.waypoint_list[self.waypoint_index]
            else:
                # All waypoints reached!
                # Bonus for low velocity at final waypoint
                velocity_norm = np.linalg.norm(vel)
                rot_vel_norm = np.linalg.norm(rot_vel)
                stop_rotation_bonus =200.0 if rot_vel_norm < 0.1 else -20.0 * rot_vel_norm
                stopping_bonus = 200.0 if velocity_norm < 0.1 else -10.0 * velocity_norm
                return self._get_observation(), reward + 300.0 + stopping_bonus + stop_rotation_bonus, True, False, {'success': True, 'stopped': velocity_norm < 0.1}
        
        # Termination conditions
        terminated = False

        if pos[2] < 0.1:
            reward -= 100.0
            return self._get_observation(), reward, True, False, {'success': False, 'crashed': True}
        if np.linalg.norm(pos) > 10:
            reward -= 50.0
            return self._get_observation(), reward, True, False, {'success': False, 'out_of_bounds': True}
        return self._get_observation(), reward, terminated, False, {}
    
    def _calculate_reward(self):
        pos = self.quadcopter.position()
        vel = self.quadcopter.velocity()
        rot_vel = self.quadcopter.omega()
        
        # Distance to current waypoint
        distance = np.linalg.norm(pos - self.current_waypoint)
        
        # Reward components
        distance_reward = -distance * 1.5  # Closer is better
        speed_penalty = -0.01 * np.linalg.norm(vel)**2  # Don't go too fast
        if np.linalg.norm(rot_vel) > 0.1:
            speed_penalty -= 0.01 * np.linalg.norm(rot_vel)**2
        time_penalty = -0.1  # Encourage minimal time
        
        # Progress reward (how much closer did we get?)
        if self.last_distance is not None:
            progress = self.last_distance - distance
            progress_reward = 30 * progress
        else:
            progress_reward = 0.0
            
        self.last_distance = distance
        
        return distance_reward + speed_penalty + time_penalty + progress_reward