import numpy as np
import gymnasium as gym
from gymnasium import spaces
from simul_files.model.quadcopter import Quadcopter
import simul_files.model.params as params # Import params module
from scipy.spatial.transform import Rotation

class WaypointQuadEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        # State: [pos(3), vel(3), quat(4), omega(3), relative_pos(3), is_final(1)] = 17D
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, 
            shape=(17,), dtype=np.float32
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
        start_pos = np.random.uniform(-1, 1, 3)
        start_pos[2] = np.random.uniform(1, 2)  # Keep above ground
        self.num_waypoints = np.random.randint(1, 3) 
        
        self.quadcopter = Quadcopter(start_pos, (0,0,0))

        self.current_step = 0
        self.max_episode_steps = 1200
        
        # Generate random waypoints
        self.waypoint_list = self._generate_waypoints(self.num_waypoints)
        self.waypoint_index = 0
        self.current_waypoint = self.waypoint_list[0]
        self.last_distance = None
        
        return self._get_observation(), {}
    
    def _generate_waypoints(self, num_waypoints=2):
        """Generate random waypoints in 3D space"""
        waypoints = []
        for _ in range(num_waypoints):
            wp = np.array([
                np.random.uniform(-1, 1),    # x
                np.random.uniform(-1, 1),    # y  
                np.random.uniform(1, 3)      # z (stay above ground)
            ])
            waypoints.append(wp)
        return waypoints
    
    def _get_observation(self):
        """17D state: [pos(3), vel(3), quat(4), omega(3), relative_pos(3), is_final(1)]"""
        pos = self.quadcopter.position()
        vel = self.quadcopter.velocity()
        quat = self.quadcopter.state[6:10]
        omega = self.quadcopter.omega()
        rel_pos = self.current_waypoint - pos

        # Normalize
        obs = np.concatenate([
            pos / 10.0,
            vel / 5.0,
            quat,
            omega / 5.0,
            rel_pos / 2.0,
            [1.0 if np.allclose(self.current_waypoint, self.waypoint_list[-1]) else 0.0]
        ]).astype(np.float32)
        
        return obs
    
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
        #print(f"Distance to waypoint {self.waypoint_index}: {distance_to_waypoint:.2f}, length of waypoint list: {len(self.waypoint_list)}")
        waypoint_dir = self.current_waypoint - pos
        unit_dir = waypoint_dir / np.linalg.norm(waypoint_dir)
        vel_toward_waypoint = np.dot(vel, unit_dir)

        
        if distance_to_waypoint  < 0.5 and vel_toward_waypoint > 0.1:
            reward += 10.0
        elif distance_to_waypoint < 0.5 and vel_toward_waypoint < 0.1:
            reward -= 10.0
        if distance_to_waypoint < 0.1:  # Reached waypoint
            reward += 100.0  # Big bonus for reaching waypoint
            self.waypoint_index += 1
            
            if self.waypoint_index < len(self.waypoint_list):
                self.current_waypoint = self.waypoint_list[self.waypoint_index]
            else:
                # All waypoints reached!
                # Bonus for low velocity at final waypoint
                velocity_norm = np.linalg.norm(vel)
                rot_vel_norm = np.linalg.norm(rot_vel)
                stop_rotation_bonus =100.0 if rot_vel_norm < 0.1 else -20.0 * rot_vel_norm
                stopping_bonus = 100.0 if velocity_norm < 0.1 else -10.0 * velocity_norm
                return self._get_observation(), reward + 400.0 + stopping_bonus + stop_rotation_bonus, True, False, {'success': True, 'stopped': velocity_norm < 0.1}
        
        truncated = self.current_step >= self.max_episode_steps
        self.current_step += 1
        # Termination conditions
        terminated = False
        #print(f" Current waypoint: {self.current_waypoint}, Position: {pos}")
        if pos[2] < 0.1:
            reward -= 100
            if vel[2] < 0:# Quadcopter is falling
               # print(vel[2])
                reward += vel[2] * 100.0  # Heavily penalize falling
            return self._get_observation(), reward, True, truncated, {'success': False, 'crashed': True}
        if np.linalg.norm(pos) > 10:
            reward -= 100.0
            return self._get_observation(), reward, True, truncated, {'success': False, 'out_of_bounds': True}
        return self._get_observation(), reward, terminated, truncated, {}
    
    def _calculate_reward(self):
        pos = self.quadcopter.position()
        vel = self.quadcopter.velocity()
        rot_vel = self.quadcopter.omega()
        
        # Distance to current waypoint
        distance = np.linalg.norm(pos - self.current_waypoint)
        
        # Reward components
        distance_reward = -distance * 2
        speed_penalty = -0.1 * np.linalg.norm(vel)**2  # Don't go too fast
        if np.linalg.norm(rot_vel) > 0.1:
            speed_penalty -= 0.01 * np.linalg.norm(rot_vel)**2
        time_penalty = -0.1
        
        # Progress reward (how much closer did we get?)
        if self.last_distance is not None:
            progress = self.last_distance - distance
            progress_reward = 20 * progress
            if progress_reward > 0:
                progress_reward += 2
        else:
            progress_reward = 0.0
            
        self.last_distance = distance
        
        return distance_reward + speed_penalty + time_penalty + progress_reward
