import numpy as np
import gymnasium as gym
from gymnasium import spaces
from simul_files.model.quadcopter import Quadcopter
import simul_files.model.params as params  # Import params module
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

        # Wind parameters
        self.wind_enabled = True
        self.wind_strength = 2.0  # Maximum wind speed (m/s)
        self.wind_turbulence = 0.5  # Turbulence intensity
        self.wind_direction_change_rate = 0.1  # How quickly wind direction changes
        
        # Wind state variables
        self.base_wind_velocity = np.zeros(3)  # Base wind direction
        self.wind_velocity = np.zeros(3)  # Current wind with turbulence
        self.wind_direction_target = np.random.uniform(0, 2*np.pi)  # Target wind direction
        
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

        # Reset wind
        self._reset_wind()
        
        return self._get_observation(), {}
    
    def _reset_wind(self):
        """Reset wind parameters for a new episode"""
        if self.wind_enabled:
            # Random base wind strength and direction
            wind_strength = np.random.uniform(0, self.wind_strength)
            wind_direction = np.random.uniform(0, 2 * np.pi)

            # Conver to velocity components(horizontal wind)
            self.base_wind_velocity = np.array([
                wind_strength * np.cos(wind_direction),
                wind_strength * np.sin(wind_direction),
                np.random.uniform(-0.2, 0.2) * wind_strength  # Small vertical component
            ])
            
            self.wind_direction_target = wind_direction
            self.wind_velocity = self.base_wind_velocity.copy()

        else:
            self.base_wind_velocity = np.zeros(3)
            self.wind_velocity = np.zeros(3)

    def _update_wind(self):
        """Update wind velocity with turbulence and direction changes"""
        if not self.wind_enabled:
            return
        
        # slowly change wind direction
        current_direction = np.arctan2(self.base_wind_velocity[1], self.base_wind_velocity[0])
        direction_diff = self.wind_direction_target - current_direction

        #wrap the direction difference to [-pi, pi]
        direction_diff = np.arctan2(np.sin(direction_diff), np.cos(direction_diff))

        # gradually change the direction
        new_direction = current_direction + direction_diff * self.wind_direction_change_rate * self.dt

        # Ocationally change target direction
        if np.random.rand() < 0.001:  # 0.1% chance to change direction
            self.wind_direction_target = np.random.uniform(0, 2 * np.pi)

        # Update base wind
        wind_magnitude = np.linalg.norm(self.base_wind_velocity[:2])
        self.base_wind_velocity[0] = wind_magnitude * np.cos(new_direction)
        self.base_wind_velocity[1] = wind_magnitude * np.sin(new_direction)

        # Add turbulence
        turbulence = np.random.normal(0, self.wind_turbulence, 3)
        self.wind_velocity = self.base_wind_velocity + turbulence

        # Add gusts (sudden changes in wind speed)
        if np.random.rand() < 0.005:  # 0.5% chance to add a gust
            gust_strength = np.random.uniform(0.5, 2.0)
            gust_direction = np.random.uniform(0, 2 * np.pi)
            gust = np.array([
                gust_strength * np.cos(gust_direction),
                gust_strength * np.sin(gust_direction),
                np.random.uniform(-0.5, 0.5) * gust_strength  # Small vertical gust
            ])
            self.wind_velocity += gust

    def _apply_wind_force(self):
        """Calculate wind force on the quadcopter"""
        if not self.wind_enabled:
            return np.zeros(3)
        
        # Get quadcopter velocity
        quad_vel = self.quadcopter.velocity()

        # Relative wind velocity (wind - quadcopter velocity)
        relative_wind = self.wind_velocity - quad_vel

        # Air drag coefficient
        air_drag_coefficient = 0.1  # Adjust as needed

        # Wind force proportional to relative wind velocity squared
        wind_force = air_drag_coefficient * relative_wind * np.linalg.norm(relative_wind) 

        return wind_force
    
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
            pos ,
            vel ,
            quat,
            omega,
            rel_pos,
            [1.0 if np.allclose(self.current_waypoint, self.waypoint_list[-1]) else 0.0]
        ]).astype(np.float32)
        
        return obs
    
    def step(self, action):

        # Update wind
        self._update_wind()

        # Apply control - use params.mass instead of self.quadcopter.mass
        F = action[0] * params.mass * params.g  # Fixed: use params.mass
        M = action[1:4] * 0.1  # Scale moments

        # Apply wind force
        wind_force = self._apply_wind_force()
        
        self.quadcopter.update(self.dt, F, M.reshape(-1, 1))  # M needs to be column vector
        #Manually apply wind force to quadcopter
        self.quadcopter.state[3:6] += wind_force * self.dt / params.mass # Adjust acceleration due to wind force
        
        # Calculate reward
        reward = self._calculate_reward()
        
        # Check if reached current waypoint
        pos = self.quadcopter.position()
        vel = self.quadcopter.velocity()
        rot_vel = self.quadcopter.omega()
        distance_to_waypoint = np.linalg.norm(pos - self.current_waypoint)
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
        qart = self.quadcopter.state[6:10]
        
        # Distance to current waypoint
        distance = np.linalg.norm(pos - self.current_waypoint)
        
        # Reward components
        distance_reward = max(0.0, 1.0 - distance)  # Closer is better
        speed_penalty = -0.1 * np.sqrt(np.sum(vel**2))  # Penalize high speed

        # Penalty for orientation
        orientation_penatity = -0.1*np.sqrt((qart[0] - 1)**2 + qart[1]**2 + qart[2]**2 + qart[3]**2)  # Penalize non-level orientation

        if np.linalg.norm(rot_vel) > 0.1:
            speed_penalty -= 0.01 * np.linalg.norm(rot_vel)**2
            
        time_penalty = -0.01
        
        # Progress reward (how much closer did we get?)
        if self.last_distance is not None:
            progress = self.last_distance - distance
            progress_reward = 2 * progress
        else:
            progress_reward = 0.0
            
        self.last_distance = distance
        
        return distance_reward + speed_penalty + time_penalty + progress_reward + orientation_penatity

    def set_wind_parameters(self, enabled=True, strength=2.0, turbulence=0.5, direction_change_rate=0.1):
        """Configure wind parameters"""
        self.wind_enabled = enabled
        self.wind_strength = strength
        self.wind_turbulence = turbulence
        self.wind_direction_change_rate = direction_change_rate
        
    def get_wind_info(self):
        """Get current wind information for debugging/visualization"""
        return {
            'base_wind_velocity': self.base_wind_velocity,
            'current_wind_velocity': self.wind_velocity,
            'wind_strength': np.linalg.norm(self.wind_velocity),
            'wind_direction': np.arctan2(self.wind_velocity[1], self.wind_velocity[0])
        }