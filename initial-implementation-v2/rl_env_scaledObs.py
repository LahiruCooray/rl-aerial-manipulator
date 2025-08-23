import numpy as np
import gymnasium as gym
from gymnasium import spaces
from simul_files.model.quadcopter import Quadcopter
import simul_files.model.params as params # Import params module
from scipy.spatial.transform import Rotation
from utils2.utils import quaternion_to_rpy, linear_trajectory, curved_trajectory, helical_trajectory

class WaypointQuadEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        # State: [pos(3), vel(3), quat(4), omega(3), relative_pos(3), relative_pos_next(3), yaw_final(1)] = 20D
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, 
            shape=(20,), dtype=np.float32
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
        self.final_yaw = None
        self.final_waypoint_reached = None
        self.counter = None
        self.counter_activated = None
        self.counter_limit = None
        self.F = None
        self.M = None
        
    def reset(self, seed=None):
        super().reset(seed=seed)
        
        # Random start position
        start_pos = np.random.uniform(-1, 1, 3)
        start_pos[2] = np.random.uniform(1, 2)  # Keep above ground
        #self.num_waypoints = np.random.randint(2, 4) 
        self.num_waypoints =  1
        # Random start orientation
        roll = np.random.uniform(-np.pi/2, np.pi/2)
        pitch = np.random.uniform(-np.pi/2, np.pi/2)
        yaw = np.random.uniform(-np.pi, np.pi)
        start_orientation = (roll, pitch, yaw) if np.random.rand() < 0 else (0, 0, 0)
        self.quadcopter = Quadcopter(start_pos,start_orientation)

        self.current_step = 0
        self.max_episode_steps = 2000
        self.counter = 0
        self.counter_activated = False
        self.counter_limit = 500
        
        # Generate random waypoints

        if np.random.rand() < 0.3:
            self.waypoint_list = linear_trajectory(start_pos, self.num_waypoints)
        elif np.random.rand() < 0.6:
            self.waypoint_list = curved_trajectory(start_pos, self.num_waypoints)
        else:
            self.waypoint_list = helical_trajectory(start_pos, self.num_waypoints)


        #self.waypoint_list = self._generate_waypoints(self.num_waypoints)
        self.final_yaw = self._generate_final_yaw()

        self.waypoint_index = 0
        self.current_waypoint = self.waypoint_list[0]
        self.last_distance = None
        self.final_waypoint_reached = False
        
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

    def _generate_final_yaw(self):
        """Generate a random final yaw angle"""
        return np.random.uniform(-np.pi, np.pi)

    def _get_observation(self):
        """20D state: [pos(3), vel(3), quat(4), omega(3), relative_pos(3), relative_pos_next(3), yaw_final(1)]"""
        pos = self.quadcopter.position()
        vel = self.quadcopter.velocity()
        quat = self.quadcopter.state[6:10]
        omega = self.quadcopter.omega()
        rel_pos = self.current_waypoint - pos
        if self.waypoint_index >= len(self.waypoint_list)-1:
            rel_pos_next = np.zeros(3)
        else:
            rel_pos_next = self.waypoint_list[self.waypoint_index+1] - self.current_waypoint

        # Normalize
        obs = np.concatenate([
            pos / 10.0,
            vel / 5.0,
            quat,
            omega / 5.0,
            rel_pos / 2.0,
            rel_pos_next / 2.0,
            [self.final_yaw / np.pi]  # Normalize yaw to [-1, 1]
        ]).astype(np.float32)

        return obs
    
    def step(self, action):
        # Apply control - use params.mass instead of self.quadcopter.mass
        F = action[0] * params.mass * params.g  # Fixed: use params.mass
        M = action[1:4] * 0.1  # Scale moments
        self.F = F
        self.M = M
        
        
        self.quadcopter.update(self.dt, F, M.reshape(-1, 1))  # M needs to be column vector
        

        # Calculate reward
        reward = self._calculate_reward()
        
        # Check if reached current waypoint
        pos = self.quadcopter.position()
        vel = self.quadcopter.velocity()
        quat = self.quadcopter.state[6:10]
        roll,pitch,yaw = quaternion_to_rpy(quat[0], quat[1], quat[2], quat[3], degrees=False)  # Convert quaternion to yaw
        rot_vel = self.quadcopter.omega()
        distance_to_waypoint = np.linalg.norm(pos - self.current_waypoint)
        truncated = self.current_step >= self.max_episode_steps
        self.current_step += 1

        if distance_to_waypoint < 0.1:  # Reached waypoint   
            if not self.final_waypoint_reached:
                self.waypoint_index += 1
                reward += 100.0  # Big bonus for reaching waypoint
            if self.waypoint_index < len(self.waypoint_list):
                self.current_waypoint = self.waypoint_list[self.waypoint_index]
            else:
                # All waypoints reached!
                # Bonus for low velocity at final waypoint
                if not self.final_waypoint_reached:
                    self.counter_activated = True
                    self.final_waypoint_reached = True
                    velocity_norm = np.linalg.norm(vel)
                    rot_vel_norm = np.linalg.norm(rot_vel)
                    #stop_rotation_bonus =100.0 if rot_vel_norm < 0.1 else -20.0 * rot_vel_norm
                    stopping_bonus = 150.0 * (1-velocity_norm**2) if velocity_norm < 1 else 0.0
                    attaining_final_yaw_bonus = 100.0 * (1 - abs(yaw - self.final_yaw)/(2 * np.pi)) if abs(yaw - self.final_yaw) < (2*np.pi) else 0.0
                    return self._get_observation(), reward + 200.0 + stopping_bonus + attaining_final_yaw_bonus, False, truncated, {'success': True, 'stopped': velocity_norm < 0.1 and rot_vel_norm < 0.1}
                else:
                    if self.counter <= self.counter_limit:
                        self.counter += 1
                        #attaining_final_yaw_bonus = 10.0 if np.isclose(yaw, self.final_yaw, atol=0.05) else -2.0 * (abs(yaw - self.final_yaw))**2
                        attaining_final_yaw_bonus = 30.0 * (1 - abs(yaw - self.final_yaw)/(2 * np.pi)) if abs(yaw - self.final_yaw) < (2*np.pi) else 0.0
                        attaining_zero_roll_bonus = 10.0 *(1 - abs(roll)/0.2) if abs(roll) < 0.2 else -.1 * abs(roll)
                        attaining_zero_pitch_bonus = 10.0 *(1-abs(pitch)/0.2) if abs(pitch) < 0.2 else -.1 * abs(pitch)
                        #print(f"attaining_final_yaw_bonus: {attaining_final_yaw_bonus}, attaining_zero_roll_bonus: {attaining_zero_roll_bonus}, attaining_zero_pitch_bonus: {attaining_zero_pitch_bonus}")
                        return self._get_observation(), reward + attaining_final_yaw_bonus + attaining_zero_roll_bonus + attaining_zero_pitch_bonus, False, truncated, {'success': True, 'stopped': np.linalg.norm(vel) < 0.1 and np.linalg.norm(rot_vel) < 0.1}
                    else:
                        attaining_final_yaw_bonus = 30.0 * (1 - abs(yaw - self.final_yaw)/(2 * np.pi)) if abs(yaw - self.final_yaw) < (2*np.pi) else 0.0
                        attaining_zero_roll_bonus = 10.0 *(1 - abs(roll)/0.2) if abs(roll) < 0.2 else -.1 * abs(roll)
                        attaining_zero_pitch_bonus = 10.0 *(1 - abs(pitch)/0.2) if abs(pitch) < 0.2 else -.1 * abs(pitch)
                        print(f"obs: {self._get_observation()}, waypoint_index: {self.waypoint_index}, attaining_final_yaw_bonus: {attaining_final_yaw_bonus}, attaining_zero_roll_bonus: {attaining_zero_roll_bonus}, attaining_zero_pitch_bonus: {attaining_zero_pitch_bonus}")
                        return self._get_observation(), reward + attaining_final_yaw_bonus + attaining_zero_roll_bonus + attaining_zero_pitch_bonus, True, truncated, {'success': True, 'stopped': np.linalg.norm(vel) < 0.1 and np.linalg.norm(rot_vel) < 0.1}

        if self.counter_activated:
            self.counter += 1

        
        # Termination conditions
        terminated = False

        if pos[2] < 0.1:
            reward -= 100
            if vel[2] < 0:# Quadcopter is falling
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
        distance_reward = -distance * 10
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

        if self.final_waypoint_reached:
            progress_reward = 0.0
            time_penalty = 0.0
            if distance < 0.1:
                distance_reward = 1.0
            
        
        return distance_reward + speed_penalty + time_penalty + progress_reward
