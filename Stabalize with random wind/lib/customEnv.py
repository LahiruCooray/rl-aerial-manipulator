# Imports
import os  
import time  # Time-related functions for delays and timestamps
import airsim  # Microsoft AirSim simulator client library
import numpy as np  
from tf_agents.specs import array_spec                        # TensorFlow Agents specifications for array shapes and types
from tf_agents.environments import py_environment             # Base class for Python environments in TF-Agents
from tf_agents.trajectories import time_step as ts            # Time step utilities for RL trajectories


#################################################
# AirSim environment definition
#################################################

class DroneEnvironment(py_environment.PyEnvironment):  # Inherit from TF-Agents Python environment base class

  '''
  :param enable_wind: whether to enable the wind
  :param randomize_initial_pose: whether to randomize the initial position and orientation of the drone
  '''
  def __init__(self, enable_wind=False, randomize_initial_pose=False, save_path=None, decrease_unreal_load=False):
    self.enable_wind = enable_wind                                # Store whether wind effects should be enabled during training
    self.randomize_initial_pose = randomize_initial_pose          # Store whether to randomize drone starting position
    self.save_path = save_path  
    self._states_arr = None                                       # Initialize array to store drone states for analysis (starts as None)

    self.client = airsim.MultirotorClient()                       # Creates the AirSim client object that acts as the communication bridge between your Python code and the AirSim simulator
    # "MultirotorClient" is specifically for drone/quadcopter control (vs "CarClient" for cars)
    if decrease_unreal_load: self.client.simRunConsoleCommand("t.MaxFPS 5")     # Limit Unreal Engine FPS to reduce computational load. Normally AirSim runs at 60 FPS.
    self.client.confirmConnection()                                             # Establish and confirm connection to AirSim simulator
    self.client.enableApiControl(True)                                          # Enable API control of the drone (vs manual control)
    self.client.armDisarm(True)                                                 # Arm the drone motors (prepare for flight)
    
    self._observation_spec = array_spec.ArraySpec(shape=(19,), dtype=np.float32, name='observation')     # Define observation space: 19-dimensional float32 array representing drone state
    self._action_spec = array_spec.BoundedArraySpec(shape=(4,), dtype=np.float32, name='action', minimum=0.0, maximum=1.0)    # Motor PWM values (0.0 to 1.0)

    
    #self._state, _, _, _, _, _, _ = self.getState()  
    self._episode_ended = False  
    self._total_reward = 0 
  
  def action_spec(self):  
    return self._action_spec  
  
  def observation_spec(self):  
    return self._observation_spec  



  '''Generates a new pose, randomized or not
  '''
  def getNewPose(self, random=False, random_uniform=False):
    pos_stddev = 0.25  
    or_stddev = 0.3  # Standard deviation for orientation noise in radians (~17 degrees)
    if random:  # If randomization is enabled
      if random_uniform:  # If using uniform random quaternion generation
        u = np.random.uniform()  # Random uniform value [0,1] for quaternion generation
        v = np.random.uniform()  
        w = np.random.uniform()  
        # Create pose with normally distributed position noise and uniformly random quaternion
        new_pose = airsim.Pose(position_val=airsim.Vector3r(0.0 + np.random.normal(0, pos_stddev), 0.0 + np.random.normal(0, pos_stddev), -100.0 + np.random.normal(0, pos_stddev)),
                                orientation_val=airsim.Quaternionr(np.sqrt(1-u)*np.sin(2*np.pi*v), np.sqrt(1-u)*np.cos(2*np.pi*v), np.sqrt(u)*np.sin(2*np.pi*w), np.sqrt(u)*np.cos(2*np.pi*w))) # (marsaglia method)
      else:  # Using normal distribution for orientation
        # Create pose with normally distributed position noise
        new_pose = airsim.Pose(position_val=airsim.Vector3r(0.0 + np.random.normal(0, pos_stddev), 0.0 + np.random.normal(0, pos_stddev), -100.0 + np.random.normal(0, pos_stddev)),
                                #orientation_val=airsim.Quaternionr(0.0, 0.0, 0.0, 1.0))  # Commented: no orientation noise
                                orientation_val=airsim.utils.to_quaternion(np.random.normal(0, or_stddev), np.random.normal(0, or_stddev), np.random.normal(0, or_stddev)))  # Convert roll, pitch, yaw noise to quaternion
    else:  # No randomization - use fixed pose
      new_pose = airsim.Pose(position_val=airsim.Vector3r(0.0, 0.0, -100.0), orientation_val=airsim.Quaternionr(0.0, 0.0, 0.0, 1.0))
    # Reference pose is always the same (origin, upright)
    reference_pose = airsim.Pose(position_val=airsim.Vector3r(0.0, 0.0, -100.0), orientation_val=airsim.Quaternionr(0.0, 0.0, 0.0, 1.0))
    return new_pose, reference_pose  # Return both the new pose and reference pose
  
  '''Resets the pose of the drone to what is specified inside the function and prints the state of the multirotor (flying or not)
  '''
  def reset_pose(self): #Core Purpose: Episode Reset Protocol
    self.client.reset()                   # Reset the AirSim simulation to initial state
    self.client.enableApiControl(True)    # Re-enable API control after reset
    self.client.armDisarm(True)           # Re-arm the drone motors after reset
    '''When AirSim resets, the drone starts in "LandedState.Landed". In this state, the physics engine is NOT fully engaged. Motor commands won't work properly. The drone won't respond to PWM values correctly.
    To engage the physics engine, we need to take off the drone. After takeoff: "LandedState.Flying" → Ready for motor control'''
    self.client.takeoffAsync(timeout_sec=0.1, vehicle_name="SimpleFlight").join()                 


    # Generate new pose (randomized or not based on initialization setting)
    new_pose, self.initial_pose = self.getNewPose(self.randomize_initial_pose)

    # Set the drone's pose in the simulator (don't ignore collisions)
    self.client.simSetVehiclePose(pose=new_pose, ignore_collision=False, vehicle_name="SimpleFlight")
    # Check and print whether drone is properly flying (physics engaged)
    if self.client.getMultirotorState().landed_state == airsim.LandedState.Landed: 
      print("[LANDED: Physics Engine NOT Engaged]")  # Drone is on ground, physics not active
    else: 
      print("[CORRECTLY FLYING: Physics Engine Engaged]")  # Drone is airborne, physics active
    time.sleep(0.01)  # Small delay to ensure drone is ready before neural network starts
  
  '''Sets a random wind in the simulation, with the given standard deviation in [m]
  '''
  def setRandomWind(self, stddev=2.5):
    x_val = np.random.normal(0, stddev)  # Random wind component in X direction
    y_val = np.random.normal(0, stddev)  
    z_val = np.random.normal(0, stddev)  
    wind = airsim.Vector3r(x_val, y_val, z_val)  # Create wind vector
    print('Wind set < x y z >: <', x_val, y_val, z_val, '>')  # Print wind values for debugging
    self.client.simSetWind(wind)  # Apply wind to the simulation
  
  '''Resets the custom environment created
  '''
  def _reset(self):
    print('Total reward for the previous episode:', self._total_reward)  
    self._total_reward = 0                              # Reset total reward counter for new episode
    self._steps = 0                                     # Reset step counter for new episode
    if self.save_path is not None:  
      if self._states_arr is not None:  # If there are states to save from previous episode
        if not os.path.exists(self.save_path+'/states'):  
          os.makedirs(self.save_path+'/states')
        # Save the states array with current timestamp as filename
        np.save(self.save_path+'/states/'+str(time.time()), self._states_arr)
      self._states_arr = np.empty((0,19))  # Initialize empty array for new episode states

    if self.enable_wind:  # If wind is enabled
      self.setRandomWind()  
    self.reset_pose()  # Reset drone to initial pose

    # Get initial state and unpack (only use first element - the state array)
    self._state, _, _, _, _, _, _ = self.getState()
    self._episode_ended = False                 # Mark episode as active
    return ts.restart(self._state)              # Return restart time step with initial observation



  '''Moves the drone as specified by the action, first checking for the termination conditions
  :param action: the tensorflow action, as described by the array spec in the __init__ function
  :param duration: how long the duration of the action has to be. If the movemetnt is continuous, only specifies the maximum duration (it is asyncronous)
  :param continuous: whether to do continuous movements or wait for the action to end before going back to the network inference
  :return: True if the episode has to end due to collisions or other, False otherwise
  '''
  def move(self, action, duration=1.002, continuous=True):
    #if self.client.simGetCollisionInfo().has_collided or self.client.simGetVehiclePose().position.z_val > -10: return True

    # With join(), thrust resets to default hovering (2.42). Without join(), it maintains network's decision
    if continuous == True:  # Continuous movement mode (asynchronous)
      scale = 2.5                            # Scale factor for delta thrust magnitude
      b_th = 0.59                            # Bias thrust value for hovering
      d_th = scale * (action-0.5) / 5        # Calculate delta thrust: scale * normalized action / 5
      th = np.clip(b_th + d_th, 0, 1)        # Final thrust = base + delta, clipped to [0,1] range
      # Send motor PWM commands asynchronously (non-blocking)
      self.client.moveByMotorPWMsAsync(front_right_pwm=float(th[0]), rear_left_pwm=float(th[1]), front_left_pwm=float(th[2]), rear_right_pwm=float(th[3]), duration=duration)
        # duration: how long the action should last (in seconds)
      #self.client.moveByMotorPWMsAsync(front_right_pwm=0.0, rear_left_pwm=float(th[1]), front_left_pwm=float(th[2]), rear_right_pwm=float(th[3]), duration=duration)
      time.sleep(0.002)  # Small delay to prevent overwhelming the simulator

    else:  # Discrete movement mode (synchronous)
      #self.client.hoverAsync()  # Commented hover command
      scale = 2.0  # Different scale factor for discrete mode
      b_th = 0.59  # Base thrust value for hovering
      d_th = scale * (action-0.5) / 5  # Calculate delta thrust
      th = np.clip(b_th + d_th, 0, 1)  # Final thrust = base + delta, clipped
      # Send motor PWM commands synchronously (blocking with .join())
      self.client.moveByMotorPWMsAsync(front_right_pwm=float(th[0]), rear_left_pwm=float(th[1]), front_left_pwm=float(th[2]), rear_right_pwm=float(th[3]), duration=duration).join()
    
    return False  # Return False (no termination condition met)

  '''Override, one step: performs an action, retrieves the reward, returns either the transition or the termination signals associated
  :param action: the action to perform decided by the agent
  :return: either a transition or a termination
  '''
  def _step(self, action):
    # If episode has ended, reset the environment
    if self._episode_ended: 
      return self.reset()  # Return reset time step

    # Execute the action and check for termination conditions
    end_now = self.move(action=action)  # Perform movement, get termination flag
    # Get new state and unpack all components
    self._state, pos, orient, ang_acc, ang_vel, lin_acc, lin_vel = self.getState()
    # If data saving is enabled, append current state to states array
    if self.save_path is not None: 
      self._states_arr = np.concatenate((self._states_arr, [self._state]), axis=0)
    # Calculate reward based on current state
    reward = self.reward_function(pos, orient, ang_acc, ang_vel, lin_acc, lin_vel)

    # Handle episode termination
    if end_now:  # If termination condition was met
      print('Collision occurred or episode termination condition met')  # Debug message
      self._episode_ended = True  # Mark episode as ended
      reward = 0  # Set reward to 0 for termination (could be negative for collision)
      return ts.termination(self._state, reward=reward)  # Return termination time step
    else:  # Episode continues
      self._total_reward += reward  # Add reward to episode total
      return ts.transition(self._state, reward=reward)  # Return transition time step



  '''Returns the state as a numpy array with float32 values
  '''
  def getState(self):
    state = self.client.getMultirotorState()  # Get current drone state from AirSim
    pos = state.kinematics_estimated.position  # Extract position (x, y, z)
    orient = state.kinematics_estimated.orientation  # Extract orientation quaternion (w, x, y, z)
    ang_acc = state.kinematics_estimated.angular_acceleration  # Extract angular acceleration (x, y, z)
    ang_vel = state.kinematics_estimated.angular_velocity  # Extract angular velocity (x, y, z)
    lin_acc = state.kinematics_estimated.linear_acceleration  # Extract linear acceleration (x, y, z)
    lin_vel = state.kinematics_estimated.linear_velocity  # Extract linear velocity (x, y, z)
    
    # Create normalized state array (19 dimensions total)
    return np.array([(pos.z_val-self.initial_pose.position.z_val)/10,  # Normalized Z position relative to initial
                    (pos.x_val-self.initial_pose.position.x_val)/10,   # Normalized X position relative to initial
                    (pos.y_val-self.initial_pose.position.y_val)/10,   # Normalized Y position relative to initial
                    orient.w_val, orient.x_val, orient.y_val, orient.z_val,  # Quaternion components (already normalized)
                    ang_acc.x_val/10, ang_acc.y_val/10, ang_acc.z_val/10,   # Normalized angular acceleration
                    ang_vel.x_val/10, ang_vel.y_val/10, ang_vel.z_val/10,   # Normalized angular velocity
                    lin_acc.x_val/10, lin_acc.y_val/10, lin_acc.z_val/10,   # Normalized linear acceleration
                    lin_vel.x_val/10, lin_vel.y_val/10, lin_vel.z_val/10],  # Normalized linear velocity
                    dtype=np.float32), pos, orient, ang_acc, ang_vel, lin_acc, lin_vel  # Return state array and individual components
  
  '''Returns the reward, given the pose (state)
  '''
  def reward_function(self, pos, orient, ang_acc, ang_vel, lin_acc, lin_vel):
    # Primary reward: 1 minus distance from initial position (max reward=1 when at exact initial position)
    reward = max(0, 1 - np.sqrt((pos.z_val-self.initial_pose.position.z_val)**2 + (pos.x_val-self.initial_pose.position.x_val)**2 + (pos.y_val-self.initial_pose.position.y_val)**2))
    # Penalty for orientation deviation from upright (quaternion distance from [0,0,0,1])
    reward -= 0.1 * np.sqrt((orient.w_val-1)**2 + orient.x_val**2 + orient.y_val**2 + orient.z_val**2)
    # Penalty for excessive angular velocity (encourages stable hovering)
    reward -= 0.1 * np.sqrt(ang_vel.x_val**2 + ang_vel.y_val**2 + ang_vel.z_val**2)

    self._steps += 1  # Increment step counter
    # Print position every 10 steps for debugging (commented out)
    #if self._steps % 10 == 0: print('Position of drone: <', pos.x_val, pos.y_val, pos.z_val, '>')

    return reward  # Return calculated reward