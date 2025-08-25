"""
author: Peter Huang
email: hbd730@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!
"""
"""
Modified by Lahiru Cooray in year 2025
new features:
- plot waypoints dynamically
- improve history tracking(reset history when position is reset)
- improved plot_quad_3d function 
- dynamically save plots as JPEG images
- added velocity, position, omega, rpy, acceleration, force and torque plotting
"""

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import cnames
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys
import os

from scipy.spatial.transform import Rotation as R


def quaternion_to_rpy(w, x, y, z, degrees=True):
    # scipy expects [x, y, z, w] order
    quat = [x, y, z, w]
    r = R.from_quat(quat)
    # 'xyz' = roll (x), pitch (y), yaw (z)
    return r.as_euler('xyz', degrees=degrees)


history = np.zeros((500,3))
velocity_history = np.zeros((500,3))  # Add velocity history tracking
position_history = np.zeros((500,3))  # Add position history tracking
omega_history = np.zeros((500,3))  # Add omega history tracking
rpy_history = np.zeros((500,3))  # Add RPY history tracking
acceleration_history = np.zeros((500,3))  # Add acceleration history tracking
force_history = np.zeros((500,1))  # Add force history tracking (scalar)
torque_history = np.zeros((500,3))  # Add torque history tracking
count = 0
timesteps_per_frame_global = 1  # Add global variable for timesteps per frame

def calculate_linear_acceleration(current_velocity, previous_velocity, dt):
    """Calculate linear acceleration from velocity difference"""
    if previous_velocity is None:
        return np.zeros(3)
    return (current_velocity - previous_velocity) / dt

def plot_quad_3d(waypoints, get_world_frame, env, timesteps_per_frame, plots2d=True):
    """
    get_world_frame is a function which returns the "next" world frame to be drawn
    """
    waypoints_holder = waypoints
    waypoints = waypoints[0]
    
    # Create separate figures - one for 3D trajectory, one for other plots
    fig_3d = plt.figure(figsize=(10, 8))
    fig_3d.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05)
    ax = fig_3d.add_subplot(111, projection='3d')  # 3D trajectory plot in separate window
    
    # Only create 2D plots figure if plots2d is True
    if plots2d:
        fig_plots = plt.figure(figsize=(20, 16))
        ax_vel = fig_plots.add_subplot(331)  # 2D velocity plot (row 1, col 1)
        ax_pos = fig_plots.add_subplot(332)  # 2D position plot (row 1, col 2)
        ax_omega = fig_plots.add_subplot(333)  # 2D omega plot (row 1, col 3)
        ax_rpy = fig_plots.add_subplot(334)  # 2D RPY plot (row 2, col 1)
        ax_accel = fig_plots.add_subplot(335)  # 2D acceleration plot (row 2, col 2)
        ax_force = fig_plots.add_subplot(336)  # 2D force plot (row 2, col 3)
        ax_torque = fig_plots.add_subplot(337)  # 2D torque plot (row 3, col 1)
        
        # Adjust spacing between subplots in the plots window to maximize space usage
        fig_plots.subplots_adjust(left=0.05, right=0.98, top=0.95, bottom=0.05, hspace=0.3, wspace=0.2)
    else:
        fig_plots = None
        ax_vel = ax_pos = ax_omega = ax_rpy = ax_accel = ax_force = ax_torque = None
    
    # Store axes globally for access in other functions
    global velocity_ax, position_ax, omega_ax, rpy_ax, accel_ax, force_ax, torque_ax, main_ax, waypoints_line, timesteps_per_frame_global, previous_velocity, fig_3d_global, fig_plots_global, plots2d_enabled
    velocity_ax = ax_vel
    position_ax = ax_pos
    omega_ax = ax_omega
    rpy_ax = ax_rpy
    accel_ax = ax_accel
    force_ax = ax_force
    torque_ax = ax_torque
    main_ax = ax
    fig_3d_global = fig_3d
    fig_plots_global = fig_plots
    plots2d_enabled = plots2d
    timesteps_per_frame_global = timesteps_per_frame
    previous_velocity = None
    
    # Plot placeholders for quadrotor body, arms, trajectory, waypoints, and history
    quad_body, = ax.plot([], [], [], '-', c='green', linewidth=2, label='Quad Body')
    quad_arms, = ax.plot([], [], [], '-', c='red', linewidth=2, label='Quad Arms')
    trajectory, = ax.plot([], [], [], '-', c='blue', marker='o', markevery=2, markersize=5, label='Trajectory')
    waypoints_plot, = ax.plot([], [], [], '.', c='magenta', markersize=6, label='Waypoints')
    history_plot, = ax.plot([], [], [], '--', c='red', markersize=2, label='History')  # Dashed line for history
    
    # Store waypoints line globally
    waypoints_line = waypoints_plot

    # Only setup 2D plot lines if plots2d is enabled
    if plots2d:
        # Velocity plot setup
        vel_x_line, = ax_vel.plot([], [], 'r-', label='Vel X', linewidth=2)
        vel_y_line, = ax_vel.plot([], [], 'g-', label='Vel Y', linewidth=2)
        vel_z_line, = ax_vel.plot([], [], 'b-', label='Vel Z', linewidth=2)
        
        ax_vel.set_xlabel('Time Steps')
        ax_vel.set_ylabel('Velocity (m/s)')
        ax_vel.set_title('Velocity')
        ax_vel.legend()
        ax_vel.grid(True)

        # Position plot setup
        pos_x_line, = ax_pos.plot([], [], 'r-', label='Pos X', linewidth=2)
        pos_y_line, = ax_pos.plot([], [], 'g-', label='Pos Y', linewidth=2)
        pos_z_line, = ax_pos.plot([], [], 'b-', label='Pos Z', linewidth=2)
        
        ax_pos.set_xlabel('Time Steps')
        ax_pos.set_ylabel('Position (m)')
        ax_pos.set_title('Position')
        ax_pos.legend()
        ax_pos.grid(True)

        # Omega plot setup
        omega_x_line, = ax_omega.plot([], [], 'r-', label='Omega X', linewidth=2)
        omega_y_line, = ax_omega.plot([], [], 'g-', label='Omega Y', linewidth=2)
        omega_z_line, = ax_omega.plot([], [], 'b-', label='Omega Z', linewidth=2)
        
        ax_omega.set_xlabel('Time Steps')
        ax_omega.set_ylabel('Angular Velocity (rad/s)')
        ax_omega.set_title('Angular Velocity')
        ax_omega.legend()
        ax_omega.grid(True)

        # RPY plot setup
        roll_line, = ax_rpy.plot([], [], 'r-', label='Roll', linewidth=2)
        pitch_line, = ax_rpy.plot([], [], 'g-', label='Pitch', linewidth=2)
        yaw_line, = ax_rpy.plot([], [], 'b-', label='Yaw', linewidth=2)
        
        ax_rpy.set_xlabel('Time Steps')
        ax_rpy.set_ylabel('Angle (rad)')
        ax_rpy.set_title('Roll-Pitch-Yaw')
        ax_rpy.legend()
        ax_rpy.grid(True)

        # Acceleration plot setup
        accel_x_line, = ax_accel.plot([], [], 'r-', label='Accel X', linewidth=2)
        accel_y_line, = ax_accel.plot([], [], 'g-', label='Accel Y', linewidth=2)
        accel_z_line, = ax_accel.plot([], [], 'b-', label='Accel Z', linewidth=2)
        
        ax_accel.set_xlabel('Time Steps')
        ax_accel.set_ylabel('Acceleration (m/s²)')
        ax_accel.set_title('Linear Acceleration')
        ax_accel.legend()
        ax_accel.grid(True)

        # Force plot setup
        force_line, = ax_force.plot([], [], 'k-', label='Force', linewidth=2)
        
        ax_force.set_xlabel('Time Steps')
        ax_force.set_ylabel('Force (N)')
        ax_force.set_title('Force')
        ax_force.legend()
        ax_force.grid(True)

        # Torque plot setup
        torque_x_line, = ax_torque.plot([], [], 'r-', label='Torque X', linewidth=2)
        torque_y_line, = ax_torque.plot([], [], 'g-', label='Torque Y', linewidth=2)
        torque_z_line, = ax_torque.plot([], [], 'b-', label='Torque Z', linewidth=2)
        
        ax_torque.set_xlabel('Time Steps')
        ax_torque.set_ylabel('Torque (N⋅m)')
        ax_torque.set_title('Torque')
        ax_torque.legend()
        ax_torque.grid(True)

    # Set axis limits and labels for 3D plot
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(-0.5, 8)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Quadrotor 3D Trajectory')
    ax.legend(loc='upper left')

    # Reset history when simulation starts
    global history, velocity_history, position_history, omega_history, rpy_history, acceleration_history, force_history, torque_history, count
    history = np.zeros((500,3))
    velocity_history = np.zeros((500,3))
    position_history = np.zeros((500,3))
    omega_history = np.zeros((500,3))
    rpy_history = np.zeros((500,3))
    acceleration_history = np.zeros((500,3))
    force_history = np.zeros((500,1))
    torque_history = np.zeros((500,3))
    count = 0
    previous_velocity = None

    plot_waypoints(waypoints)
    
    # Create animation for 3D figure
    an = animation.FuncAnimation(
        fig_3d,  # Use 3D figure for animation
        anim_callback,
        fargs=(get_world_frame, waypoints_holder, env),  # Pass env to callback
        init_func=None,
        frames=400,
        interval=10,
        blit=False
    )

    if len(sys.argv) > 1 and sys.argv[1] == 'save':
        print("saving")
        an.save('sim.gif', dpi=80, writer='imagemagick', fps=60)
    else:
        # Show both windows
        plt.show(block=False)  # Show both figures non-blocking
        plt.show()  # Final blocking show

def save_plots_as_jpeg(save_dir, episode_num):
    """Save current plots as JPEG images"""
    global fig_3d_global, fig_plots_global, plots2d_enabled
    
    # Create directory if it doesn't exist
    os.makedirs(save_dir, exist_ok=True)
    
    # Save 3D trajectory plot
    fig_3d_path = os.path.join(save_dir, f"3d_trajectory_episode_{episode_num}.jpg")
    fig_3d_global.savefig(fig_3d_path, format='jpeg', dpi=150, bbox_inches='tight')
    
    # Save 2D plots only if they exist
    if plots2d_enabled and fig_plots_global is not None:
        fig_plots_path = os.path.join(save_dir, f"telemetry_plots_episode_{episode_num}.jpg")
        fig_plots_global.savefig(fig_plots_path, format='jpeg', dpi=150, bbox_inches='tight')
        print(f"Plots saved: {fig_3d_path}, {fig_plots_path}")
    else:
        print(f"3D plot saved: {fig_3d_path}")

def reset_history(save_plots=False, save_dir="./plot_saves", episode_num=0):
    """Call this function when position is reset to clear history"""
    global history, velocity_history, position_history, omega_history, rpy_history, acceleration_history, force_history, torque_history, count, previous_velocity
    
    # Save plots if requested
    if save_plots and count > 0:  # Only save if we have data
        save_plots_as_jpeg(save_dir, episode_num)
    
    history = np.zeros((500,3))
    velocity_history = np.zeros((500,3))
    position_history = np.zeros((500,3))
    omega_history = np.zeros((500,3))
    rpy_history = np.zeros((500,3))
    acceleration_history = np.zeros((500,3))
    force_history = np.zeros((500,1))
    torque_history = np.zeros((500,3))
    count = 0
    previous_velocity = None

def plot_waypoints(waypoints):
    global main_ax, waypoints_line
    waypoints_line.set_data(waypoints[:,0], waypoints[:,1])
    waypoints_line.set_3d_properties(waypoints[:,2])

def set_limit(x, y, z):
    ax = plt.gca()
    ax.set_xlim(x)
    ax.set_ylim(y)
    ax.set_zlim(z)

def anim_callback(i, get_world_frame, waypoints_holder, env):
    frame = get_world_frame(i)
    set_frame(frame, env)  # Pass env to set_frame
    plot_waypoints(waypoints_holder[0])

def set_frame(frame, env):
    # convert 3x6 world_frame matrix into three line_data objects which is 3x2 (row:point index, column:x,y,z)
    lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]
    global main_ax
    lines = main_ax.get_lines()
    for line, line_data in zip(lines[:3], lines_data):
        x, y, z = line_data
        line.set_data(x, y)
        line.set_3d_properties(z)

    global history, velocity_history, position_history, omega_history, rpy_history, acceleration_history, force_history, torque_history, count, velocity_ax, previous_velocity
    
    # Plot history trajectory
    history[count] = frame[:,4]
    
    # Get and store velocity data
    velocity = env.quadcopter.velocity()
    velocity_history[count] = velocity
    
    # Calculate and store acceleration data
    dt = 1.0/200.0  # Use the simulation timestep
    acceleration = calculate_linear_acceleration(velocity, previous_velocity, dt)
    acceleration_history[count] = acceleration
    previous_velocity = velocity.copy()
    
    # Get and store position data
    position = env.quadcopter.position()
    position_history[count] = position
    
    # Get and store omega data
    omega = env.quadcopter.omega()
    omega_history[count] = omega
    
    # Get and store RPY data
    quat = env.quadcopter.state[6:10]
    roll, pitch, yaw = quaternion_to_rpy(quat[0], quat[1], quat[2], quat[3], degrees=False)
    rpy_history[count] = [roll, pitch, yaw]
    
    # Get and store force data
    force = env.F
    force_history[count] = force
    
    # Get and store torque data
    torque = env.M.flatten()
    torque_history[count] = torque
    
    if count < np.size(history, 0) - 1:
        count += 1
    
    # Update position history
    zline = history[:count,-1]
    xline = history[:count,0]
    yline = history[:count,1]
    lines[4].set_data(xline, yline)  # Use index 4 for history line
    lines[4].set_3d_properties(zline)
    
    # Update 2D plots only if they are enabled
    if plots2d_enabled:
        # Update velocity plot
        update_velocity_plot()
        
        # Update position plot
        update_position_plot()
        
        # Update omega plot
        update_omega_plot()
        
        # Update RPY plot
        update_rpy_plot()
        
        # Update acceleration plot
        update_acceleration_plot()
        
        # Update force plot
        update_force_plot()
        
        # Update torque plot
        update_torque_plot()
        
        # Redraw the plots figure
        global fig_plots_global
        if fig_plots_global is not None:
            fig_plots_global.canvas.draw_idle()

def update_velocity_plot():
    """Update the velocity plot with current data"""
    global velocity_history, count, velocity_ax, timesteps_per_frame_global, plots2d_enabled
    
    if not plots2d_enabled or velocity_ax is None or count == 0:
        return
        
    time_steps = np.arange(count) * timesteps_per_frame_global
    vel_x = velocity_history[:count, 0]
    vel_y = velocity_history[:count, 1] 
    vel_z = velocity_history[:count, 2]
    
    # Get velocity plot lines
    vel_lines = velocity_ax.get_lines()
    
    # Update velocity lines
    vel_lines[0].set_data(time_steps, vel_x)  # Vel X
    vel_lines[1].set_data(time_steps, vel_y)  # Vel Y  
    vel_lines[2].set_data(time_steps, vel_z)  # Vel Z
    
    # Auto-scale axes
    if count > 1:
        velocity_ax.set_xlim(0, count)
        all_velocities = np.concatenate([vel_x, vel_y, vel_z])
        vel_min, vel_max = np.min(all_velocities), np.max(all_velocities)
        margin = (vel_max - vel_min) * 0.1
        velocity_ax.set_ylim(vel_min - margin, vel_max + margin)

def update_position_plot():
    """Update the position plot with current data"""
    global position_history, count, position_ax, timesteps_per_frame_global, plots2d_enabled
    
    if not plots2d_enabled or position_ax is None or count == 0:
        return
        
    time_steps = np.arange(count) * timesteps_per_frame_global
    pos_x = position_history[:count, 0]
    pos_y = position_history[:count, 1] 
    pos_z = position_history[:count, 2]
    
    # Get position plot lines
    pos_lines = position_ax.get_lines()
    
    # Update position lines
    pos_lines[0].set_data(time_steps, pos_x)  # Pos X
    pos_lines[1].set_data(time_steps, pos_y)  # Pos Y  
    pos_lines[2].set_data(time_steps, pos_z)  # Pos Z
    
    # Auto-scale axes
    if count > 1:
        position_ax.set_xlim(0, count * timesteps_per_frame_global)
        all_positions = np.concatenate([pos_x, pos_y, pos_z])
        pos_min, pos_max = np.min(all_positions), np.max(all_positions)
        margin = (pos_max - pos_min) * 0.1
        position_ax.set_ylim(pos_min - margin, pos_max + margin)

def update_omega_plot():
    """Update the omega plot with current data"""
    global omega_history, count, omega_ax, timesteps_per_frame_global, plots2d_enabled
    
    if not plots2d_enabled or omega_ax is None or count == 0:
        return
        
    time_steps = np.arange(count) * timesteps_per_frame_global
    omega_x = omega_history[:count, 0]
    omega_y = omega_history[:count, 1] 
    omega_z = omega_history[:count, 2]
    
    # Get omega plot lines
    omega_lines = omega_ax.get_lines()
    
    # Update omega lines
    omega_lines[0].set_data(time_steps, omega_x)  # Omega X
    omega_lines[1].set_data(time_steps, omega_y)  # Omega Y  
    omega_lines[2].set_data(time_steps, omega_z)  # Omega Z
    
    # Auto-scale axes
    if count > 1:
        omega_ax.set_xlim(0, count * timesteps_per_frame_global)
        all_omegas = np.concatenate([omega_x, omega_y, omega_z])
        omega_min, omega_max = np.min(all_omegas), np.max(all_omegas)
        margin = (omega_max - omega_min) * 0.1
        omega_ax.set_ylim(omega_min - margin, omega_max + margin)

def update_rpy_plot():
    """Update the RPY plot with current data"""
    global rpy_history, count, rpy_ax, timesteps_per_frame_global, plots2d_enabled
    
    if not plots2d_enabled or rpy_ax is None or count == 0:
        return
        
    time_steps = np.arange(count) * timesteps_per_frame_global
    roll = rpy_history[:count, 0]
    pitch = rpy_history[:count, 1] 
    yaw = rpy_history[:count, 2]
    
    # Get RPY plot lines
    rpy_lines = rpy_ax.get_lines()
    
    # Update RPY lines
    rpy_lines[0].set_data(time_steps, roll)  # Roll
    rpy_lines[1].set_data(time_steps, pitch)  # Pitch
    rpy_lines[2].set_data(time_steps, yaw)  # Yaw
    
    # Auto-scale axes
    if count > 1:
        rpy_ax.set_xlim(0, count * timesteps_per_frame_global)
        all_rpys = np.concatenate([roll, pitch, yaw])
        rpy_min, rpy_max = np.min(all_rpys), np.max(all_rpys)
        margin = (rpy_max - rpy_min) * 0.1
        rpy_ax.set_ylim(rpy_min - margin, rpy_max + margin)

def update_acceleration_plot():
    """Update the acceleration plot with current data"""
    global acceleration_history, count, accel_ax, timesteps_per_frame_global, plots2d_enabled
    
    if not plots2d_enabled or accel_ax is None or count == 0:
        return
        
    time_steps = np.arange(count) * timesteps_per_frame_global
    accel_x = acceleration_history[:count, 0]
    accel_y = acceleration_history[:count, 1] 
    accel_z = acceleration_history[:count, 2]
    
    # Get acceleration plot lines
    accel_lines = accel_ax.get_lines()
    
    # Update acceleration lines
    accel_lines[0].set_data(time_steps, accel_x)  # Accel X
    accel_lines[1].set_data(time_steps, accel_y)  # Accel Y  
    accel_lines[2].set_data(time_steps, accel_z)  # Accel Z
    
    # Auto-scale axes
    if count > 1:
        accel_ax.set_xlim(0, count * timesteps_per_frame_global)
        all_accels = np.concatenate([accel_x, accel_y, accel_z])
        accel_min, accel_max = np.min(all_accels), np.max(all_accels)
        margin = (accel_max - accel_min) * 0.1
        accel_ax.set_ylim(accel_min - margin, accel_max + margin)

def update_force_plot():
    """Update the force plot with current data"""
    global force_history, count, force_ax, timesteps_per_frame_global, plots2d_enabled
    
    if not plots2d_enabled or force_ax is None or count == 0:
        return
        
    time_steps = np.arange(count) * timesteps_per_frame_global
    force_mag = force_history[:count, 0]
    
    # Get force plot lines
    force_lines = force_ax.get_lines()
    
    # Update force line
    force_lines[0].set_data(time_steps, force_mag)  # Force magnitude
    
    # Auto-scale axes
    if count > 1:
        force_ax.set_xlim(0, count * timesteps_per_frame_global)
        force_min, force_max = np.min(force_mag), np.max(force_mag)
        margin = (force_max - force_min) * 0.1
        force_ax.set_ylim(force_min - margin, force_max + margin)

def update_torque_plot():
    """Update the torque plot with current data"""
    global torque_history, count, torque_ax, timesteps_per_frame_global, plots2d_enabled
    
    if not plots2d_enabled or torque_ax is None or count == 0:
        return
        
    time_steps = np.arange(count) * timesteps_per_frame_global
    torque_x = torque_history[:count, 0]
    torque_y = torque_history[:count, 1] 
    torque_z = torque_history[:count, 2]
    
    # Get torque plot lines
    torque_lines = torque_ax.get_lines()
    
    # Update torque lines
    torque_lines[0].set_data(time_steps, torque_x)  # Torque X
    torque_lines[1].set_data(time_steps, torque_y)  # Torque Y  
    torque_lines[2].set_data(time_steps, torque_z)  # Torque Z
    
    # Auto-scale axes
    if count > 1:
        torque_ax.set_xlim(0, count * timesteps_per_frame_global)
        all_torques = np.concatenate([torque_x, torque_y, torque_z])
        torque_min, torque_max = np.min(all_torques), np.max(all_torques)
        margin = (torque_max - torque_min) * 0.1
        torque_ax.set_ylim(torque_min - margin, torque_max + margin)