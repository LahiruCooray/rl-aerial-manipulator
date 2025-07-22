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
"""

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import cnames
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys

history = np.zeros((500,3))
count = 0

def plot_quad_3d(waypoints, get_world_frame):
    """
    get_world_frame is a function which returns the "next" world frame to be drawn
    """
    waypoints_holder = waypoints
    waypoints = waypoints[0]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot placeholders for quadrotor body, arms, trajectory, waypoints, and history
    quad_body, = ax.plot([], [], [], '-', c='green', linewidth=2, label='Quad Body')
    quad_arms, = ax.plot([], [], [], '-', c='red', linewidth=2, label='Quad Arms')
    trajectory, = ax.plot([], [], [], '-', c='blue', marker='o', markevery=2, markersize=5, label='Trajectory')
    waypoints_plot, = ax.plot([], [], [], '.', c='magenta', markersize=6, label='Waypoints')
    history_plot, = ax.plot([], [], [], '--', c='red', markersize=2, label='History')  # Dashed line for history

    # Set axis limits and labels
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(-0.5, 8)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Quadrotor 3D Trajectory')
    ax.legend(loc='upper left')

    # Reset history when simulation starts
    global history, count
    history = np.zeros((500,3))
    count = 0

    plot_waypoints(waypoints)
    an = animation.FuncAnimation(
        fig,
        anim_callback,
        fargs=(get_world_frame, waypoints_holder),
        init_func=None,
        frames=400,
        interval=10,
        blit=False
    )

    if len(sys.argv) > 1 and sys.argv[1] == 'save':
        print("saving")
        an.save('sim.gif', dpi=80, writer='imagemagick', fps=60)
    else:
        plt.show()

def reset_history():
    """Call this function when position is reset to clear history"""
    global history, count
    history = np.zeros((500,3))
    count = 0

def plot_waypoints(waypoints):
    ax = plt.gca()
    lines = ax.get_lines()
    lines[-2].set_data(waypoints[:,0], waypoints[:,1])
    lines[-2].set_3d_properties(waypoints[:,2])

def set_limit(x, y, z):
    ax = plt.gca()
    ax.set_xlim(x)
    ax.set_ylim(y)
    ax.set_zlim(z)

def anim_callback(i, get_world_frame, waypoints_holder):
    frame = get_world_frame(i)
    set_frame(frame)
    plot_waypoints(waypoints_holder[0])

def set_frame(frame):
    # convert 3x6 world_frame matrix into three line_data objects which is 3x2 (row:point index, column:x,y,z)
    lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]
    ax = plt.gca()
    lines = ax.get_lines()
    for line, line_data in zip(lines[:3], lines_data):
        x, y, z = line_data
        line.set_data(x, y)
        line.set_3d_properties(z)

    global history, count
    # plot history trajectory
    history[count] = frame[:,4]
    if count < np.size(history, 0) - 1:
        count += 1
    zline = history[:count,-1]
    xline = history[:count,0]
    yline = history[:count,1]
    lines[-1].set_data(xline, yline)
    lines[-1].set_3d_properties(zline)
    # ax.plot3D(xline, yline, zline, 'blue')
