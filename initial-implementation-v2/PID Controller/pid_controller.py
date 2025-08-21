"""
author: Peter Huang, Antonio Cuni
email: hbd730@gmail.com, anto.cuni@gmail.com
license: BSD
Please feel free to use and modify this, but keep the above information. Thanks!


Modified by Methsara Shamika in year 2025

"""

import numpy as np
import model.params as params
from math import sin, cos

# PID Gains
k_p_x, k_d_x, k_i_x = 3, 30, 1
k_p_y, k_d_y, k_i_y = 3, 30, 1
k_p_z, k_d_z, k_i_z = 1000, 200, 10
k_p_phi, k_d_phi, k_i_phi = 160, 3, 1
k_p_theta, k_d_theta, k_i_theta = 160, 3, 1
k_p_psi, k_d_psi, k_i_psi = 80, 5, 1

# Integral error memory
integral_error = {
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
    'phi': 0.0,
    'theta': 0.0,
    'psi': 0.0,
}

# Anti-windup limits (Prevents error from accumulating too much over time.)
MAX_INTEGRAL = 100

def run(quad, des_state, dt):
    # Current state
    x, y, z = quad.position()
    x_dot, y_dot, z_dot = quad.velocity()
    phi, theta, psi = quad.attitude()
    p, q, r = quad.omega()

    # Desired state
    des_x, des_y, des_z = des_state.pos
    des_x_dot, des_y_dot, des_z_dot = des_state.vel
    des_x_ddot, des_y_ddot, des_z_ddot = des_state.acc
    des_psi = des_state.yaw
    des_psi_dot = des_state.yawdot

    # Errors
    err_x = des_x - x
    err_y = des_y - y
    err_z = des_z - z
    err_x_dot = des_x_dot - x_dot
    err_y_dot = des_y_dot - y_dot
    err_z_dot = des_z_dot - z_dot

    # Update integral errors
    integral_error['x'] += err_x * dt
    integral_error['y'] += err_y * dt
    integral_error['z'] += err_z * dt

    # Clamp integrals
    for key in ['x', 'y', 'z']:
        integral_error[key] = np.clip(integral_error[key], -MAX_INTEGRAL, MAX_INTEGRAL)

    # Compute commanded accelerations with PID
    commanded_r_ddot_x = (des_x_ddot
                          + k_d_x * err_x_dot
                          + k_p_x * err_x
                          + k_i_x * integral_error['x'])

    commanded_r_ddot_y = (des_y_ddot
                          + k_d_y * err_y_dot
                          + k_p_y * err_y
                          + k_i_y * integral_error['y'])

    commanded_r_ddot_z = (des_z_ddot
                          + k_d_z * err_z_dot
                          + k_p_z * err_z
                          + k_i_z * integral_error['z'])

    # Thrust
    F = params.mass * (params.g + commanded_r_ddot_z)

    # Desired angles from commanded accelerations (Roll and Pitch from X/Y Acceleration)
    des_phi = 1 / params.g * (commanded_r_ddot_x * sin(des_psi) - commanded_r_ddot_y * cos(des_psi))
    des_theta = 1 / params.g * (commanded_r_ddot_x * cos(des_psi) + commanded_r_ddot_y * sin(des_psi))
    des_psi_err = des_psi - psi

    # Angular errors
    err_phi = des_phi - phi
    err_theta = des_theta - theta
    err_psi = des_psi_err
    err_p = 0 - p
    err_q = 0 - q
    err_r = des_psi_dot - r

    # Update angle integrals
    integral_error['phi'] += err_phi * dt
    integral_error['theta'] += err_theta * dt
    integral_error['psi'] += err_psi * dt

    for key in ['phi', 'theta', 'psi']:
        integral_error[key] = np.clip(integral_error[key], -MAX_INTEGRAL, MAX_INTEGRAL)

    # Moment vector with PID
    M = np.array([
        k_p_phi * err_phi + k_d_phi * err_p + k_i_phi * integral_error['phi'],
        k_p_theta * err_theta + k_d_theta * err_q + k_i_theta * integral_error['theta'],
        k_p_psi * err_psi + k_d_psi * err_r + k_i_psi * integral_error['psi']
    ]).reshape((3, 1))

    return F, M
