"""
We will be implementing an Extended Kalman Filter

For now, our robot does not survive kidnapping :o, we can implement this in the future by adding states

The state vector is : [ x_k         : measured by the camera
                        y_k         : measured by the camera
                        theta_k     : measured by the camera
                        vright_k    : measured by the robot
                        vleft_k ]    : measured by the robot

When writing an extended kalman filter, there are two main steps: 
1) Prediction -> use dynamical model to update state estimates using Euler integration
    keep in mind that P will go up here (error covariance)
2) Update -> use measurements from sensor(s) to correct predictions 
    P will decrease here

In summary, the algorithm goes like this:

1. Initialise
    - state estimate x_hat[0]
    - P (error covariance, sigma in class), Q (), R matrixes
    - T sample time

2. Predict
    - new Mu(t)
    - new sigma(t)

3. Update
    - K 
    - Mu_corr(t) 
    - Sigma_corr(t) 
"""
from math import *
import numpy as np
import time
import serial
import matplotlib.pyplot as plt

from Thymio import Thymio


def calculate_jacobians(x, u,Ts):
    """
    Calculate Jacobian matrices A and H for the differential drive motion model.
    """
    wheelbase = 0.095  # Wheelbase of the Thymio robot aka distance between the two drive wheels
    v_right, v_left = u

    # Compute linear and angular velocities
    v = (v_right + v_left) / 2
    omega = (v_right - v_left) / wheelbase

    # Jacobian of the motion model with respect to the state
    A = np.array([ #x = [x_k, y_k, theta_k, v_right_wheel, v_left_wheel]
        [1, 0, Ts * v * np.sin(x[2]), Ts * np.cos(x[2]), Ts * np.cos(x[2])],
        [0, 1, -Ts * v * np.cos(x[2]), Ts * np.sin(x[2]), Ts * np.sin(x[2])],
        [0, 0, 1, -Ts * omega, Ts * omega],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]
    ])

    # Jacobian of the measurement model with respect to the state
    H = np.eye(5)

    return A, H

def motion_model(x, u, Ts):
    """
    Differential drive motion model for the Thymio robot.
    x: state vector [x_k, y_k, theta_k, v_right_wheel, v_left_wheel]
    u: control input [v_right, v_left]
    Ts: time step
    """
    wheelbase = 0.095  # Wheelbase of the Thymio robot (adjust as needed)
    v_right, v_left = u

    # Compute linear and angular velocities
    v = (v_right + v_left) / 2
    omega = (v_right - v_left) / wheelbase

    # Update state
    x[0] += Ts * v * np.cos(x[2])
    x[1] += Ts * v * np.sin(x[2])
    x[2] += Ts * omega
    x[3:] = u  # Update wheel velocities (control)
    return x

#initialisation step

Ts = 0.1

A = np.eye(5)  # Identity matrix since we assume a simple linear system
H = np.eye(5)  # Identity matrix since we directly measure the state
x_hat = np.zeros((5, 1))  # Initial state estimate

P = np.eye(5) * 0.1 #error covariance matrix

# Process noise covariance and measurement noise covariance
# Q and R are diagonal matrices
Q = np.diag([0.1, 0.1, 0.1, 0.01, 0.01])  # To be tuned
R = np.diag([0.1, 0.1, 0.1, 0.01, 0.01])

u = np.array([[0.1], [0.1]])

# Sensor measurements -> i put a Gaussian for now but it should be changed in the future
z = np.random.normal(size=(5, 1))  # Simulated sensor measurements

# Put actual control data here
u = np.array([[0.1], [0.1]])  

# Prediction Step
A, _ = calculate_jacobians(x_hat, u, Ts)
x_hat_minus = motion_model(x_hat, u, Ts)
P_minus = A @ P @ A.T + Q  # Error covariance prediction

# Update Step
_, H = calculate_jacobians(x_hat_minus, u, Ts)
K = P_minus @ H.T @ np.linalg.inv(H @ P_minus @ H.T + R)  # Kalman gain
x_hat = x_hat_minus + K @ (z - H @ x_hat_minus)  # Updated state estimate
P = (np.eye(5) - K @ H) @ P_minus  # Updated error covariance

