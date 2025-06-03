import numpy as np
from scipy.linalg import solve_discrete_are, solve
from scipy.integrate import ode
import matplotlib.pyplot as plt

import meshcat
from meshcat.animation import Animation

# nonlinear continuous-time dynamics
m_b = 2.5  # mass of the ball
m_t = 1.0  # mass of the top
J_b = 12.5  # moment of inertia of the ball
R = 0.16  # radius of the ball
l = 0.6  # length of the top
g = 9.81  # acceleration due to gravity

def f(t, x, u):
    x1, x2, x3, x4 = x
    return np.array(
        [
            x2,
            (-R**2 * (u + g * m_t * np.cos(x3) * np.sin(x3))) / (J_b + R**2 * (m_b + m_t - m_t * np.cos(x3)**2)),
            x4,
            (R**2 * u * np.cos(x3) + J_b * g * np.sin(x3) + R**2 * g * (m_b + m_t) * np.sin(x3)) / (l*(J_b + R**2 * (m_b + m_t - m_t * np.cos(x3)**2)))
        ]
    )

#Continuous-time system matrices
A_con = np.array([[0, 1, 0, 0],
              [0, 0, -0.01999, 0],
              [0, 0, 0, 1],
              [0, 0, 16.38, 0]])

B_con = np.array([[0],
              [-0.002038],
              [0],
              [0.003396]])

# linearized discrete-time dynamics
h = 1e-2  # time step

A = np.eye(4) + A_con * h   #Euler discretization
B = B_con * h               #Euler discretization


# simulation
N = 500 # number of time steps

x0 = np.array([0.0, 0.0, 0.0, 0.0])  # initial state

x_eq = np.array([0.0, 0.0, 0.0, 0.0])  # equilibrium state
u_eq = np.array([0.0])  # equilibrium input

xs = np.zeros((4, N + 1))  # state trajectory
us = np.zeros((1, N))  # input trajectory

#  infinite horizon
#   problem
Q = np.diag([10, 1, 100, 10])  # state cost matrix   [pos, vel, angle, ang_vel]
R = np.array([1])  # input cost matrix
    #Pokud bude systém pomalý, zmenším R na 0.1, pokud bude rychlý, zvětším R na 10

# controller
S = solve_discrete_are(A, B, Q, R)  # solution to the discrete-time algebraic Riccati equation
K = solve(R + B.T @ S @ B, B.T @ S @ A)  # optimal gain matrix