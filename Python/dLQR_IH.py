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

print("A:", A)