import numpy as np
from scipy.linalg import solve_discrete_are, solve
from scipy.integrate import ode
import matplotlib.pyplot as plt

import meshcat
from meshcat.animation import Animation

import time

start = time.time() # start timer

# nonlinear continuous-time dynamics
m_b = 2.5  # mass of the ball
m_t = 1.0  # mass of the top
#J_b = 12.5  # moment of inertia of the ball
J_b = 0.085  # moment of inertia of the ball (asi zde byla chyba. vypočten podle vzorce J=2/3*m*R^2)
R_b = 0.16  # radius of the ball
l = 0.6  # length of the top
g = 9.81  # acceleration due to gravity

def f(t, x, u):
    #print(f"t={t}, x={x}, u={u}, type(u)={type(u)}, shape(u)={np.shape(u)}")
    u = float(u)
    x1, x2, x3, x4 = [float(xx) for xx in x]  # zajistí, že všechny jsou float
    result = [
        x2,
        (-R_b**2 * (u + g * m_t * np.cos(x3) * np.sin(x3))) / (J_b + R_b**2 * (m_b + m_t - m_t * np.cos(x3)**2)),
        x4,
        (R_b**2 * u * np.cos(x3) + J_b * g * np.sin(x3) + R_b**2 * g * (m_b + m_t) * np.sin(x3)) / (l*(J_b + R_b**2 * (m_b + m_t - m_t * np.cos(x3)**2)))
    ]
    #print("result types:", [type(val) for val in result])
    #print(result)
    return np.array(result)

#Continuous-time system matrices
A_con = np.array([[0, 1, 0, 0],
                  [0, 0, -1.6855, 0],
                  [0, 0, 0, 1],
                  [0, 0, 19.1591, 0]])

B_con = np.array([[0],
                  [-0.1718],
                  [0],
                  [0.2864]])

# linearized discrete-time dynamics
h = 1e-2  # time step

A = np.eye(4) + A_con * h   #Euler discretization
B = B_con * h               #Euler discretization


# simulation
N = 500 # number of time steps

# initial state
x0 = np.array([0.0, 0.0, np.deg2rad(5), 0.0])  # initial state stabilization from an angle of 5 degrees
#x0 = np.array([1, 0.0, 0.0, 0.0])  # initial state stabilization from a distance of 1 m

x_eq = np.array([0.0, 0.0, 0.0, 0.0])  # equilibrium state
u_eq = np.array([0.0])  # equilibrium input

xs = np.zeros((4, N + 1))  # state trajectory
us = np.zeros((1, N))  # input trajectory

#  infinite horizon
#   problem
Q = np.diag([500, 10, 100, 10])  # state cost matrix   [pos, vel, angle, ang_vel]git
R = np.array([1])  # input cost matrix
    #Pokud bude systém pomalý, zmenším R na 0.1, pokud bude rychlý, zvětším R na 10


# controller
S = solve_discrete_are(A, B, Q, R)  # solution to the discrete-time algebraic Riccati equation
K = solve(R + B.T @ S @ B, B.T @ S @ A)  # optimal gain matrix

# simulation
solver = ode(f).set_integrator("dopri5")  # set up the ODE solver

xs[:, 0] = x0  # set initial state

for k in range(N):
    solver.set_initial_value(xs[:, k])  # reset initial conditions to last state
    us[:, k] = u_eq - K @ (xs[:, k] - x_eq)  # calculate control input
    solver.set_f_params(us[0, k])  # set control input in solver
    solver.integrate(h)  # integrate a single step
    xs[:, k + 1] = solver.y  # save result to states

# print("us:", us)

# Plotting
t = np.arange(N + 1) * h  # časová osa pro stavy
t_u = np.arange(N) * h    # časová osa pro vstupy

fig, (ax1, ax2) = plt.subplots(2, 1)

for i in range(xs.shape[0]):
    ax1.plot(t, xs[i, :], label=f"x{i+1}")

for i in range(us.shape[0]):
    ax2.plot(t_u, us[i, :], label=f"u{i+1}")

ax1.legend(loc='upper right')
ax2.legend()
ax1.grid(True)
ax2.grid(True)
ax1.set_title("State Trajectories")
ax2.set_title("Input Trajectories")
ax1.set_ylabel("x1 [m], x2 [m/s], x3 [rad], x4 [rad/s]")
ax2.set_xlabel("t [s]")
ax2.set_ylabel("M [Nm]")


plt.show()

end = time.time()  # end timer
print(f"Time taken: {end - start:.4f} s")   # print time taken