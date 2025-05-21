import osqp
import numpy as np
import scipy.sparse as sp
import matplotlib.pyplot as plt

# Discrete time state-space representation
dt = 1e-2
A = np.eye(4) + np.array([[0, 1.0000, 0,0], [0, 0, -0.0200, 0], [0, 0, 0, 1.0000], [0, 0, 16.3833, 0]]) * dt
B = np.array([[0, 0], [0, 0], [1, 0], [0, 1]]) * dt

n = 4  # number of states
m = 2  # number of inputs

# Optimal control problem
N = 100  # MPC horizon
M = 1000  # total number of steps

x0 = np.array([2, 1, 0, 0])  # initial state
u_max = 2  # symmetric upper and lower bound on inputs

Q = 1e0 * np.eye(4)
R = 1e-2 * np.eye(2)

# QP model of the problem
## Objective matrix (P)
objective_matrix = sp.lil_matrix((N * (m + n), N * (m + n)))
objective_matrix[0:N*m, 0:N*m] += np.kron(np.eye(N), R)  # input x input
objective_matrix[N*m:, N*m:] += np.kron(np.eye(N), Q)    # state x state

## Objective vector (q)
objective_vector = np.zeros(N * m + N * n)

## Constraint matrix (A)
constraint_matrix = sp.lil_matrix((N * (n + m), N * (n + m)))
constraint_matrix[0:N*n, 0:N*m] += np.kron(np.eye(N), B)  # state-constraints x input
constraint_matrix[0:N*n, N*m:] += np.kron(np.eye(N), -np.eye(n))  # state-constraints x future state
constraint_matrix[n:N*n, N*m:-n] += np.kron(np.eye(N-1), A)  # state-constraint x current state
constraint_matrix[N*n:, 0:N*m] += np.kron(np.eye(N), np.eye(m))  # input-constraints x input

## Lower bounds (l)
lower_bounds = np.zeros(N * (n + m))
lower_bounds[N * n:] = -u_max # input-constraints

## Upper bounds (u)
upper_bounds = np.zeros(N * (n + m))
upper_bounds[N * n:] = u_max # input-constraints

# OSQP model setup
model = osqp.OSQP()

model.setup(
    P=objective_matrix.tocsc(), q=objective_vector,
    A=constraint_matrix.tocsc(), l=lower_bounds, u=upper_bounds,
    verbose=False
)

# MPC simulation
## pre-allocation
xs = np.zeros((n, M + 1))
us = np.zeros((m, M))

## initial state
xs[:, 0] = x0

## simulation loop
for i in range(M):
    # initial state condition
    lower_bounds[:n] = -A @ xs[:, i]
    upper_bounds[:n] = -A @ xs[:, i]
    model.update(l=lower_bounds, u=upper_bounds)

    # MPC calculation
    results = model.solve()

    # policy application
    us[:, i] = results.x[:m]  # first MPC input assigned as the current input
    noise = np.random.uniform(-1, 1, 2)  # random input noise
    xs[:, i + 1] = A @ xs[:, i] + B @ (us[:, i] + noise)  # next state

# Visualization
plt.subplot(2, 1, 1)
for i in range(4):
    plt.plot(xs[i, :], label=f"x{i+1}")
plt.legend()

plt.subplot(2, 1, 2)
for i in range(2):
    plt.plot(us[i, :], label=f"u{i+1}")
plt.legend()

plt.show()

