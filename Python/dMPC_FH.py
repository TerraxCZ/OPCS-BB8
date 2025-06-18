import osqp
import numpy as np
import scipy.sparse as sp
import matplotlib.pyplot as plt
import time

start = time.time()  # start timer

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

n = 4  # number of states
m = 1  # number of inputs

# Optimal control problem
N = 100  # MPC horizon
M = 1500  # total number of steps

# initial state
x0 = np.array([0.0, 0.0, np.deg2rad(5), 0.0])  # initial state stabilization from an angle of 5 degrees
u_max = 6  # symmetric input limits

Q = np.diag([500, 10, 100, 10])  # state cost matrix   [pos, vel, angle, ang_vel]git
R = np.array([1])  # input cost matrix

# QP model of the problem
## Objective matrix (P)
objective_matrix = sp.lil_matrix((N * (m + n), N * (m + n)))    # input x state
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
USE_NOISE = Truegit  # Pokud True, bude přidán šum do vstupu

for i in range(M):
    # initial state condition
    lower_bounds[:n] = -A @ xs[:, i]
    upper_bounds[:n] = -A @ xs[:, i]
    model.update(l=lower_bounds, u=upper_bounds)

    # MPC calculation
    results = model.solve()

    # policy application
    us[:, i] = results.x[:m]  # first MPC input assigned as the current input
    if USE_NOISE:
        noise = np.random.uniform(-1, 1, 2)
        noise = [noise[0]]
    else:
        noise = [0]

    # NELINEÁRNÍ MODEL - Eulerův krok
    xs[:, i + 1] = xs[:, i] + h * f(0, xs[:, i], us[:, i] + noise[0])  # použije pouze první složku šumu

# Visualization
t = np.arange(M + 1) * h
t_u = np.arange(M) * h

plt.figure(figsize=(10, 6))

# Dynamický podnadpis podle šumu
noise_str = "with noise" if USE_NOISE else "without noise"

plt.subplot(2, 1, 1)
for i in range(4):
    labels = ["x1", "x2", "x3", "x4"]
    plt.plot(t, xs[i, :], label=labels[i])
plt.legend()
plt.grid(True)
plt.title(f"State Trajectories ({noise_str})")
plt.ylabel("x1 [m], x2 [m/s], x3 [rad], x4 [rad/s]")

plt.subplot(2, 1, 2)
plt.plot(t_u, us[0, :], label="u1")
plt.legend()
plt.grid(True)
plt.title(f"Input Trajectories ({noise_str})")
plt.xlabel("t [s]")
plt.ylabel("M [Nm]")

plt.tight_layout()
plt.show()

end = time.time()  # end timer
print(f"Time taken: {end - start:.4f} s")   # print time taken