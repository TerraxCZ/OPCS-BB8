import numpy as np
import scipy.sparse as sp
import matplotlib.pyplot as plt
import osqp

# === Časový krok simulace ===
dt = 0.005  # 5 ms

# === Kontinuální matice A a B (převzato z MATLABu) ===
A = np.array([
    [0, 1.0000,  0,       0],
    [0, 0,      -0.0200,  0],
    [0, 0,       0,       1.0000],
    [0, 0,      16.3833,  0]
])

B = np.array([
    [0],
    [0.002],
    [0],
    [-0.0034]
])

# === Eulerova diskretizace ===
Ad = np.eye(4) + A * dt
Bd = B * dt

# === Rozměry systému ===
n = 4   # počet stavů
m = 1   # počet vstupů (jedna síla)

# === Parametry MPC ===
N = 30       # MPC horizont
M = 200      # počet kroků simulace
x0 = np.array([0.51, 0, 0.5, 0])  # počáteční stav: malá odchylka
u_max = 1   # omezení síly (±1 N)

# === Váhovací matice pro stav a vstup ===
Q = np.diag([10, 1, 100, 1])   # vyšší váha na polohu a úhel
R = 0.01 * np.eye(m)

# === OSQP objektivní funkce (P matice) ===
P = sp.block_diag([
    sp.kron(sp.eye(N), R),
    sp.kron(sp.eye(N), Q)
], format='csc')

q = np.zeros(N * m + N * n)

# === Omezení ===
Ax = sp.kron(sp.eye(N), -np.eye(n)) + sp.kron(sp.eye(N, k=-1), Ad)
Bu = sp.kron(sp.eye(N), Bd)
Aeq = sp.hstack([Bu, Ax])
Aineq = sp.vstack([
    Aeq,
    sp.hstack([sp.eye(N * m), sp.csr_matrix((N * m, N * n))]),
    sp.hstack([-sp.eye(N * m), sp.csr_matrix((N * m, N * n))])
]).tocsc()

# === Spodní a horní meze ===
l = np.zeros(Aineq.shape[0])
u = np.zeros(Aineq.shape[0])

# === Vstupní omezení ===
l[N * n:] = -u_max
u[N * n:] = u_max

# === OSQP setup ===
model = osqp.OSQP()
model.setup(P=P, q=q, A=Aineq, l=l, u=u, verbose=False)

# === Simulace ===
xs = np.zeros((n, M + 1))
us = np.zeros((m, M))
xs[:, 0] = x0

for i in range(M):
    # Nastavení rovnosti: x₀ = aktuální stav
    l[:n] = -Ad @ xs[:, i]
    u[:n] = -Ad @ xs[:, i]

    # Aktualizace modelu
    model.update(l=l, u=u)

    # Vyřešení QP
    res = model.solve()
    if res.info.status != 'solved':
        print("OSQP neselhal – MPC nevyřešeno")
        break

    # Aplikace prvního vstupu
    u_opt = res.x[:m]
    us[:, i] = u_opt
    xs[:, i+1] = Ad @ xs[:, i] + Bd.flatten() * u_opt  # přechod stavu

# === Graf výstupů ===
plt.figure(figsize=(10, 6))

plt.subplot(2, 1, 1)
for j in range(n):
    plt.plot(xs[j, :], label=f'x{j+1}')
plt.title("Stavy systému")
plt.xlabel("Krok")
plt.ylabel("x")
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(us[0, :], label="u")
plt.title("Řídicí vstup")
plt.xlabel("Krok")
plt.ylabel("u")
plt.legend()

plt.tight_layout()
plt.show()