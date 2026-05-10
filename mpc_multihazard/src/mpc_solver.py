"""
MPC Solver — finite-horizon QP via CVXPY (OSQP backend)
Bobot: Q (state cost), R (input cost), N (prediction horizon)
"""

import numpy as np
import cvxpy as cp


def build_cost_matrices(n, m, Q_diag, R_diag):
    """
    Args:
        n       : dimensi state
        m       : dimensi input
        Q_diag  : list bobot diagonal Q, panjang n
        R_diag  : list bobot diagonal R, panjang m
    Returns:
        Q (n x n), R (m x m)
    """
    Q = np.diag(Q_diag)
    R = np.diag(R_diag)
    return Q, R


def solve_mpc(Ad, Bd, Ed, Q, R, N, x0, d_seq, x_ref, u_min, u_max):
    """
    Selesaikan MPC finite-horizon dengan CVXPY.

    Formulasi:
        min  sum_{k=0}^{N-1} [(x_k - r_k)^T Q (x_k - r_k) + u_k^T R u_k]
             + (x_N - r_{N-1})^T Q (x_N - r_{N-1})
        s.t. x_{k+1} = Ad x_k + Bd u_k + Ed d_k
             u_min <= u_k <= u_max
             x_0 = x0

    Args:
        Ad, Bd, Ed  : matriks diskrit (n x n, n x m, n x nd)
        Q, R        : bobot cost (n x n, m x m)
        N           : prediction horizon (int)
        x0          : state awal (n,)
        d_seq       : sekuens gangguan (N x nd) — bisa zero-padded
        x_ref       : referensi state per langkah (N x n)
        u_min, u_max: batas input (m,)

    Returns:
        u_opt (m,) — kontrol optimal langkah pertama
    """
    n = Ad.shape[0]
    m = Bd.shape[1]

    # Pastikan d_seq dan x_ref shape (N, *)
    d_seq  = np.atleast_2d(d_seq)   # (N, nd)
    x_ref  = np.atleast_2d(x_ref)   # (N, n)

    x = cp.Variable((N + 1, n))
    u = cp.Variable((N, m))

    cost = 0
    constraints = [x[0] == x0]

    for k in range(N):
        dx = x[k] - x_ref[k]
        cost += cp.quad_form(dx, Q) + cp.quad_form(u[k], R)
        constraints += [
            x[k + 1] == Ad @ x[k] + Bd @ u[k] + Ed @ d_seq[k],
            u[k] >= u_min,
            u[k] <= u_max,
        ]

    # Terminal cost (gunakan bobot Q, bukan P terpisah)
    dx_N = x[N] - x_ref[-1]
    cost += cp.quad_form(dx_N, Q)

    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.CLARABEL)

    if prob.status not in ("optimal", "almost_feasible"):
        # Fallback to SCS if CLARABEL fails
        prob.solve(solver=cp.SCS, eps=1e-5, max_iters=10000)

    if prob.status not in ("optimal", "optimal_inaccurate", "solved",
                           "almost_feasible", "solved_inaccurate"):
        raise RuntimeError(f"MPC solve failed: status={prob.status}")

    if u.value is None:
        raise RuntimeError(f"MPC solve returned None (status={prob.status})")

    return u.value[0]
