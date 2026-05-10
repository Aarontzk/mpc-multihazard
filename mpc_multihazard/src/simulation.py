"""
Skenario simulasi multi-hazard: gempa bumi + banjir
Loop closed-loop MPC dengan preview gangguan.
"""

import numpy as np
from .mpc_solver import solve_mpc


def generate_earthquake_disturbance(t_seq, magnitude=1.0, t_start=5.0, duration=10.0,
                                    freq_hz=2.0):
    """
    Profil gangguan gempa: sinusoidal teredam (damped sine).
    Merepresentasikan percepatan tanah (m/s^2).

    Args:
        t_seq     : array waktu (T,)
        magnitude : amplitudo puncak (m/s^2)
        t_start   : waktu mulai gempa (s)
        duration  : durasi gempa (s)
        freq_hz   : frekuensi dominan sinusoid (Hz), default 2 Hz

    Returns:
        d (T,)
    """
    omega  = 2.0 * np.pi * freq_hz   # frekuensi dominan
    zeta   = 0.3                      # damping ratio envelope
    t_seq  = np.asarray(t_seq)

    mask   = (t_seq >= t_start) & (t_seq <= t_start + duration)
    tau    = np.where(mask, t_seq - t_start, 0.0)
    d      = np.where(mask,
                      magnitude * np.exp(-zeta * omega * tau) * np.sin(omega * tau),
                      0.0)
    return d


def generate_flood_disturbance(t_seq, peak=1.0, t_peak=30.0, t_rise=10.0, decay=0.05):
    """
    Profil gangguan banjir: ramp naik linear, lalu eksponensial turun.
    Merepresentasikan gaya fluida horizontal (N).

    Args:
        t_seq  : array waktu (T,)
        peak   : gaya puncak (N)
        t_peak : waktu puncak (s)
        t_rise : durasi kenaikan sebelum puncak (s)
        decay  : laju penurunan eksponensial (1/s)

    Returns:
        d (T,)
    """
    t_seq   = np.asarray(t_seq)
    t_start = t_peak - t_rise

    d = np.where(
        t_seq < t_start, 0.0,
        np.where(
            t_seq < t_peak,
            peak * (t_seq - t_start) / t_rise,
            peak * np.exp(-decay * (t_seq - t_peak))
        )
    )
    return d


def run_simulation(Ad, Bd, Ed, Q, R, N, x0, disturbance_seq, x_ref, u_min, u_max, dt):
    """
    Loop simulasi closed-loop MPC.

    Pada setiap langkah k:
      1. Ambil preview gangguan d_seq[k : k+N] (zero-pad jika kurang)
      2. Solve MPC → u_opt
      3. Propagasi state: x_{k+1} = Ad x_k + Bd u_opt + Ed d_k

    Args:
        Ad, Bd, Ed      : matriks diskrit
        Q, R            : bobot cost
        N               : prediction horizon
        x0              : state awal (n,)
        disturbance_seq : gangguan (T x nd)
        x_ref           : referensi state (T x n), target = nol untuk regulasi
        u_min, u_max    : batas input (m,)
        dt              : sampling time (tidak dipakai di sini, disimpan untuk plot)

    Returns:
        x_history (T+1 x n), u_history (T x m)
    """
    disturbance_seq = np.atleast_2d(disturbance_seq)   # (T, nd)
    x_ref           = np.atleast_2d(x_ref)             # (T, n)

    T  = disturbance_seq.shape[0]
    n  = Ad.shape[0]
    m  = Bd.shape[1]
    nd = Ed.shape[1]

    x_history = np.zeros((T + 1, n))
    u_history = np.zeros((T, m))
    x_history[0] = x0

    for k in range(T):
        # Preview gangguan untuk horizon N
        d_window = disturbance_seq[k : k + N]
        if len(d_window) < N:
            pad      = np.zeros((N - len(d_window), nd))
            d_window = np.vstack([d_window, pad])

        # Referensi untuk horizon N
        r_window = x_ref[k : k + N]
        if len(r_window) < N:
            pad      = np.tile(x_ref[-1], (N - len(r_window), 1))
            r_window = np.vstack([r_window, pad])

        # Solve & apply
        u_opt       = solve_mpc(Ad, Bd, Ed, Q, R, N,
                                x_history[k], d_window, r_window, u_min, u_max)
        u_history[k] = u_opt

        # Propagasi state (true dynamics)
        x_history[k + 1] = (Ad @ x_history[k]
                            + Bd @ u_opt
                            + Ed @ disturbance_seq[k])

    return x_history, u_history
