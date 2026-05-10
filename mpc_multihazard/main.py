"""
Main entry point — simulasi MPC multi-hazard pada bangunan 3-DOF.

Model struktural: shear building 3 lantai, masing-masing 333.3 kg
  State   : x = [x1, x2, x3, x1_dot, x2_dot, x3_dot]
  Input   : u = [f1, f2, f3]   (gaya aktuator per lantai)
  Gangguan: d = [ag, ff]        (percepatan gempa, gaya fluida banjir)

Frekuensi natural bangunan (dihitung otomatis):
  Mode 1: 1.233 Hz  (fundamental — dipakai untuk eksitasi gempa)
  Mode 2: 3.020 Hz
  Mode 3: 4.775 Hz

Eksitasi gempa pada frekuensi fundamental -> resonansi maksimal -> respons terbesar.
Ts = 0.05 s (20 Hz sampling) cukup untuk menangkap semua mode (Nyquist > 4.775 Hz).
"""

import numpy as np
import matplotlib.pyplot as plt

from src.system_model import build_continuous_matrices, discretize_system
from src.mpc_solver import build_cost_matrices
from src.simulation import (
    generate_earthquake_disturbance,
    generate_flood_disturbance,
    run_simulation,
)
from src.metric import compute_all, compare_controllers


def compute_natural_frequencies(M, K):
    """Hitung frekuensi natural sistem (Hz) dari matriks massa & kekakuan."""
    Minv = np.linalg.inv(M)
    eig_vals = np.linalg.eigvalsh(Minv @ K)
    omega_n  = np.sqrt(np.sort(np.abs(eig_vals)))
    freq_hz  = omega_n / (2.0 * np.pi)
    return freq_hz, omega_n


def main():
    # ── Parameter struktural (3-DOF shear building) ──────────────────────────
    m1 = m2 = m3 = 333.3      # massa tiap lantai (kg)
    k1, k2, k3   = 1.8e5, 8.0e4, 5.0e4   # kekakuan antar lantai (N/m)
    c1, c2, c3   = 500.0, 400.0, 300.0   # redaman antar lantai (Ns/m)

    M = np.diag([m1, m2, m3])

    K = np.array([[k1 + k2,  -k2,       0.0    ],
                  [-k2,       k2 + k3,  -k3    ],
                  [0.0,      -k3,        k3    ]])

    C = np.array([[c1 + c2,  -c2,       0.0    ],
                  [-c2,       c2 + c3,  -c3    ],
                  [0.0,      -c3,        c3    ]])

    # Frekuensi natural
    freq_hz, omega_n = compute_natural_frequencies(M, K)
    f1 = freq_hz[0]   # fundamental — dipakai untuk gempa
    print(f"Natural frequencies: {np.round(freq_hz, 3)} Hz")
    print(f"Earthquake excitation at f1 = {f1:.3f} Hz (resonance)")

    # Aktuator: satu per lantai
    L_u = np.eye(3)

    # Gangguan:
    #   kolom 0 = gempa  -> gaya inersia -m_i * ag per lantai
    #   kolom 1 = banjir -> gaya horizontal di lantai 1 saja
    L_d = np.array([[-m1, 1.0],
                    [-m2, 0.0],
                    [-m3, 0.0]])

    # ── Diskritisasi ─────────────────────────────────────────────────────────
    # Ts=0.05s (20 Hz) cukup: Nyquist=10 Hz >> f_max_mode=4.775 Hz
    Ts = 0.05

    A_cont, B_cont, E_cont = build_continuous_matrices(M, C, K, L_u, L_d)
    Ad, Bd, Ed = discretize_system(A_cont, B_cont, E_cont, Ts)

    n = Ad.shape[0]   # 6 states
    m = Bd.shape[1]   # 3 inputs

    # ── Bobot MPC ────────────────────────────────────────────────────────────
    # State [x1,x2,x3, x1d,x2d,x3d]: penalti disp >> vel
    Q_diag = [1e4, 1e4, 1e4, 1e2, 1e2, 1e2]
    R_diag = [1e-6, 1e-6, 1e-6]
    Q, R   = build_cost_matrices(n, m, Q_diag, R_diag)
    N      = 20    # prediction horizon

    # ── Batas input ──────────────────────────────────────────────────────────
    u_max = np.array([5000.0, 5000.0, 5000.0])   # N
    u_min = -u_max

    # ── Waktu & gangguan ─────────────────────────────────────────────────────
    T_sim = 60.0
    t_seq = np.arange(0, T_sim, Ts)
    T     = len(t_seq)

    # Gempa: eksitasi pada frekuensi fundamental -> resonansi
    eq    = generate_earthquake_disturbance(
                t_seq, magnitude=3.0, t_start=5.0, duration=15.0,
                freq_hz=f1)

    # Banjir: puncak di t=35 s, datang setelah gempa reda
    flood = generate_flood_disturbance(t_seq, peak=2000.0, t_peak=35.0,
                                       t_rise=10.0, decay=0.05)

    d_seq = np.column_stack([eq, flood])   # (T, 2)

    # ── Referensi: regulasi ke equilibrium nol ────────────────────────────────
    x_ref = np.zeros((T, n))
    x0    = np.zeros(n)

    # ── Simulasi MPC ─────────────────────────────────────────────────────────
    print(f"Running MPC simulation — 3-DOF, {T} steps, N={N}...")
    x_hist, u_hist = run_simulation(
        Ad, Bd, Ed, Q, R, N, x0, d_seq, x_ref, u_min, u_max, Ts
    )

    # ── Simulasi Uncontrolled (baseline) ─────────────────────────────────────
    print("Running uncontrolled simulation (baseline)...")
    x_unc = np.zeros((T + 1, n))
    x_unc[0] = x0
    for k in range(T):
        x_unc[k + 1] = Ad @ x_unc[k] + Ed @ d_seq[k]

    t_disturbance_end = 45.0   # setelah gempa+banjir mereda

    # ── Metrik evaluasi [Proposal 3.8] ────────────────────────────────────────
    compute_all(
        x_hist, u_hist,
        Ts                = Ts,
        roof_idx          = 2,    # lantai 3 = atap
        lower_idx         = 1,
        upper_idx         = 2,
        x_uncontrolled    = x_unc,
        t_disturbance_end = t_disturbance_end,
    )

    # ── Perbandingan MPC vs Uncontrolled [Proposal 3.9] ──────────────────────
    compare_controllers(
        {"MPC": x_hist, "Uncontrolled": x_unc},
        Ts=Ts, roof_idx=2
    )

    # ── Plot ─────────────────────────────────────────────────────────────────
    t_full = np.arange(T + 1) * Ts

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    # Displacement semua lantai (MPC vs Uncontrolled untuk lantai 3/atap)
    colors = ["steelblue", "darkorange", "green"]
    for i in range(3):
        axes[0].plot(t_full, x_hist[:, i] * 1e3,
                     color=colors[i], label=f"Floor {i+1} (MPC)")
    axes[0].plot(t_full, x_unc[:, 2] * 1e3,
                 color="red", linestyle="--", alpha=0.6, label="Floor 3 (Uncontrolled)")
    axes[0].set_ylabel("Displacement (mm)")
    axes[0].set_title(f"3-DOF MPC | Earthquake f={f1:.2f} Hz (resonance) + Flood")
    axes[0].legend(fontsize=8)
    axes[0].grid(True)

    # Control forces
    for i in range(3):
        axes[1].plot(t_seq, u_hist[:, i], color=colors[i],
                     label=f"u{i+1} (Floor {i+1})")
    axes[1].set_ylabel("Control Force (N)")
    axes[1].legend(fontsize=8)
    axes[1].grid(True)

    # Disturbance
    axes[2].plot(t_seq, eq,    label=f"Earthquake (m/s^2) @ {f1:.2f} Hz")
    axes[2].plot(t_seq, flood, label="Flood force (N)", linestyle="--")
    axes[2].set_ylabel("Disturbance")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend()
    axes[2].grid(True)

    plt.tight_layout()
    plt.savefig("results_3dof.png", dpi=150)
    plt.show()
    print("Plot saved -> results_3dof.png")


if __name__ == "__main__":
    main()
