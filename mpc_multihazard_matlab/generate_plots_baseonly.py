"""Render plots for base_only run (separate from per_floor)."""
from pathlib import Path
import json
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).parent
RES = ROOT / "results"
data = np.load(RES / "matlab_mirror_base_only.npz", allow_pickle=True)
summary = json.loads((RES / "summary_base_only.json").read_text().replace("NaN", "null"))
fn = summary["_meta"]["natural_freq_hz"]

scenarios = ["seismic", "flood", "tsunami", "combined"]
IDX_Z = 6

for sc in scenarios:
    t = data[f"{sc}_t"]
    unc = data[f"{sc}_unc_x"]
    lqr = data[f"{sc}_lqr_x"]
    mpc = data[f"{sc}_mpc_x"]
    u_mpc = data[f"{sc}_mpc_u"]
    u_lqr = data[f"{sc}_lqr_u"]
    ref = data[f"{sc}_ref"]
    h = data[f"{sc}_h"]

    fig, ax = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    for i in range(3):
        ax[i, 0].plot(t, unc[i] * 1e3, label="Open-loop", color="#c0392b", lw=1.0)
        ax[i, 0].plot(t, lqr[i] * 1e3, label="LQR",       color="#2980b9", lw=1.0, alpha=0.8)
        ax[i, 0].plot(t, mpc[i] * 1e3, label="MPC",       color="#27ae60", lw=1.3)
        ax[i, 0].set_ylabel(f"q_{i+1} [mm]")
        ax[i, 0].grid(True, alpha=0.4)
        if i == 0:
            ax[i, 0].set_title(f"BASE-ONLY MLFS - Floor displacements - {sc}", fontsize=11)
            ax[i, 0].legend(loc="best", fontsize=8)
        if i == 0:
            d_unc = unc[0]; d_lqr = lqr[0]; d_mpc = mpc[0]
        else:
            d_unc = unc[i] - unc[i - 1]
            d_lqr = lqr[i] - lqr[i - 1]
            d_mpc = mpc[i] - mpc[i - 1]
        ax[i, 1].plot(t, d_unc * 1e3, color="#c0392b", lw=1.0, label="Open-loop")
        ax[i, 1].plot(t, d_lqr * 1e3, color="#2980b9", lw=1.0, alpha=0.8, label="LQR")
        ax[i, 1].plot(t, d_mpc * 1e3, color="#27ae60", lw=1.3, label="MPC")
        ax[i, 1].set_ylabel(f"drift_{i+1} [mm]")
        ax[i, 1].grid(True, alpha=0.4)
        if i == 0:
            ax[i, 1].set_title("Inter-story drifts", fontsize=11)
    ax[2, 0].set_xlabel("time [s]"); ax[2, 1].set_xlabel("time [s]")
    fig.suptitle(f"BASE-ONLY MLFS - {sc.upper()} - natural freq = {[f'{x:.3f}' for x in fn]} Hz",
                 fontsize=12, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(RES / f"baseonly_{sc}_displacement.png", dpi=140); plt.close(fig)

    fig, ax = plt.subplots(2, 1, figsize=(13, 6), sharex=True)
    t_u = t[:-1]
    ax[0].plot(t_u, u_mpc[0], lw=1.2, color="#e67e22", label="u_MLFS (base)")
    ax[0].plot(t_u, u_mpc[1], lw=1.4, color="black", label="u_FAHFS")
    ax[0].set_ylabel("force [N]")
    ax[0].set_title("BASE-ONLY MPC actuator forces (1 MLFS + 1 FAHFS)")
    ax[0].grid(True, alpha=0.4)
    ax[0].axhline(15000, color="gray", ls=":", lw=0.6); ax[0].axhline(-15000, color="gray", ls=":", lw=0.6)
    ax[0].axhline(1e5,   color="gray", ls="--", lw=0.6); ax[0].axhline(-1e5,   color="gray", ls="--", lw=0.6)
    ax[0].legend(loc="best", fontsize=8)
    e_mpc = np.cumsum(np.sum(u_mpc ** 2, axis=0)) * (t[1] - t[0])
    e_lqr = np.cumsum(np.sum(u_lqr ** 2, axis=0)) * (t[1] - t[0])
    ax[1].plot(t_u, e_mpc, color="#27ae60", lw=1.4, label="MPC")
    ax[1].plot(t_u, e_lqr, color="#2980b9", lw=1.0, alpha=0.8, label="LQR")
    ax[1].set_ylabel("cum. energy [N^2 s]")
    ax[1].set_xlabel("time [s]")
    ax[1].set_title("Cumulative control energy")
    ax[1].grid(True, alpha=0.4); ax[1].legend(loc="best", fontsize=8)
    fig.tight_layout(); fig.savefig(RES / f"baseonly_{sc}_forces.png", dpi=140); plt.close(fig)

    fig, ax = plt.subplots(1, 1, figsize=(13, 4))
    h_arr = h.tolist() + [h[-1]] if t.size == h.size + 1 else h
    ax.plot(t, h_arr, color="#3498db", lw=1.4, label="Water height h(t)")
    ax.plot(t[:ref.shape[1]], ref[IDX_Z], color="#e67e22", lw=1.2, ls="--",
            label="z_base reference")
    ax.plot(t, mpc[IDX_Z], color="#27ae60", lw=1.6, label="MPC z_base actual")
    ax.plot(t, unc[IDX_Z], color="#c0392b", lw=0.8, alpha=0.6, label="Open-loop z_base")
    ax.set_ylabel("elevation [m]")
    ax.set_xlabel("time [s]")
    ax.set_title(f"BASE-ONLY FAHFS elevation - {sc}")
    ax.grid(True, alpha=0.4); ax.legend(loc="best", fontsize=9)
    fig.tight_layout(); fig.savefig(RES / f"baseonly_{sc}_elevation.png", dpi=140); plt.close(fig)

print("Base-only plots saved.")
