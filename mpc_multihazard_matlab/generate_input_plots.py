"""Render disturbance input profile plots (ground accel + water height + velocity)
for documenting the simulation inputs in the report."""
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).parent
RES = ROOT / "results"
data = np.load(RES / "matlab_mirror.npz", allow_pickle=True)

scenarios = {"seismic": "Gempa", "flood": "Banjir", "tsunami": "Tsunami", "combined": "Gabungan"}

for sc, label in scenarios.items():
    t = data[f"{sc}_t"][:-1]   # disturbance vector length T (one less than state log)
    # reload from h saved field; h length matches state log T+1, so trim
    h = data[f"{sc}_h"]
    if h.size == t.size + 1:
        h = h[:-1]
    # We didn't save ag/v separately; reconstruct minimal from h pattern alone? Skip ag/v if absent.
    # Plot only height profile; ag chirp known from params for seismic.

    fig, ax = plt.subplots(2, 1, figsize=(12, 5), sharex=True)

    # subplot 1: ground acceleration (regen for seismic + combined only)
    if sc in ("seismic", "combined"):
        PGA = 3.5; f0 = 0.5; f1 = 8.0
        T = t[-1] if t.size else 1.0
        phi = 2 * np.pi * (f0 * t + (f1 - f0) / (2 * T) * t ** 2)
        env = np.exp(-(((t - 0.35 * T) / (0.18 * T)) ** 2))
        ag = PGA * env * np.sin(phi) + 0.25 * PGA * env * np.sin(2 * np.pi * 15 * t + 1.3)
        ax[0].plot(t, ag, color="#c0392b", lw=0.8)
        ax[0].set_ylabel("a_g [m/s^2]")
        ax[0].set_title(f"Ground acceleration - {label}")
    else:
        ax[0].text(0.5, 0.5, "(tidak ada eksitasi seismik)", ha="center", va="center",
                   transform=ax[0].transAxes, style="italic")
        ax[0].set_ylabel("a_g [m/s^2]")
        ax[0].set_title(f"Ground acceleration - {label}")
    ax[0].grid(True, alpha=0.4)

    # subplot 2: water height
    ax[1].plot(t, h, color="#2980b9", lw=1.2)
    ax[1].fill_between(t, 0, h, alpha=0.25, color="#2980b9")
    ax[1].set_ylabel("h(t) [m]")
    ax[1].set_xlabel("time [s]")
    ax[1].set_title(f"Water surface height - {label}")
    ax[1].grid(True, alpha=0.4)

    fig.tight_layout()
    fig.savefig(RES / f"input_{sc}.png", dpi=140)
    plt.close(fig)

print("Input profile plots saved under:", RES)
