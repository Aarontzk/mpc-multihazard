"""
Metrik evaluasi kinerja MPC multi-hazard.
Sesuai Proposal Bab 3.8 (Parameter Evaluasi Kinerja) dan Bab 3.9 (Validasi Model).

State convention (2-DOF):
    x[:, 0]  = x1      displacement lantai 1 (m)
    x[:, 1]  = x2      displacement lantai 2 / ROOF (m)
    x[:, 2]  = x1_dot  kecepatan lantai 1 (m/s)
    x[:, 3]  = x2_dot  kecepatan lantai 2 (m/s)

Input convention:
    u[:, 0]  = f1      gaya aktuator MLFS  lantai 1 (N)
    u[:, 1]  = f2      gaya aktuator FAHFS lantai 2 (N)
"""

import numpy as np


# ──────────────────────────────────────────────────────────────────────────────
# 1. Roof Displacement
# ──────────────────────────────────────────────────────────────────────────────

def roof_displacement(x_hist, roof_idx=1):
    """
    Displacement atap (lantai teratas).

    Args:
        x_hist   : trajectory state (T+1 x n)
        roof_idx : indeks kolom state untuk DOF atap (default 1 = lantai 2)

    Returns dict:
        peak_m   : nilai absolut maksimum (m)
        rms_m    : root-mean-square (m)
        ise      : integral squared error = sum(x^2) — kalikan Ts untuk satuan m^2·s
        series   : time-series displacement (T+1,)
    """
    x_roof = x_hist[:, roof_idx]
    return {
        "peak_m" : float(np.max(np.abs(x_roof))),
        "rms_m"  : float(np.sqrt(np.mean(x_roof ** 2))),
        "ise"    : float(np.sum(x_roof ** 2)),
        "series" : x_roof,
    }


# ──────────────────────────────────────────────────────────────────────────────
# 2. Air Gap  (inter-story drift = relative displacement antar lantai)
# ──────────────────────────────────────────────────────────────────────────────

def air_gap(x_hist, lower_idx=0, upper_idx=1):
    """
    Celah / inter-story drift antara dua DOF.
    Untuk hardware: peta ke clearance aktual pada mekanisme aktuator hibrid.

    Args:
        x_hist    : trajectory state (T+1 x n)
        lower_idx : indeks DOF lantai bawah
        upper_idx : indeks DOF lantai atas

    Returns dict:
        peak_m    : drift absolut maksimum (m)
        rms_m     : RMS drift (m)
        series    : time-series drift (T+1,)
    """
    drift = x_hist[:, upper_idx] - x_hist[:, lower_idx]
    return {
        "peak_m" : float(np.max(np.abs(drift))),
        "rms_m"  : float(np.sqrt(np.mean(drift ** 2))),
        "series" : drift,
    }


# ──────────────────────────────────────────────────────────────────────────────
# 3. Hybrid Actuator Force
# ──────────────────────────────────────────────────────────────────────────────

def hybrid_actuator_force(u_hist, Ts=1.0):
    """
    Analisis gaya aktuator hibrid (MLFS + FAHFS).

    Args:
        u_hist : trajectory kontrol (T x m)
        Ts     : sampling time (s) — untuk hitung energi

    Returns dict:
        peak_N         : gaya absolut maks per aktuator (m,)
        rms_N          : RMS gaya per aktuator (m,)
        resultant_peak : puncak norma Euclidean ||u|| (N)
        energy_J       : energi kontrol = sum(||u_k||^2) * Ts  [Proposal 3.8.3: sum(u^2)]
        series         : u_hist referensi langsung
    """
    peak_N         = np.max(np.abs(u_hist), axis=0)
    rms_N          = np.sqrt(np.mean(u_hist ** 2, axis=0))
    resultant      = np.linalg.norm(u_hist, axis=1)
    resultant_peak = float(np.max(resultant))
    energy_J       = float(np.sum(resultant ** 2) * Ts)

    return {
        "peak_N"         : peak_N,
        "rms_N"          : rms_N,
        "resultant_peak" : resultant_peak,
        "energy_J"       : energy_J,
        "series"         : u_hist,
    }


# ──────────────────────────────────────────────────────────────────────────────
# 4. Peak Displacement Reduction [Proposal 3.8.1]
#    Persentase reduksi vs sistem tanpa kontrol (uncontrolled baseline)
# ──────────────────────────────────────────────────────────────────────────────

def peak_displacement_reduction(x_ctrl, x_unctrl, roof_idx=1):
    """
    Persentase reduksi peak displacement MPC vs uncontrolled.  [Proposal 3.8.1]
    Juga hitung vibration reduction ratio (RMS) untuk kinerja MLFS. [Proposal 3.4]

    Args:
        x_ctrl   : trajectory state dengan MPC (T+1 x n)
        x_unctrl : trajectory state tanpa kontrol (T+1 x n)
        roof_idx : indeks DOF atap

    Returns dict:
        peak_reduction_pct  : reduksi peak (%)
        rms_reduction_pct   : reduksi RMS (%)
        vrr                 : vibration reduction ratio = RMS_ctrl / RMS_unctrl
                              (< 1 berarti MPC lebih baik)
        peak_ctrl_m         : peak displacement dengan MPC (m)
        peak_unctrl_m       : peak displacement tanpa kontrol (m)
    """
    x_c = x_ctrl[:, roof_idx]
    x_u = x_unctrl[:, roof_idx]

    peak_c = float(np.max(np.abs(x_c)))
    peak_u = float(np.max(np.abs(x_u)))
    rms_c  = float(np.sqrt(np.mean(x_c ** 2)))
    rms_u  = float(np.sqrt(np.mean(x_u ** 2)))

    peak_red = (peak_u - peak_c) / peak_u * 100.0 if peak_u > 0 else 0.0
    rms_red  = (rms_u  - rms_c)  / rms_u  * 100.0 if rms_u  > 0 else 0.0
    vrr      = rms_c / rms_u if rms_u > 0 else float("nan")

    return {
        "peak_reduction_pct" : float(peak_red),
        "rms_reduction_pct"  : float(rms_red),
        "vrr"                : float(vrr),
        "peak_ctrl_m"        : peak_c,
        "peak_unctrl_m"      : peak_u,
    }


# ──────────────────────────────────────────────────────────────────────────────
# 5. Settling Time [Proposal 3.8.2]
#    Waktu respons turun & tetap di bawah threshold_pct% dari nilai puncak
# ──────────────────────────────────────────────────────────────────────────────

def settling_time(x_hist, Ts, roof_idx=1, threshold_pct=5.0, t_disturbance_end=None):
    """
    Settling time: waktu pertama kali respons masuk dan tetap di bawah
    threshold_pct% dari nilai puncak.  [Proposal 3.8.2]

    Args:
        x_hist            : trajectory state (T+1 x n)
        Ts                : sampling time (s)
        roof_idx          : indeks DOF atap
        threshold_pct     : batas persentase dari puncak (default 5%)
        t_disturbance_end : waktu (s) gangguan berakhir — settling diukur dari sini.
                            Jika None, diukur dari seluruh simulasi.

    Returns dict:
        settling_time_s   : settling time (s), atau None jika tidak tercapai
        threshold_m       : nilai threshold absolut (m)
        peak_m            : nilai puncak (m)
    """
    x_roof = np.abs(x_hist[:, roof_idx])
    peak   = float(np.max(x_roof))
    thr    = peak * threshold_pct / 100.0

    # Tentukan indeks awal pengukuran
    if t_disturbance_end is not None:
        k_start = int(t_disturbance_end / Ts)
    else:
        k_start = 0

    x_after = x_roof[k_start:]
    st_s    = None

    for k in range(len(x_after)):
        if np.all(x_after[k:] <= thr):   # tetap di bawah threshold sampai akhir
            st_s = float((k_start + k) * Ts)
            break

    return {
        "settling_time_s" : st_s,
        "threshold_m"     : float(thr),
        "peak_m"          : peak,
    }


# ──────────────────────────────────────────────────────────────────────────────
# 6. Elevation Adaptation Error — FAHFS [Proposal 3.4, 3.8]
#    Seberapa akurat FAHFS menjaga elevasi struktur dari muka air banjir
# ──────────────────────────────────────────────────────────────────────────────

def elevation_adaptation_error(x_hist, flood_level_seq, fahfs_dof_idx=0, Ts=1.0):
    """
    Error adaptasi elevasi FAHFS vs tinggi muka air banjir.  [Proposal 3.4]
    FAHFS harus menjaga x_fahfs >= flood_level (elevasi aman).

    Args:
        x_hist          : trajectory state (T+1 x n)
        flood_level_seq : target elevasi aman (tinggi muka air) per langkah (T+1,)
        fahfs_dof_idx   : indeks DOF yang dikendalikan FAHFS (default 0 = lantai 1)
        Ts              : sampling time (s)

    Returns dict:
        peak_error_m    : error elevasi puncak (m)
        rms_error_m     : RMS error elevasi (m)
        breach_count    : jumlah langkah di mana x < flood_level (struktur terendam)
        breach_duration : durasi total perendaman (s)
        series_error    : time-series error (T+1,)
    """
    T1            = min(len(x_hist), len(flood_level_seq))
    x_fahfs       = x_hist[:T1, fahfs_dof_idx]
    flood_level   = np.asarray(flood_level_seq[:T1])

    error         = flood_level - x_fahfs          # positif = struktur terendam
    breach_mask   = error > 0

    return {
        "peak_error_m"    : float(np.max(np.abs(error))),
        "rms_error_m"     : float(np.sqrt(np.mean(error ** 2))),
        "breach_count"    : int(np.sum(breach_mask)),
        "breach_duration" : float(np.sum(breach_mask) * Ts),
        "series_error"    : error,
    }


# ──────────────────────────────────────────────────────────────────────────────
# 7. Robustness — Sensitivity terhadap variasi gangguan [Proposal 3.8.4, 3.9]
# ──────────────────────────────────────────────────────────────────────────────

def robustness_sensitivity(peak_results: dict):
    """
    Ukur sensitivitas kinerja terhadap variasi magnitude gangguan.  [Proposal 3.9]

    Args:
        peak_results : dict {magnitude_label: peak_displacement_m}
                       Contoh: {"0.5x": 0.002, "1.0x": 0.004, "2.0x": 0.009}

    Returns dict:
        sensitivity_ratio : (peak_max - peak_min) / peak_nominal
                            rendah = robust
        peak_results      : data input (pass-through)
    """
    values    = list(peak_results.values())
    keys      = list(peak_results.keys())
    peak_max  = float(np.max(values))
    peak_min  = float(np.min(values))
    nominal   = float(values[len(values) // 2])   # nilai tengah sebagai nominal

    sens = (peak_max - peak_min) / nominal if nominal > 0 else float("nan")

    return {
        "sensitivity_ratio" : sens,
        "peak_max_m"        : peak_max,
        "peak_min_m"        : peak_min,
        "peak_nominal_m"    : nominal,
        "peak_results"      : peak_results,
    }


# ──────────────────────────────────────────────────────────────────────────────
# 8. Perbandingan MPC vs Baseline [Proposal 3.9]
# ──────────────────────────────────────────────────────────────────────────────

def compare_controllers(results: dict, Ts=1.0, roof_idx=1):
    """
    Bandingkan beberapa kontroler (MPC, LQR, PID, Uncontrolled).  [Proposal 3.9]

    Args:
        results : dict {label: x_hist}
                  Contoh: {"MPC": x_mpc, "LQR": x_lqr, "Uncontrolled": x_unc}
        Ts      : sampling time (s)
        roof_idx: indeks DOF atap

    Returns:
        DataFrame-like dict {label: {peak_m, rms_m, ise}}
    """
    summary = {}
    for label, x_hist in results.items():
        x_roof = x_hist[:, roof_idx]
        summary[label] = {
            "peak_m" : float(np.max(np.abs(x_roof))),
            "rms_m"  : float(np.sqrt(np.mean(x_roof ** 2))),
            "ise"    : float(np.sum(x_roof ** 2) * Ts),
        }

    # Print tabel perbandingan
    print("=" * 60)
    print(f"{'Controller':<18} {'Peak (mm)':>12} {'RMS (mm)':>12} {'ISE (m^2s)':>12}")
    print("-" * 60)
    for label, m in summary.items():
        print(f"{label:<18} {m['peak_m']*1e3:>12.3f} {m['rms_m']*1e3:>12.3f} {m['ise']:>12.4f}")
    print("=" * 60)

    return summary


# ──────────────────────────────────────────────────────────────────────────────
# Ringkasan semua metrik sekaligus
# ──────────────────────────────────────────────────────────────────────────────

def compute_all(x_hist, u_hist, Ts=1.0, roof_idx=1, lower_idx=0, upper_idx=1,
                x_uncontrolled=None, t_disturbance_end=None,
                flood_level_seq=None, fahfs_dof_idx=0):
    """
    Hitung semua metrik dan cetak ringkasan.  [Proposal Bab 3.8]

    Args:
        x_hist, u_hist      : trajectory MPC
        Ts                  : sampling time (s)
        roof_idx            : indeks DOF atap
        lower_idx/upper_idx : indeks DOF untuk air gap
        x_uncontrolled      : trajectory tanpa kontrol (opsional, untuk PDR)
        t_disturbance_end   : waktu gangguan berakhir (s) untuk settling time
        flood_level_seq     : target elevasi banjir (T+1,) untuk FAHFS error
        fahfs_dof_idx       : DOF FAHFS untuk elevation error

    Returns:
        dict dengan semua metrik
    """
    roof     = roof_displacement(x_hist, roof_idx)
    gap      = air_gap(x_hist, lower_idx, upper_idx)
    actuator = hybrid_actuator_force(u_hist, Ts)
    st       = settling_time(x_hist, Ts, roof_idx,
                             threshold_pct=5.0, t_disturbance_end=t_disturbance_end)

    print("=" * 55)
    print("  EVALUASI KINERJA MPC  [Proposal Bab 3.8]")
    print("=" * 55)

    # 3.8.1 Peak displacement reduction
    print(f"[3.8.1] PEAK DISPLACEMENT")
    print(f"  Peak  : {roof['peak_m']*1e3:8.3f} mm")
    print(f"  RMS   : {roof['rms_m']*1e3:8.3f} mm")
    print(f"  ISE   : {roof['ise']:8.4f} m^2")

    if x_uncontrolled is not None:
        pdr = peak_displacement_reduction(x_hist, x_uncontrolled, roof_idx)
        print(f"  Peak reduction vs uncontrolled : {pdr['peak_reduction_pct']:6.1f} %")
        print(f"  RMS  reduction vs uncontrolled : {pdr['rms_reduction_pct']:6.1f} %")
        print(f"  Vibration Reduction Ratio      : {pdr['vrr']:6.3f}  (MLFS)")
    else:
        pdr = None
        print(f"  (Sediakan x_uncontrolled untuk hitung PDR vs baseline)")

    # 3.8.2 Settling time
    print(f"\n[3.8.2] SETTLING TIME")
    if st["settling_time_s"] is not None:
        print(f"  Settling time (5%) : {st['settling_time_s']:7.2f} s")
    else:
        print(f"  Settling time      : tidak tercapai dalam simulasi")
    print(f"  Threshold          : {st['threshold_m']*1e3:.3f} mm")

    # 3.8.3 Control energy
    print(f"\n[3.8.3] CONTROL ENERGY  sum(u^2)")
    for i, (pk, rms) in enumerate(zip(actuator['peak_N'], actuator['rms_N'])):
        label = ["MLFS (u1)", "FAHFS (u2)"][i] if i < 2 else f"u{i+1}"
        print(f"  {label}  peak: {pk:8.1f} N   rms: {rms:8.1f} N")
    print(f"  ||u|| peak         : {actuator['resultant_peak']:8.1f} N")
    print(f"  Energy sum(||u||^2)*Ts : {actuator['energy_J']:8.1f} N^2*s")

    # Air gap
    print(f"\n[+] AIR GAP (inter-story drift)")
    print(f"  Peak  : {gap['peak_m']*1e3:8.3f} mm")
    print(f"  RMS   : {gap['rms_m']*1e3:8.3f} mm")

    # FAHFS elevation adaptation
    if flood_level_seq is not None:
        elev = elevation_adaptation_error(x_hist, flood_level_seq, fahfs_dof_idx, Ts)
        print(f"\n[3.4] FAHFS ELEVATION ADAPTATION ERROR")
        print(f"  Peak error     : {elev['peak_error_m']*1e3:8.3f} mm")
        print(f"  RMS error      : {elev['rms_error_m']*1e3:8.3f} mm")
        print(f"  Breach count   : {elev['breach_count']:8d} steps")
        print(f"  Breach duration: {elev['breach_duration']:8.2f} s")
    else:
        elev = None

    # 3.8.4 Robustness note
    print(f"\n[3.8.4] ROBUSTNESS")
    print(f"  Gunakan robustness_sensitivity(peak_results) untuk sensitivity analysis.")
    print(f"  Gunakan compare_controllers(results) untuk perbandingan MPC vs LQR/PID.")
    print("=" * 55)

    return {
        "roof"     : roof,
        "air_gap"  : gap,
        "actuator" : actuator,
        "settling" : st,
        "pdr"      : pdr,
        "elevation": elev,
    }
