# MPC Multi-Hazard Adaptive Control

Implementasi simulasi sistem kontrol adaptif multi-hazard structural-hydrodynamics berbasis Model Predictive Control. Companion code untuk proposal **Bilfaqih dkk. (DRPM-ITS, 2026)** "Pemodelan dan Simulasi Sistem Kontrol Adaptif Multi-Hazard Structural-Hydrodynamics Berbasis Model Predictive Control".

## Plant

| Spec | Value |
|---|---|
| Stories | 3 |
| Mass per floor | 333,3 kg (1 ton total) |
| Story stiffness | 1,5 × 10⁵ N/m |
| Damping | Rayleigh, ζ = 0.02 |
| Sample time | 1 ms (1 kHz) for MATLAB; 5 ms for Python mirror |
| MLFS actuator | ±5 kN per floor (per_floor) atau ±15 kN base only |
| FAHFS actuator | ±100 kN, stroke 0–5 m |

State (8): `x = [q1 q2 q3 q1' q2' q3' z_base z_base']`
Input: `u = [u_MLFS_1..3, u_FAHFS]` (per_floor) atau `u = [u_MLFS_base, u_FAHFS]` (base_only)
Disturbance (4): `d = [a_g; F_f1; F_f2; F_f3]`

## Stack

```
mpc_multihazard_matlab/         <-- main MATLAB stack
├── main.m                       Entry point: tune → benchmark 4 skenario
├── matlab_mirror.py             Python mirror untuk validasi (MATLAB-free)
├── generate_plots.py            Plot per-floor variant
├── generate_plots_baseonly.py   Plot base-only variant
├── generate_input_plots.py      Disturbance profile plots
├── src/
│   ├── build_model.m            3-DOF struktur + 1-DOF FAHFS state-space
│   ├── build_mpc.m              Dense-form QP MPC matrices
│   ├── build_reference.m        z_ref dengan lookahead 3 s
│   ├── compute_fluid_force.m    F_fluid per lantai vs z_base aktif
│   ├── compute_metrics.m        Peak, drift, energy, Jain fairness, z tracking
│   ├── get_disturbances.m       Skenario gempa, banjir, tsunami, combined
│   ├── plot_results.m           Visualisasi
│   ├── print_metrics.m          Console KPI
│   ├── run_lqr.m, run_mpc.m, run_uncontrolled.m
│   ├── solve_box_qp.m           FISTA fallback (no Optim Toolbox needed)
│   └── tune_weights.m           5-D grid search Q,R
├── simulink/                    <-- Simulink integration
│   ├── build_simulink_model.m   Auto-build .slx
│   ├── prep_simulink_workspace.m
│   ├── mpc_step.m               MPC inside MATLAB Function block
│   ├── build_disturbance_at.m
│   ├── build_reference_at.m
│   ├── get_params.m
│   └── README.md
└── results/                     <-- generated, gitignored

mpc_multihazard/                 <-- earlier Python prototype (legacy)
laporan_gen.js                   <-- docx report generator (Node.js)
```

## Quick start

### Native MATLAB

```matlab
cd mpc_multihazard_matlab
main                              % runs tune + 4 scenarios
```

### Python mirror (when MATLAB unavailable)

```bash
cd mpc_multihazard_matlab
python matlab_mirror.py per_floor    # 3 MLFS aktuator
python matlab_mirror.py base_only    # 1 MLFS aktuator
python generate_plots.py
python generate_plots_baseonly.py
python generate_input_plots.py
```

### Simulink

```matlab
cd mpc_multihazard_matlab/simulink
build_simulink_model
prep_simulink_workspace('per_floor', 'seismic')
sim('mpc_multihazard');
plot(x_log(:, 1:3))
```

### Generate report

```bash
npm install docx
node laporan_gen.js
```

Output: `Laporan_MPC_Multi_Hazard.docx` di root project.

## Hasil utama (per_floor variant)

| Skenario | Open-loop | MPC + FAHFS | Reduksi |
|---|---|---|---|
| Gempa | 96,9 mm | 0,47 mm | **99,29 %** |
| Banjir | 611,2 mm | 0,00 mm | **100,00 %** |
| Tsunami | 8767,4 mm | 690,4 mm | **91,33 %** |
| Combined | 8766,7 mm | 690,6 mm | **91,33 %** |

Bobot tuning optimal: `q_disp=100, q_vel=1000, q_z=1e5, r_MLFS=1e-6, r_FAHFS=1e-8`.

## Kebaruan

1. Integrasi MLFS (force horizontal) + FAHFS (lift vertical) sebagai dual-subsystem dengan mekanisme fisika berbeda
2. Multi-hazard adaptive control via single MPC tunggal
3. MPC untuk struktur multi-physics (struktur + hidrolik)
4. Validated via MATLAB stack + Python mirror

Detail di [`Laporan_MPC_Multi_Hazard.docx`](Laporan_MPC_Multi_Hazard.docx).

## Lisensi

Riset internal DRPM-ITS 2026 — untuk publikasi jurnal Q3.
