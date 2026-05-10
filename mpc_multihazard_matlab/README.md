# MPC Multi-Hazard Structural-Hydrodynamic Adaptive Control (MATLAB)

Simulation companion to the proposal *"Pemodelan dan Simulasi Sistem Kontrol
Adaptif Multi-Hazard Structural-Hydrodynamics Berbasis Model Predictive
Control"* (Bilfaqih et al., DRPM-ITS 2026).

## Plant

| Spec | Value |
|---|---|
| Stories | 3 |
| Mass per floor | 333.3 kg (1 ton total) |
| Story stiffness | 1.5 × 10⁵ N/m |
| Damping | Rayleigh, ζ = 0.02 |
| Sample time | 1 ms (1 kHz) |
| Actuators | 1 hybrid MLFS+FAHFS per floor, ±5 kN |

State: `x = [q1 q2 q3 q1' q2' q3']ᵀ` (6 states).
Input: `u = [u1 u2 u3]ᵀ` (3 forces).
Disturbance: `d = [a_g; F_f1; F_f2; F_f3]ᵀ`.

Continuous form (proposal §3.2):

```
M q'' + C q' + K q = F_eq + F_ctrl + F_fluid
```

Hydrodynamic load distributed over submerged facade per floor:

```
F_drag_i  = 0.5 ρ Cd (w d_i)   v² sign(v)
F_hydro_i = 0.5 ρ g  d_i² w
```

## Controller

Dense-form QP MPC (proposal §3.3):

```
min  Σ (xᵀ Q x + uᵀ R u)
s.t. x_{k+1} = A x_k + B u_k + E d_k
     u_min ≤ u_k ≤ u_max
```

Solved each step with `quadprog` (interior-point-convex), warm-started with
the previous solution shifted by one. Disturbance preview window = horizon
length (treats the next `N` samples of `d` as known — sensor estimate).

## Iterative weight tuning

`tune_weights.m` runs a coarse 3-D log grid over (`q_disp`, `q_vel`, `r`)
plus a refined pass around the current best. Score:

```
S = 0.50 · reduction_avg
  + 0.30 · Jain_fairness
  − 0.15 · normalized_energy
  − 0.05 · saturation_fraction
```

`Jain_fairness = (Σ d_i)² / (n Σ d_i²)` over peak inter-story drifts —
penalises configurations that protect one floor at the expense of others
(the *fairness* requirement).

## How to run

Open MATLAB in this folder and:

```matlab
main
```

Requires base MATLAB + Optimization Toolbox (`quadprog`). Control System
Toolbox (`dlqr`) is used only for the LQR baseline; if missing, comment
out the LQR block in `main.m`.

Octave: works after replacing `quadprog` opts with the Octave equivalent
and installing the `optim` package (`pkg load optim`).

## Outputs (saved under `results/`)

| File | Content |
|---|---|
| `seismic.png`, `flood.png`, `tsunami.png`, `combined.png` | Time-history plots: floor displacements, inter-story drifts, actuator forces, cumulative energy. |
| `tuning_log.mat` | Full sweep log (`q_disp`, `q_vel`, `r`, reduction, fairness, energy, saturation, score). |
| `results.mat` | All scenario runs + chosen `Q*`, `R*`, `params`, `sys`. |

Console prints per scenario: peak displacement, peak inter-story drift,
peak acceleration, settling time, peak actuator force per floor, control
energy, Jain fairness, and avg/per-floor peak-displacement reduction
versus the uncontrolled baseline.

## Files

```
main.m                     Orchestration: tune → benchmark → save.
src/build_model.m          M, C, K, state-space (continuous + ZOH discrete).
src/get_disturbances.m     Seismic chirp + flood/tsunami profiles.
src/build_mpc.m            Pre-builds dense-form QP matrices.
src/run_mpc.m              Receding-horizon closed-loop sim.
src/run_lqr.m              Discrete LQR baseline (saturation-clipped).
src/run_uncontrolled.m     Open-loop response.
src/compute_metrics.m      Proposal §3.8 KPIs + Jain fairness.
src/tune_weights.m         Coarse → refined Q,R search.
src/print_metrics.m        Console summary.
src/plot_results.m         Comparative time-history plots.
```
