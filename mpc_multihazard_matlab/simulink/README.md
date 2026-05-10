# Simulink Integration

Cara pakai Simulink untuk simulasi MPC multi-hazard.

## Quick start

```matlab
% 1. Pindah ke folder simulink
cd mpc_multihazard_matlab/simulink

% 2. Build model otomatis (sekali saja, atau ulang setelah edit)
build_simulink_model            % atau build_simulink_model('base_only')

% 3. Populate workspace dengan params + disturbance untuk skenario tertentu
prep_simulink_workspace('per_floor', 'seismic')   % atau 'flood', 'tsunami', 'combined'

% 4. Run simulasi
sim('mpc_multihazard');

% 5. Hasil tersimpan di workspace sebagai x_log, u_log
plot(x_log(:, 1:3))    % displacement 3 lantai
```

## Apa yang di-build

`build_simulink_model.m` membuat file `mpc_multihazard.slx` dengan blok-blok:

| Blok | Tipe | Fungsi |
|---|---|---|
| Clock | Source | Waktu simulasi |
| DisturbanceGen | MATLAB Function | Sample a_g, h, v dari workspace |
| ReferenceGen | MATLAB Function | Sample reference z_ref vs t |
| MPC_Controller | MATLAB Function | Receding horizon MPC, panggil `mpc_step` + FISTA QP |
| FluidForce | MATLAB Function | F_fluid per lantai vs z_base aktual |
| Plant | Discrete State-Space | x(k+1) = Ad·x + Bd·[u; d] |
| MuxIn / MuxDist | Mux | Gabung input/disturbance vektor |
| DemuxX | Demux | Pisah state untuk scope |
| Scope_q, Scope_u, Scope_z | Scope | Visualisasi displacement, gaya, elevasi |
| Log_x, Log_u | To Workspace | Save x_log, u_log |

## Wiring diagram

```
Clock ──┬──► DisturbanceGen ──► [ag, h, v] ──┬──► MPC_Controller ──► u
        │                                     │           ▲
        ├──► ReferenceGen ──► r ──────────────┘           │
        │                                                  │
        │   [ag] ───────────────────────────► MuxDist ─┐  │
        │   [h, v] ──► FluidForce ──► [F_f] ──────────┘  │
        │                                ▲                │
        │                                │                │
        │                                │   [u; d]       │
        │                                │      ▼         │
        │                            Plant (Discrete SS) ─┴──► state x
        │                                      │
        │                                      ▼
        │                              DemuxX, Scopes, To Workspace
```

## Parameter dari workspace

Yang di-baca Simulink dari workspace (set otomatis oleh `prep_simulink_workspace`):

- `Ad`, `Bd`, `Ed` — discrete plant matrices
- `Q`, `R` — MPC weights
- `Ts`, `T_sim` — sampling + total durasi
- `NU`, `ND`, `NX` — dimensi
- `u_lim` — actuator bounds
- `dist_data` — struct {t, ag, h, v}
- `ref_data` — reference matrix (8 x N)
- `mpc_data` — pre-built MPC matrices (Phi, Gamma, Gd, Qbar, H, ...)
- `params` — full params struct

## Variasi skenario

```matlab
% Per-floor (3 MLFS), gempa
prep_simulink_workspace('per_floor', 'seismic');  sim('mpc_multihazard');

% Per-floor, tsunami
prep_simulink_workspace('per_floor', 'tsunami');  sim('mpc_multihazard');

% Base-only (1 MLFS), combined
prep_simulink_workspace('base_only', 'combined'); sim('mpc_multihazard');
```

## Solver setting

Model di-set otomatis ke:
- Solver: `FixedStepDiscrete`
- Step size: `Ts` (variable workspace, default 1e-3 s)
- Stop time: `T_sim`

Bisa diubah manual via Modeling > Model Settings > Solver kalau perlu.

## Troubleshooting

**"Undefined function 'mpc_step'"** — pastikan `addpath` ke folder `simulink/` dan `src/`. Atau jalankan `build_simulink_model` (otomatis tambah path).

**"Variable Ad not defined"** — jalankan `prep_simulink_workspace` dulu sebelum `sim`.

**Solver lambat** — turunkan `T_sim` atau naikkan `Ts` ke 5e-3 saat eksperimen awal.

**FISTA tidak konvergen** — cek `mpc_data.L` (Lipschitz constant). Bila terlalu kecil, naikkan margin di `solve_box_qp.m`.

## Catatan

- `mpc_step.m` pakai persistent variable untuk warm-start QP. Reset dengan `clear mpc_step` antar run.
- Simulink call `evalin('base', ...)` untuk akses workspace var. Pastikan workspace bersih sebelum `prep_simulink_workspace`.
- Performance: model ini run di MATLAB interpreted MATLAB Function blocks — lebih lambat dari pure script. Untuk speed, ubah ke S-Function atau Code Generation (Embedded Coder).
