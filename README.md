# MPC Multi-Hazard Adaptive Control

Simulasi sistem kontrol adaptif bangunan tahan **gempa + banjir + tsunami** secara simultan, berbasis Model Predictive Control (MPC). Companion code untuk proposal **Bilfaqih dkk., DRPM-ITS 2026**.

---

## TL;DR

Bangunan 3 lantai 1 ton dilengkapi 2 sistem aktuator yang dikoordinasi 1 MPC:

| Sistem | Fungsi | Mekanisme |
|---|---|---|
| **MLFS** (Magnetic Levitation Foundation System) | Redam getaran gempa | Aktuator elektromagnetik gaya horizontal |
| **FAHFS** (Flood-Adaptive Hydraulic Foundation System) | Hindari banjir/tsunami | Silinder hidrolik **angkat seluruh struktur** ke atas air |

**Hasil reduksi displacement vs open-loop:**

| Skenario | Per-floor (3 MLFS) | Base-only (1 MLFS) |
|---|---|---|
| Gempa | **99,29 %** | 74,54 % |
| Banjir | **100,00 %** | 100,00 % |
| Tsunami | **91,33 %** | 91,19 % |
| Combined (gempa + tsunami) | **91,33 %** | 91,19 % |

---

## Konsep

```
                     ┌─────────────────────┐
                     │   MPC Controller    │
                     │  (1 brain, 4 input) │
                     └──────────┬──────────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        ▼                       ▼                       ▼
  ┌───────────┐          ┌───────────┐          ┌───────────┐
  │  MLFS  3  │          │  MLFS  2  │          │  MLFS  1  │  ← 3 aktuator (per_floor)
  └─────┬─────┘          └─────┬─────┘          └─────┬─────┘     atau 1 (base_only)
        │                      │                      │
   ┌────┴──────────────────────┴──────────────────────┴────┐
   │                       LANTAI 3                         │  ← 333,3 kg
   │         k = 1,5e5 N/m  ζ = 2 % Rayleigh                │
   ├────────────────────────────────────────────────────────┤
   │                       LANTAI 2                         │
   ├────────────────────────────────────────────────────────┤
   │                       LANTAI 1                         │
   └─────────────────────────┬──────────────────────────────┘
                             │
                  ┌──────────▼──────────┐
                  │       FAHFS         │  ← silinder hidrolik
                  │   (lift z_base)     │     stroke 0..5 m
                  └──────────┬──────────┘
                             │
~~~~~~~ banjir h(t) ~~~~~~~~~│~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                             │
        ▼ ground accel a_g(t) (gempa)
========= TANAH ========================================
```

**Strategi MPC:**
- Gempa → MLFS aktif redam getaran
- Banjir/tsunami → FAHFS angkat struktur sebelum air datang (lookahead 3 detik)
- Combined → MPC otomatis koordinasi keduanya

---

## Quick Start

### Pilihan A — Python mirror (paling cepat, tidak butuh MATLAB)

```bash
pip install numpy scipy matplotlib
cd mpc_multihazard_matlab

# Run simulasi (10–15 menit per varian)
python matlab_mirror.py per_floor    # 3 aktuator MLFS
python matlab_mirror.py base_only    # 1 aktuator MLFS

# Generate plots
python generate_plots.py
python generate_plots_baseonly.py
python generate_input_plots.py

# Generate laporan .docx
cd ..
npm install docx
node laporan_gen.js
```

Hasil: `Laporan_MPC_Multi_Hazard.docx` di root project.

### Pilihan B — MATLAB native

```matlab
cd mpc_multihazard_matlab
main                                  % runs tune + 4 scenarios
```

Butuh MATLAB R2020a+. Optimization Toolbox (`quadprog`) opsional — fallback FISTA otomatis.

### Pilihan C — Simulink GUI

```matlab
cd mpc_multihazard_matlab/simulink
build_simulink_model
prep_simulink_workspace('per_floor', 'seismic')
sim('mpc_multihazard');
plot(x_log(:, 1:3))    % displacement 3 lantai
```

Detail: [`mpc_multihazard_matlab/simulink/README.md`](mpc_multihazard_matlab/simulink/README.md).

---

## Glosarium Singkat

| Istilah | Penjelasan |
|---|---|
| **MPC** | Model Predictive Control — optimisasi kontrol berdasarkan prediksi N langkah ke depan, ulangi tiap sampling |
| **MLFS** | Magnetic Levitation Foundation System — aktuator gaya elektromagnetik |
| **FAHFS** | Flood-Adaptive Hydraulic Foundation System — silinder hidrolik untuk angkat pondasi |
| **Open-loop** | Kondisi tanpa kontrol (baseline pembanding) |
| **LQR** | Linear Quadratic Regulator — kontroler klasik (pembanding) |
| **State** | Variabel sistem: posisi, kecepatan, elevasi |
| **Disturbance** | Gangguan eksternal: gempa, banjir, tsunami |
| **Reference (z_ref)** | Target elevasi yang harus dilacak FAHFS |
| **Horizon** | Berapa langkah ke depan MPC memprediksi (di sini: 20 langkah) |
| **Jain Fairness** | Indeks 0..1, ukur seberapa merata proteksi antar lantai |
| **FISTA** | Fast Iterative Shrinkage-Thresholding — algoritma QP yang dipakai bila `quadprog` absent |
| **ZOH** | Zero-Order Hold — metode diskritisasi continuous → discrete time |

---

## Struktur Direktori

```
MPC RL Disaster/
├── README.md                       ← file ini
├── CLAUDE.md                       Instruksi internal AI assistant
├── AGENTS.md                       Catatan agentic workflow
├── package.json                    Node deps (untuk laporan_gen.js)
├── laporan_gen.js                  Generator laporan .docx
│
├── mpc_multihazard_matlab/         ★ STACK UTAMA (MATLAB + Python mirror)
│   ├── main.m                      Entry MATLAB: tune + benchmark 4 skenario
│   ├── matlab_mirror.py            Mirror Python 1:1 (untuk validasi tanpa MATLAB)
│   ├── generate_plots.py           Plot per-floor variant
│   ├── generate_plots_baseonly.py  Plot base-only variant
│   ├── generate_input_plots.py     Plot profil disturbance
│   ├── README.md                   Detail MATLAB stack
│   ├── src/                        12 file .m: model, MPC, metric, tuning
│   ├── simulink/                   ★ Integrasi Simulink (GUI workflow)
│   │   ├── build_simulink_model.m  Auto-build .slx
│   │   ├── prep_simulink_workspace.m
│   │   ├── mpc_step.m              MPC inside MATLAB Function block
│   │   └── README.md
│   └── results/                    [gitignored] hasil run
│
├── mpc_multihazard/                ★ Prototype Python awal (legacy, untuk referensi)
│   ├── main.py
│   └── src/
│
└── Laporan_MPC_Multi_Hazard.docx   [gitignored, regenerable] laporan akademik final
```

---

## Plant Spesifikasi

| Komponen | Nilai | Satuan |
|---|---|---|
| Jumlah lantai | 3 | DOF struktural |
| Massa per lantai | 333,3 | kg (total 1 ton) |
| Stiffness per lantai | 1,5 × 10⁵ | N/m |
| Damping ratio (Rayleigh) | 0,02 | — |
| Frekuensi natural | 1,503 / 4,210 / 6,084 | Hz |
| Tinggi tiap lantai | 3,0 | m |
| Sample time | 1 (MATLAB) / 5 (mirror) | ms |
| MLFS limit (per lantai) | ±5 000 | N |
| MLFS limit (base only) | ±15 000 | N |
| FAHFS limit | ±100 000 | N |
| FAHFS stroke | 0 .. 5,0 | m |
| FAHFS safety margin | 0,5 | m di atas h(t) |
| FAHFS lookahead | 3,0 | detik |

State (8): `x = [q1 q2 q3 q1' q2' q3' z_base z_base']`
Input: `[u_MLFS_1, u_MLFS_2, u_MLFS_3, u_FAHFS]` (per_floor) atau `[u_MLFS_base, u_FAHFS]` (base_only)
Disturbance: `[a_g; F_f1; F_f2; F_f3]`

---

## Skenario Uji

| Skenario | Profil | Durasi |
|---|---|---|
| **seismic** | Chirp 0,5–8 Hz, PGA 3,5 m/s², envelope Gaussian | 30 s |
| **flood** | Kenaikan air bertahap ke 2,5 m, surge 1,5 m/s | 60 s |
| **tsunami** | Soliton-like sech² h_peak = 4,5 m, v_peak = 6 m/s | 25 s |
| **combined** | Gempa + tsunami onset pada t = 12 s | 40 s |

---

## Metrik Evaluasi

Diimplementasi di [`src/compute_metrics.m`](mpc_multihazard_matlab/src/compute_metrics.m):

- **Peak displacement** per lantai (mm)
- **RMS displacement** per lantai (mm)
- **Peak inter-story drift** (mm)
- **Peak acceleration** (m/s²) — kenyamanan
- **Settling time** (s)
- **Peak control force** per aktuator (N)
- **Control energy** Σu²·Ts (N²·s)
- **Peak elevation z_base** (m) — untuk FAHFS
- **Tracking RMSE z_base** (m)
- **Jain fairness** drift antar lantai (0..1)
- **Reduksi % vs open-loop** per lantai dan rata-rata

---

## Tuning Bobot

Score function multi-objective (di [`src/tune_weights.m`](mpc_multihazard_matlab/src/tune_weights.m)):

```
S = 0,35·reduksi_avg + 0,20·fairness − 0,10·E_norm − 0,05·sat + 0,30·z_track_quality
```

Pencarian via grid logaritmik 5-D pada (`q_disp`, `q_vel`, `q_z`, `r_MLFS`, `r_FAHFS`).

**Hasil optimal (per_floor):**

```
q_disp  = 100        ← penalty perpindahan struktur
q_vel   = 1000       ← penalty kecepatan struktur
q_z     = 1e5        ← penalty error elevasi FAHFS
r_MLFS  = 1e-6       ← penalty energi MLFS
r_FAHFS = 1e-8       ← penalty energi FAHFS
score   = 0,7879
```

Insight: `q_z >> q_disp` → strategi optimal MPC memprioritaskan **bypass via FAHFS** daripada **kompensasi via MLFS**.

---

## Dependencies

| Tool | Versi | Wajib? |
|---|---|---|
| MATLAB | R2020a+ | Opsional (Python mirror tersedia) |
| MATLAB Optimization Toolbox | — | Opsional (FISTA fallback) |
| MATLAB Control System Toolbox | — | Opsional (Riccati fallback) |
| MATLAB Simulink | R2020a+ | Hanya untuk GUI workflow |
| Python | 3.9+ | Wajib (mirror + plots) |
| NumPy, SciPy, Matplotlib | — | Wajib |
| Node.js | 18+ | Wajib (laporan generator) |
| `docx` (npm package) | 8+ | Wajib (laporan generator) |

Install Python deps:

```bash
pip install numpy scipy matplotlib
```

Install Node deps:

```bash
npm install docx
```

---

## Workflow Reproduksi Hasil

```bash
# 1. Clone
git clone <repo-url> mpc-multihazard
cd mpc-multihazard

# 2. Run simulasi (Python mirror — 15 menit per varian)
cd mpc_multihazard_matlab
python matlab_mirror.py per_floor
python matlab_mirror.py base_only

# 3. Generate semua plot
python generate_plots.py
python generate_plots_baseonly.py
python generate_input_plots.py

# 4. Generate laporan
cd ..
node laporan_gen.js

# 5. Buka hasil
start Laporan_MPC_Multi_Hazard.docx
```

---

## Troubleshooting

**`ModuleNotFoundError: No module named 'numpy'`**
→ `pip install numpy scipy matplotlib`

**`Error using optimoptions ... Invalid solver specified`**
→ Optimization Toolbox tidak terinstall. Code otomatis fallback ke FISTA — abaikan, lanjut.

**`Undefined function 'dlqr'`**
→ Control System Toolbox tidak terinstall. Code otomatis fallback ke iterative Riccati.

**Simulink: `Variable Ad not defined`**
→ Lupa run `prep_simulink_workspace` sebelum `sim`.

**`Cannot find module 'docx'`**
→ Run `npm install docx` di root project.

**Tuning lambat (>30 menit)**
→ Edit `matlab_mirror.py`: turunkan `T_tune` dari 8 → 4 detik, atau kurangi grid points.

**Plot kosong / blank**
→ Check `matlab_mirror.npz` ada di `mpc_multihazard_matlab/results/`. Kalau tidak, run `matlab_mirror.py` dulu.

---

## Iterasi Pengembangan

History sederhana (lihat `git log` untuk detail):

| Tag | Apa yang berubah |
|---|---|
| `v0.1` (legacy Python) | Prototype awal, FAHFS digabung sebagai gaya horizontal — flood/tsunami cuma 2-21% reduksi |
| `v0.2` | MATLAB stack, FAHFS masih sebagai gaya horizontal |
| `v0.3` | **FAHFS proper sebagai sistem peninggian elevasi** (sesuai proposal) — flood/tsunami melonjak ke 91-100% |
| `v1.0-initial` | Per-floor + base-only variant + Simulink integration + laporan automation |

---

## Kebaruan vs Proposal

Penelitian ini berkontribusi:

1. **Integrasi MLFS + FAHFS** sebagai dual-subsystem dengan mekanisme fisika berbeda (force vs lift)
2. **Multi-hazard adaptive control** via MPC tunggal
3. **Strategi optimal terungkap**: prioritaskan bypass via FAHFS daripada kompensasi via MLFS
4. **Validasi cross-platform**: MATLAB stack + Python mirror produce identical results

Detail teknis di [`Laporan_MPC_Multi_Hazard.docx`](Laporan_MPC_Multi_Hazard.docx).

---

## Lisensi & Sitasi

Riset internal DRPM-ITS 2026, target publikasi jurnal Q3 Scopus.

Sitasi yang disarankan:

```
Bilfaqih, Y., Sahal, M., Hidayat, Z., Gamayanti, N. (2026).
"Pemodelan dan Simulasi Sistem Kontrol Adaptif Multi-Hazard
Structural-Hydrodynamics Berbasis Model Predictive Control."
Departemen Teknik Elektro, FTEIC, Institut Teknologi Sepuluh Nopember.
TM/DRPM-ITS/PN.01.028.
```

---

## Kontak

Proyek di-maintain oleh Tim Riset Sistem Kontrol Cerdas, Departemen Teknik Elektro ITS.
Issue/PR: silakan buka di repo GitHub setelah di-push.
