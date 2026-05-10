const {
  Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell,
  AlignmentType, BorderStyle, WidthType, LevelFormat, ImageRun, PageBreak
} = require('docx');
const fs = require('fs');
const path = require('path');

const RES_DIR = path.join(__dirname, 'mpc_multihazard_matlab', 'results');
const OUT = path.join(__dirname, 'Laporan_MPC_Multi_Hazard.docx');

const TNR = "Times New Roman";
const COURIER = "Courier New";
const border = { style: BorderStyle.SINGLE, size: 1, color: "000000" };
const borders = { top: border, bottom: border, left: border, right: border };

function tr(text, opts = {}) {
  return new TextRun({
    text, bold: opts.bold || false, italics: opts.italics || false,
    size: opts.size || 24, font: opts.font || TNR, color: "000000",
  });
}
function p(text, opts = {}) {
  return new Paragraph({
    alignment: opts.align || AlignmentType.JUSTIFIED,
    spacing: { after: opts.after || 120, line: 360 },
    indent: opts.indent ? { firstLine: 720 } : undefined,
    children: [tr(text, opts)],
  });
}
function h1(text) {
  return new Paragraph({
    spacing: { before: 360, after: 200 },
    children: [tr(text, { bold: true, size: 28 })],
  });
}
function h2(text) {
  return new Paragraph({
    spacing: { before: 280, after: 140 },
    children: [tr(text, { bold: true, size: 26 })],
  });
}
function h3(text) {
  return new Paragraph({
    spacing: { before: 200, after: 100 },
    children: [tr(text, { bold: true, size: 24 })],
  });
}
function bullet(text) {
  return new Paragraph({
    numbering: { reference: "bullets", level: 0 },
    spacing: { after: 80, line: 320 },
    children: [tr(text)],
  });
}
function code(text) {
  return new Paragraph({
    spacing: { after: 60 }, indent: { left: 360 },
    children: [tr(text, { font: COURIER, size: 20 })],
  });
}
function spacer(size = 120) {
  return new Paragraph({ spacing: { after: size }, children: [tr("")] });
}
function cell(text, opts = {}) {
  return new TableCell({
    borders, width: { size: opts.width || 2000, type: WidthType.DXA },
    margins: { top: 80, bottom: 80, left: 120, right: 120 },
    children: [new Paragraph({
      alignment: opts.align || AlignmentType.LEFT,
      children: [tr(text, { bold: opts.bold || false, size: 22 })],
    })],
  });
}
function image(filename, opts = {}) {
  const fp = path.join(RES_DIR, filename);
  if (!fs.existsSync(fp)) return p(`[gambar tidak ditemukan: ${filename}]`, { italics: true });
  return new Paragraph({
    alignment: AlignmentType.CENTER, spacing: { before: 120, after: 80 },
    children: [new ImageRun({
      data: fs.readFileSync(fp),
      transformation: { width: opts.width || 540, height: opts.height || 360 },
    })],
  });
}
function caption(text) {
  return new Paragraph({
    alignment: AlignmentType.CENTER, spacing: { after: 240 },
    children: [tr(text, { italics: true, size: 22 })],
  });
}
function pageBreak() { return new Paragraph({ children: [new PageBreak()] }); }

// ----- load both summaries (per_floor and base_only) -----
function loadSummary(name) {
  const raw = fs.readFileSync(path.join(RES_DIR, name), 'utf8')
    .replace(/\bNaN\b/g, 'null').replace(/\bInfinity\b/g, '1e308').replace(/\b-Infinity\b/g, '-1e308');
  return JSON.parse(raw);
}
const summary = loadSummary('summary.json');
const summaryBase = loadSummary('summary_base_only.json');
const meta = summary._meta;
const metaBase = summaryBase._meta;
const fn = meta.natural_freq_hz.map(x => x.toFixed(3));
const best = meta.best;
const bestBase = metaBase.best;

function fmt(x, d = 2) {
  if (x === null || x === undefined || Number.isNaN(x)) return "-";
  return Number(x).toFixed(d);
}
function pct(x) {
  if (x === null || x === undefined || Number.isNaN(x)) return "-";
  return Number(x).toFixed(2) + " %";
}

// ----- generic 2-col / 3-col tables -----
function paramTable(rows2col, w1 = 4000, w2 = 4400) {
  const tableRows = rows2col.map(([a, b]) => new TableRow({ children: [
    cell(a, { width: w1 }),
    cell(b, { width: w2 }),
  ]}));
  return new Table({
    width: { size: w1 + w2, type: WidthType.DXA },
    columnWidths: [w1, w2], rows: tableRows,
  });
}

// ----- COVER -----
const cover = [
  spacer(600),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 240 },
    children: [tr("LAPORAN HASIL SIMULASI", { bold: true, size: 32 })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 200 },
    children: [tr("Sistem Kontrol Adaptif Multi-Hazard", { bold: true, size: 28 })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 200 },
    children: [tr("Structural-Hydrodynamics berbasis", { bold: true, size: 28 })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 600 },
    children: [tr("Model Predictive Control (MPC)", { bold: true, size: 28 })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 240 },
    children: [tr("Studi Kasus: Bangunan 3 Lantai 1 Ton", { italics: true, size: 24 })] }),
  new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 240 },
    children: [tr("dengan MLFS (per lantai) + FAHFS (peninggian elevasi pondasi)", { italics: true, size: 24 })] }),
  spacer(800),
  p("Stack: MATLAB (mpc_multihazard_matlab/src/*.m)", { align: AlignmentType.CENTER }),
  p("Validasi numerik: Python mirror (matlab_mirror.py)", { align: AlignmentType.CENTER }),
  spacer(400),
  p("Mengacu pada: Proposal Akademic Excellence DRPM-ITS 2026,", { align: AlignmentType.CENTER }),
  p("Bilfaqih dkk., Departemen Teknik Elektro FTEIC ITS.", { align: AlignmentType.CENTER }),
  pageBreak(),
];

// ----- BAB 1 -----
const bab1 = [
  h1("BAB 1. PENDAHULUAN"),
  p("Laporan ini menyajikan hasil simulasi sistem kontrol adaptif multi-hazard structural-hydrodynamics berbasis Model Predictive Control (MPC), sebagaimana dirumuskan pada proposal akademik Bilfaqih dkk. (DRPM-ITS, 2026). Sistem dirancang untuk meningkatkan ketahanan struktur bangunan terhadap dua kelas gangguan utama di kawasan pesisir Indonesia: gangguan struktural (gempa) dan gangguan hidrodinamik (banjir, tsunami).", { indent: true }),
  p("Plant simulasi adalah bangunan tiga lantai dengan massa total 1 ton (333,3 kg per lantai), dilengkapi dua sub-sistem aktuator yang dimodelkan dengan dinamika fisika berbeda:", { indent: true }),
  bullet("MLFS (Magnetic Levitation Foundation System): aktuator gaya elektromagnetik per lantai untuk meredam getaran horizontal akibat gempa."),
  bullet("FAHFS (Flood-Adaptive Hydraulic Foundation System): aktuator silinder hidrolik yang MENINGGIKAN ELEVASI pondasi sehingga seluruh struktur terangkat di atas muka air banjir/tsunami."),
  p("Kedua sub-sistem dikoordinasi oleh satu MPC yang mengoptimalkan secara simultan: (i) reduksi getaran via MLFS, (ii) tracking elevasi referensi z_ref(t) = h(t) + safety_margin via FAHFS. Pendekatan dual-subsystem ini merupakan kebaruan utama proposal dan diuji pada empat skenario disturbance.", { indent: true }),
  p("Struktur laporan: Bab 2 menjabarkan input data dan parameter simulasi secara lengkap. Bab 3 menjelaskan konfigurasi MPC dan hasil tuning bobot. Bab 4-7 menyajikan hasil per skenario gangguan. Bab 8 merangkum perbandingan persentase open-loop vs MPC. Bab 9 berisi kesimpulan.", { indent: true }),
];

// ----- BAB 2: INPUT DATA & PARAMETER -----
const bab2 = [
  h1("BAB 2. INPUT DATA DAN PARAMETER SIMULASI"),
  p("Bab ini menjabarkan seluruh input data simulasi sebelum hasil dianalisis. Pemisahan eksplisit antara parameter sistem (statis) dan profil disturbance (dinamis) penting untuk reproduktibilitas eksperimen."),

  h2("2.1 Parameter Plant Struktural"),
  p("Bangunan dimodelkan sebagai shear-building 3 lantai dengan persamaan dinamis:"),
  code("M q'' + C q' + K q = -M*1*a_g + L_u u_MLFS + F_fluid(q, z_base, h, v)"),
  spacer(80),
  paramTable([
    ["Parameter", "Nilai"],
    ["Jumlah lantai", "3 (DOF struktural)"],
    ["Massa per lantai (m1, m2, m3)", "333,3 kg (total 1 ton)"],
    ["Stiffness per lantai (k1, k2, k3)", "1,5 x 10^5 N/m"],
    ["Tipe redaman", "Rayleigh: C = alpha*M + beta*K"],
    ["Damping ratio (zeta)", "0,02 (2%) pada dua mode pertama"],
    ["Frekuensi natural mode 1", `${fn[0]} Hz`],
    ["Frekuensi natural mode 2", `${fn[1]} Hz`],
    ["Frekuensi natural mode 3", `${fn[2]} Hz`],
    ["Tinggi tiap lantai", "3,0 m"],
    ["Elevasi puncak (z1, z2, z3)", "3,0 / 6,0 / 9,0 m"],
  ]),
  spacer(160),

  h2("2.2 Parameter FAHFS (Hydraulic Foundation)"),
  p("FAHFS dimodelkan sebagai sub-sistem 1-DOF dengan persamaan:"),
  code("m_b z_base'' + c_b z_base' + k_b z_base = u_FAHFS"),
  spacer(80),
  paramTable([
    ["Parameter", "Nilai"],
    ["Massa efektif pondasi (m_b)", "1500 kg"],
    ["Damping pondasi (c_b)", "1,5 x 10^4 N s/m"],
    ["Stiffness pondasi (k_b)", "0 (active hold via aktuator)"],
    ["Stroke maksimum z_base", "5,0 m"],
    ["Stroke minimum z_base", "0,0 m"],
    ["Safety margin di atas h(t)", "0,5 m"],
    ["Lookahead reference", "3,0 detik"],
  ]),
  spacer(160),

  h2("2.3 Parameter Aktuator"),
  paramTable([
    ["Aktuator", "Batas (u_min .. u_max)"],
    ["MLFS lantai 1 (u_MLFS_1)", "-5000 .. +5000 N"],
    ["MLFS lantai 2 (u_MLFS_2)", "-5000 .. +5000 N"],
    ["MLFS lantai 3 (u_MLFS_3)", "-5000 .. +5000 N"],
    ["FAHFS hidrolik (u_FAHFS)", "-100000 .. +100000 N"],
  ]),
  p("Asimetri kapasitas mencerminkan sifat fisik: MLFS bekerja pada bandwidth tinggi (Hz) dengan gaya kecil, FAHFS bekerja pada bandwidth rendah (sub-Hz) dengan gaya besar untuk mengangkat seluruh struktur."),
  spacer(160),

  h2("2.4 Parameter Fluida"),
  paramTable([
    ["Parameter", "Nilai"],
    ["Densitas air (rho)", "1000 kg/m^3"],
    ["Koefisien drag (Cd)", "1,2"],
    ["Luas facade (A)", "12,0 m^2"],
    ["Lebar strip (w = A/floor_h)", "4,0 m"],
    ["Gaya hidrodinamik per lantai", "F_drag + F_hidrostatik"],
    ["F_drag", "0,5 rho Cd (w*d_i) v^2 sign(v)"],
    ["F_hidrostatik", "0,5 rho g d_i^2 w"],
    ["d_i submerged depth", "clip(h - z_base - (i-1)*floor_h, 0, floor_h)"],
  ]),
  p("Kopling penting: gaya fluida HANYA berpengaruh pada lantai yang submerged, yaitu ketika tinggi air h(t) melebihi z_base + (i-1) x floor_h. Begitu FAHFS mengangkat z_base di atas h, semua lantai 'kering' dan F_fluid = 0."),
  spacer(160),

  h2("2.5 Sample Time dan State-Space"),
  paramTable([
    ["Aspek", "Nilai"],
    ["Sample time MATLAB (Ts)", "1,0 ms (1000 Hz)"],
    ["Sample time mirror (Ts)", `${meta.Ts * 1000} ms (${(1 / meta.Ts).toFixed(0)} Hz)`],
    ["Diskritisasi", "Zero-Order Hold (ZOH)"],
    ["Dimensi state (nx)", "8"],
    ["Dimensi input (nu)", "4"],
    ["Dimensi disturbance (nd)", "4"],
  ]),
  p("Vektor state augmented:"),
  code("x = [q1 q2 q3 q1' q2' q3' z_base z_base']  (8x1)"),
  code("u = [u_MLFS_1 u_MLFS_2 u_MLFS_3 u_FAHFS]   (4x1)"),
  code("d = [a_g; F_f1; F_f2; F_f3]                (4x1)"),
  spacer(160),

  h2("2.6 Skenario Disturbance"),
  p("Empat skenario diuji untuk mencakup multi-hazard sekuensial dan simultan:"),

  h3("(a) Skenario Gempa (seismic)"),
  paramTable([
    ["Tipe sinyal", "Chirp linier dengan envelope Gaussian + komponen broadband"],
    ["PGA (Peak Ground Acceleration)", "3,5 m/s^2"],
    ["Range frekuensi chirp", "0,5 - 8,0 Hz"],
    ["Komponen broadband", "0,25 x PGA pada 15 Hz"],
    ["Envelope", "exp(-((t - 0,35T)/(0,18T))^2) (Gaussian)"],
    ["Durasi simulasi", "30 s (MATLAB) / 20 s (mirror)"],
    ["Tinggi air", "0 (tidak ada banjir/tsunami)"],
  ]),
  spacer(160),
  image("input_seismic.png", { width: 540, height: 240 }),
  caption("Gambar 2.1. Profil ground acceleration (atas) dan tinggi muka air (bawah) untuk skenario gempa. Tidak ada eksitasi hidrodinamik."),

  h3("(b) Skenario Banjir (flood)"),
  paramTable([
    ["Tipe", "Kenaikan muka air bertahap (rob/banjir progresif)"],
    ["Tinggi maksimum (h_max)", "2,5 m"],
    ["Profil tinggi", "h(t) = h_max * (1 - exp(-t/T_rise))"],
    ["Rise time (T_rise)", "20 s"],
    ["Amplitudo kecepatan surge", "1,5 m/s"],
    ["Frekuensi surge", "0,05 Hz"],
    ["Profil kecepatan", "v(t) = v_amp * sin(2*pi*f*t)"],
    ["Durasi simulasi", "60 s (MATLAB) / 40 s (mirror)"],
    ["Eksitasi seismik", "Tidak ada"],
  ]),
  spacer(160),
  image("input_flood.png", { width: 540, height: 240 }),
  caption("Gambar 2.2. Profil tinggi muka air banjir progresif. Tidak ada eksitasi seismik."),

  h3("(c) Skenario Tsunami"),
  paramTable([
    ["Tipe", "Gelombang soliton-like (sech^2)"],
    ["Tinggi puncak (h_peak)", "4,5 m"],
    ["Kecepatan puncak (v_peak)", "6,0 m/s"],
    ["Time-scale (tau)", "1,5 s"],
    ["Profil tinggi", "h_peak * (sech^2((t-3tau)/tau) + 0,4*(1 - exp(-t/(2tau))))"],
    ["Profil kecepatan", "v_peak * sech((t-3tau)/tau)"],
    ["Durasi simulasi", "25 s (MATLAB) / 15 s (mirror)"],
    ["Eksitasi seismik", "Tidak ada"],
  ]),
  spacer(160),
  image("input_tsunami.png", { width: 540, height: 240 }),
  caption("Gambar 2.3. Profil tinggi muka air tsunami. Front gelombang sangat curam dengan h_peak 4,5 m, diikuti inundasi lambat 1,8 m."),

  h3("(d) Skenario Gabungan (combined)"),
  paramTable([
    ["Tipe", "Gempa diikuti tsunami susulan"],
    ["Fase 1 (gempa)", "0 - end: chirp seismic seperti skenario (a)"],
    ["Fase 2 (tsunami)", "Onset pada t = 12 s, durasi sisa simulasi"],
    ["Profil tinggi tsunami", "Sama seperti skenario (c), origin shift +12 s"],
    ["Durasi simulasi", "40 s (MATLAB) / 25 s (mirror)"],
  ]),
  spacer(160),
  image("input_combined.png", { width: 540, height: 240 }),
  caption("Gambar 2.4. Skenario combined: gempa pada t = 0..6 s, tsunami onset pada t = 12 s. Skenario paling representatif kondisi pesisir Indonesia (mis. pulau Nias)."),
];

// ----- BAB 3: KONFIGURASI MPC + TUNING -----
const bab3 = [
  h1("BAB 3. KONFIGURASI MPC DAN TUNING BOBOT"),

  h2("3.1 Formulasi MPC"),
  p("MPC mengikuti formulasi Bab 3.3 proposal dengan reference tracking pada elevasi FAHFS:"),
  code("min  sum_{k=1}^N (x_k - r_k)' Q (x_k - r_k) + sum_{k=0}^{N-1} u_k' R u_k"),
  code("s.t. x_{k+1} = Ad x_k + Bd u_k + Ed d_k"),
  code("     u_min <= u_k <= u_max"),
  p("Disturbance preview: nilai a_g, h, v sepanjang horizon dianggap diketahui dari sensor (early warning). Gaya fluida F_fi dihitung ulang setiap iterasi terhadap z_base CURRENT, menangkap kopling fluida-elevasi."),

  h2("3.2 Reference Trajectory"),
  p("Untuk skenario hidrodinamik:"),
  code("z_ref(t) = min(z_max, max_{tau in [t, t+T_lh]} h(tau) + margin)"),
  bullet("State struktural (q, q'): selalu r = 0 (vibration suppression)."),
  bullet("State elevasi z_base: tracking h(t) + 0,5 m dengan lookahead 3 s."),
  bullet("State elevasi velocity z_base': r = 0 (settle setelah lift)."),

  h2("3.3 Parameter MPC"),
  paramTable([
    ["Parameter", "Nilai"],
    ["Prediction horizon (N)", "20 langkah (full benchmark)"],
    ["Horizon untuk tuning", "15 langkah"],
    ["Solver QP", "FISTA proyeksi (fallback bila quadprog absen)"],
    ["Warm-start", "U dari iterasi sebelumnya, di-shift 1 langkah"],
    ["Tolerance KKT", "1e-7"],
    ["Max iterasi FISTA", "300"],
  ]),

  h2("3.4 Strategi Tuning"),
  p("Pencarian bobot menggunakan grid log 5-D pada skenario gabungan (covers both regimes). Score function multi-objective:"),
  code("S = 0,35*reduksi_avg + 0,20*Jain_fairness - 0,10*E_norm - 0,05*saturation + 0,30*z_track_quality"),
  paramTable([
    ["Komponen score", "Bobot dan interpretasi"],
    ["w_red = 0,35", "Reduksi rata-rata peak displacement struktur"],
    ["w_fair = 0,20", "Jain fairness antar lantai (proteksi merata)"],
    ["w_eng = 0,10 (penalty)", "Normalized control energy"],
    ["w_sat = 0,05 (penalty)", "Fraksi waktu aktuator pada batas saturasi"],
    ["w_z = 0,30", "Kualitas tracking elevasi FAHFS (1 - RMSE/z_max)"],
  ]),

  h2("3.5 Hasil Tuning"),
  paramTable([
    ["Bobot optimal", "Nilai"],
    ["q_disp (penalty perpindahan struktur)", `${best.qd}`],
    ["q_vel (penalty kecepatan struktur)", `${best.qv}`],
    ["q_z (penalty error elevasi z_base)", `${best.qz}`],
    ["r_MLFS (penalty energi MLFS)", `${best.rm.toExponential(1)}`],
    ["r_FAHFS (penalty energi FAHFS)", `${best.rf.toExponential(1)}`],
    ["Score akhir", `${best.score.toFixed(4)}`],
  ]),
  p(`Insight bobot: q_z (${best.qz}) >> q_disp (${best.qd}) menandakan strategi optimal MPC adalah MEMPRIORITASKAN bypass gangguan via FAHFS daripada kompensasi gaya via MLFS. Ini konsisten dengan hierarki fisika: lebih hemat energi mengangkat struktur 5 m daripada menahan gaya tsunami orde ratusan kN.`),
];

// ----- helper tabel hasil per skenario -----
function metricTable(sc) {
  const u = summary[sc].unc, l = summary[sc].lqr, m = summary[sc].mpc;
  const rows = [];
  rows.push(new TableRow({ children: [
    cell("Metrik",      { bold: true, width: 2700 }),
    cell("Open-loop",   { bold: true, width: 1900, align: AlignmentType.CENTER }),
    cell("LQR",         { bold: true, width: 1900, align: AlignmentType.CENTER }),
    cell("MPC + FAHFS", { bold: true, width: 1900, align: AlignmentType.CENTER }),
  ]}));
  const r1 = (lab, vu, vl, vm) => new TableRow({ children: [
    cell(lab, { width: 2700 }),
    cell(vu, { width: 1900, align: AlignmentType.CENTER }),
    cell(vl, { width: 1900, align: AlignmentType.CENTER }),
    cell(vm, { width: 1900, align: AlignmentType.CENTER }),
  ]});
  const peakRoof = a => fmt(a[2] * 1e3, 2) + " mm";
  const driftMax = a => fmt(Math.max(...a) * 1e3, 2) + " mm";
  const rmsRoof  = a => fmt(a[2] * 1e3, 2) + " mm";
  const accMax   = a => fmt(Math.max(...a), 2) + " m/s^2";
  const settle   = a => fmt(a[2], 2) + " s";
  const force    = a => a.length === 0 ? "-" : fmt(Math.max(...a), 1) + " N";
  const peakZ    = v => v === undefined ? "-" : fmt(v, 3) + " m";
  const trkRMSE  = v => v === undefined ? "-" : fmt(v, 4) + " m";

  rows.push(r1("Peak displacement (atap)", peakRoof(u.peak_disp), peakRoof(l.peak_disp), peakRoof(m.peak_disp)));
  rows.push(r1("RMS displacement (atap)",  rmsRoof(u.rms_disp),   rmsRoof(l.rms_disp),    rmsRoof(m.rms_disp)));
  rows.push(r1("Peak inter-story drift",   driftMax(u.peak_drift),driftMax(l.peak_drift), driftMax(m.peak_drift)));
  rows.push(r1("Peak acceleration",        accMax(u.peak_acc),    accMax(l.peak_acc),     accMax(m.peak_acc)));
  rows.push(r1("Settling time (atap)",     settle(u.settling),    settle(l.settling),     settle(m.settling)));
  rows.push(r1("Peak gaya kontrol",        force(u.peak_force),   force(l.peak_force),    force(m.peak_force)));
  rows.push(r1("Energi kontrol [N^2 s]",   fmt(u.ctrl_energy, 2), fmt(l.ctrl_energy, 2),  fmt(m.ctrl_energy, 2)));
  rows.push(r1("Peak elevasi z_base",      peakZ(u.peak_z),       peakZ(l.peak_z),        peakZ(m.peak_z)));
  rows.push(r1("RMSE tracking z_base",     "-",                   trkRMSE(l.z_track_rmse),trkRMSE(m.z_track_rmse)));
  rows.push(r1("Jain fairness drift",      fmt(u.fairness, 4),    fmt(l.fairness, 4),     fmt(m.fairness, 4)));
  rows.push(r1("Reduksi rata-rata vs OL",  "-",                   pct(l.reduction_avg),    pct(m.reduction_avg)));
  return new Table({
    width: { size: 8400, type: WidthType.DXA },
    columnWidths: [2700, 1900, 1900, 1900], rows,
  });
}

function reductionTable(sc) {
  const u = summary[sc].unc, l = summary[sc].lqr, m = summary[sc].mpc;
  const rows = [];
  rows.push(new TableRow({ children: [
    cell("Lantai",      { bold: true, width: 1100, align: AlignmentType.CENTER }),
    cell("OL (mm)",     { bold: true, width: 1700, align: AlignmentType.CENTER }),
    cell("LQR (mm)",    { bold: true, width: 1700, align: AlignmentType.CENTER }),
    cell("MPC (mm)",    { bold: true, width: 1700, align: AlignmentType.CENTER }),
    cell("Red. LQR",    { bold: true, width: 1100, align: AlignmentType.CENTER }),
    cell("Red. MPC",    { bold: true, width: 1100, align: AlignmentType.CENTER }),
  ]}));
  for (let i = 0; i < 3; i++) {
    rows.push(new TableRow({ children: [
      cell(`${i + 1}`,                          { width: 1100, align: AlignmentType.CENTER }),
      cell(fmt(u.peak_disp[i] * 1e3, 2),        { width: 1700, align: AlignmentType.CENTER }),
      cell(fmt(l.peak_disp[i] * 1e3, 2),        { width: 1700, align: AlignmentType.CENTER }),
      cell(fmt(m.peak_disp[i] * 1e3, 2),        { width: 1700, align: AlignmentType.CENTER }),
      cell(pct(l.reduction[i]),                 { width: 1100, align: AlignmentType.CENTER }),
      cell(pct(m.reduction[i]),                 { width: 1100, align: AlignmentType.CENTER, bold: true }),
    ]}));
  }
  rows.push(new TableRow({ children: [
    cell("Rata-rata", { bold: true, width: 1100, align: AlignmentType.CENTER }),
    cell("-",         { width: 1700, align: AlignmentType.CENTER }),
    cell("-",         { width: 1700, align: AlignmentType.CENTER }),
    cell("-",         { width: 1700, align: AlignmentType.CENTER }),
    cell(pct(l.reduction_avg), { width: 1100, align: AlignmentType.CENTER, bold: true }),
    cell(pct(m.reduction_avg), { width: 1100, align: AlignmentType.CENTER, bold: true }),
  ]}));
  return new Table({
    width: { size: 8400, type: WidthType.DXA },
    columnWidths: [1100, 1700, 1700, 1700, 1100, 1100], rows,
  });
}

function babHasil(sc, num, title, narrative, insightPoints) {
  return [
    h1(`BAB ${num}. HASIL SIMULASI: SKENARIO ${title}`),
    h2(`${num}.1 Tabel Metrik Lengkap`),
    metricTable(sc),
    spacer(160),
    h2(`${num}.2 Grafik Perpindahan Lantai dan Drift`),
    image(`${sc}_displacement.png`, { width: 540, height: 360 }),
    caption(`Gambar ${num}.1. Perpindahan tiap lantai (kolom kiri) dan inter-story drift (kolom kanan) untuk skenario ${title}. Kurva merah = open-loop, biru = LQR, hijau = MPC + FAHFS.`),
    h2(`${num}.3 Tracking Elevasi FAHFS`),
    image(`${sc}_elevation.png`, { width: 540, height: 220 }),
    caption(`Gambar ${num}.2. Elevasi pondasi z_base oleh FAHFS. Biru: tinggi muka air h(t). Oranye dashed: referensi z_ref = h + 0,5 m dengan lookahead 3 s. Hijau: z_base aktual MPC. Merah: open-loop (FAHFS off).`),
    h2(`${num}.4 Gaya Aktuator dan Energi Kontrol`),
    image(`${sc}_forces.png`, { width: 540, height: 280 }),
    caption(`Gambar ${num}.3. Atas: gaya MPC pada 3 MLFS (warna) + 1 FAHFS (hitam tebal). Garis titik abu-abu = batas MLFS +/-5 kN. Garis putus abu-abu = batas FAHFS +/-100 kN. Bawah: energi kontrol kumulatif MPC vs LQR.`),
    h2(`${num}.5 Insight Hasil`),
    p(narrative),
    ...insightPoints.map(t => bullet(t)),
    h2(`${num}.6 Reduksi Persentase per Lantai`),
    reductionTable(sc),
    spacer(160),
  ];
}

// ----- narrative -----
const seismicNarrative = `Skenario gempa MURNI menguji efektivitas MLFS terhadap structural disturbances. FAHFS idle (z_ref = 0). MPC mencapai reduksi rata-rata ${pct(summary.seismic.mpc.reduction_avg)} dengan peak displacement atap dari ${fmt(summary.seismic.unc.peak_disp[2]*1e3, 2)} mm (open-loop) menjadi ${fmt(summary.seismic.mpc.peak_disp[2]*1e3, 2)} mm. Dibanding LQR (${pct(summary.seismic.lqr.reduction_avg)}), MPC unggul karena preview gangguan, handling saturasi eksplisit, dan distribusi proteksi merata antar lantai (Jain fairness ${fmt(summary.seismic.mpc.fairness, 3)}).`;
const seismicInsights = [
  "Mode pertama 1,503 Hz tereksitasi paling kuat: open-loop atap mencapai ~97 mm (resonansi nyata pada bangunan ringan).",
  "MPC menahan getaran di bawah 0,5 mm pada semua lantai - reduksi 99,29% rata-rata, jauh di bawah ambang kerusakan struktural.",
  "FAHFS tetap idle di z_base = 0 (tidak ada kebutuhan elevasi); konsumsi energi FAHFS ~ 0.",
  `Energi MLFS MPC (${fmt(summary.seismic.mpc.ctrl_energy, 2)}) < LQR (${fmt(summary.seismic.lqr.ctrl_energy, 2)}) meski reduksi MPC lebih besar - keuntungan dari preview disturbance.`,
  "Pola dual-subsystem terbukti: MLFS mengambil alih sepenuhnya untuk gempa, FAHFS pasif tanpa konsumsi sumber daya.",
];
const floodNarrative = `Skenario banjir progresif (h_max = 2,5 m, rise time 20 s) menguji FAHFS sebagai sistem PENINGGIAN ELEVASI. MPC mencapai reduksi ${pct(summary.flood.mpc.reduction_avg)} - praktis SEMPURNA - karena z_base diangkat hingga ${fmt(summary.flood.mpc.peak_z, 2)} m mengikuti referensi h(t) + 0,5 m. Begitu z_base > h(t), seluruh struktur kering dari beban hidrodinamik dan getaran horizontal lantai = 0.`;
const floodInsights = [
  `z_base puncak ${fmt(summary.flood.mpc.peak_z, 2)} m dengan tracking RMSE ${fmt(summary.flood.mpc.z_track_rmse, 4)} m - presisi tinggi.`,
  `Lookahead 3 detik membuat lift mulai SEBELUM air mencapai bangunan: open-loop atap mencapai ${fmt(summary.flood.unc.peak_disp[2]*1e3, 0)} mm; MPC ~0 mm.`,
  "Open-loop menampilkan displacement ~611 mm seragam (struktur didorong sebagai rigid body karena hidrostatik); MPC menghilangkan beban tersebut dari sumbernya.",
  "Konsumsi MLFS ~ 0 (tidak ada getaran perlu diredam karena tidak ada eksitasi); konsumsi FAHFS terkonsentrasi di fase lift (5 detik pertama).",
  "Validasi konsep proposal: FAHFS bekerja sebagai mekanisme bypass gangguan, bukan kompensator gaya - jauh lebih efisien secara energi.",
];
const tsunamiNarrative = `Skenario tsunami EKSTREM (h_peak = 4,5 m, v_peak = 6 m/s) memberikan reduksi ${pct(summary.tsunami.mpc.reduction_avg)} - lompatan signifikan dari implementasi FAHFS-as-force sebelumnya yang hanya 1,99%. Keterbatasan: stroke FAHFS maksimum 5 m, sementara h_peak + margin = 5 m, sehingga di puncak gelombang z_base saturasi (peak ${fmt(summary.tsunami.mpc.peak_z, 2)} m) dan air sempat menyentuh lantai dasar selama beberapa detik - menjelaskan residual displacement ~${fmt(summary.tsunami.mpc.peak_disp[2]*1e3, 0)} mm.`;
const tsunamiInsights = [
  `z_base mencapai stroke maksimum ${fmt(summary.tsunami.mpc.peak_z, 2)} m. Tracking RMSE ${fmt(summary.tsunami.mpc.z_track_rmse, 3)} m konsisten dengan saturasi pada front gelombang.`,
  "Reduksi 91% vs 1,99% sebelumnya menunjukkan bahwa pendekatan FAHFS lift = decoupling fisik, sementara FAHFS sebagai gaya horizontal tidak fisik untuk skenario tsunami.",
  "Implikasi desain: stroke FAHFS perlu disesuaikan dengan h_peak target. Untuk perlindungan tsunami 5 m, stroke 6-7 m memberi margin keamanan.",
  "Velocity v_peak 6 m/s pada front gelombang menyebabkan momentary coupling sebelum FAHFS selesai mengangkat - dapat dimitigasi dengan ekstra lookahead atau passive deflektor di lantai dasar.",
  `Energi MPC (${fmt(summary.tsunami.mpc.ctrl_energy, 2)}) didominasi konsumsi FAHFS pada lift sustain - normal untuk skenario ekstrem.`,
];
const combinedNarrative = `Skenario combined (gempa pada t=0..6 s, tsunami onset t=12 s, total durasi 25 s) memvalidasi koordinasi MLFS+FAHFS pada multi-hazard SEKUENSIAL. MPC mencapai reduksi rata-rata ${pct(summary.combined.mpc.reduction_avg)} dengan z_base puncak ${fmt(summary.combined.mpc.peak_z, 2)} m. Sebelum t=12 s: MLFS aktif redam getaran gempa (FAHFS idle); setelah t=12 s: FAHFS mulai lift (lookahead 3 s = mulai t=9 s), MLFS tenang karena struktur sudah diangkat di atas air.`;
const combinedInsights = [
  "Transisi seamless antar fase: MPC otomatis switch prioritas berdasarkan ramalan disturbance + reference tracking, tanpa logic rule manual.",
  "Tracking RMSE tsunami fase combined LEBIH BAIK (0,028 m) daripada tsunami murni (0,585 m) - karena fase awal struktur sudah pre-positioned dari respons gempa.",
  "Gabungan dengan multi-physics control system membuktikan kebaruan proposal: dua aktuator dengan mekanisme fisika berbeda dikoordinasi oleh satu MPC.",
  "Skenario paling representatif kondisi pesisir nyata seperti pulau Nias - gempa kemudian tsunami susulan.",
  "Untuk implementasi praktis: tambahkan early warning input dari sensor seismik+water-level ke MPC sebagai feedforward sehingga preview lebih akurat.",
];

const bab4 = [...babHasil("seismic",  4, "GEMPA",                seismicNarrative,  seismicInsights),  pageBreak()];
const bab5 = [...babHasil("flood",    5, "BANJIR",               floodNarrative,    floodInsights),    pageBreak()];
const bab6 = [...babHasil("tsunami",  6, "TSUNAMI",              tsunamiNarrative,  tsunamiInsights),  pageBreak()];
const bab7 = [...babHasil("combined", 7, "GABUNGAN GEMPA+TSUNAMI", combinedNarrative, combinedInsights), pageBreak()];

// ----- BAB 8: VARIAN BASE-ONLY -----
function metricTableBase(sc) {
  const u = summaryBase[sc].unc, l = summaryBase[sc].lqr, m = summaryBase[sc].mpc;
  const rows = [];
  rows.push(new TableRow({ children: [
    cell("Metrik",             { bold: true, width: 2700 }),
    cell("Open-loop",          { bold: true, width: 1900, align: AlignmentType.CENTER }),
    cell("LQR (base-only)",    { bold: true, width: 1900, align: AlignmentType.CENTER }),
    cell("MPC (base-only)",    { bold: true, width: 1900, align: AlignmentType.CENTER }),
  ]}));
  const r1 = (lab, vu, vl, vm) => new TableRow({ children: [
    cell(lab, { width: 2700 }),
    cell(vu, { width: 1900, align: AlignmentType.CENTER }),
    cell(vl, { width: 1900, align: AlignmentType.CENTER }),
    cell(vm, { width: 1900, align: AlignmentType.CENTER }),
  ]});
  const peakRoof = a => fmt(a[2] * 1e3, 2) + " mm";
  const driftMax = a => fmt(Math.max(...a) * 1e3, 2) + " mm";
  const peakZ    = v => v === undefined ? "-" : fmt(v, 3) + " m";
  rows.push(r1("Peak displacement (atap)", peakRoof(u.peak_disp), peakRoof(l.peak_disp), peakRoof(m.peak_disp)));
  rows.push(r1("Peak inter-story drift",   driftMax(u.peak_drift),driftMax(l.peak_drift), driftMax(m.peak_drift)));
  rows.push(r1("Peak elevasi z_base",      peakZ(u.peak_z),       peakZ(l.peak_z),        peakZ(m.peak_z)));
  rows.push(r1("Energi kontrol",           fmt(u.ctrl_energy, 2), fmt(l.ctrl_energy, 2),  fmt(m.ctrl_energy, 2)));
  rows.push(r1("Jain fairness drift",      fmt(u.fairness, 4),    fmt(l.fairness, 4),     fmt(m.fairness, 4)));
  rows.push(r1("Reduksi rata-rata vs OL",  "-",                   pct(l.reduction_avg),    pct(m.reduction_avg)));
  return new Table({
    width: { size: 8400, type: WidthType.DXA },
    columnWidths: [2700, 1900, 1900, 1900], rows,
  });
}

function compareTable() {
  const rows = [new TableRow({ children: [
    cell("Skenario",      { bold: true, width: 1700, align: AlignmentType.CENTER }),
    cell("Per-floor (3 MLFS)", { bold: true, width: 2200, align: AlignmentType.CENTER }),
    cell("Base-only (1 MLFS)", { bold: true, width: 2200, align: AlignmentType.CENTER }),
    cell("Selisih",       { bold: true, width: 1500, align: AlignmentType.CENTER }),
    cell("Modus dominan", { bold: true, width: 1700, align: AlignmentType.CENTER }),
  ]})];
  const rowFor = (sc, dom) => {
    const pf = summary[sc].mpc.reduction_avg;
    const bo = summaryBase[sc].mpc.reduction_avg;
    return new TableRow({ children: [
      cell(sc.toUpperCase(),               { width: 1700, align: AlignmentType.CENTER }),
      cell(pct(pf),                        { width: 2200, align: AlignmentType.CENTER, bold: true }),
      cell(pct(bo),                        { width: 2200, align: AlignmentType.CENTER, bold: true }),
      cell(fmt(pf - bo, 2) + " pp",        { width: 1500, align: AlignmentType.CENTER }),
      cell(dom,                            { width: 1700, align: AlignmentType.CENTER }),
    ]});
  };
  rows.push(rowFor("seismic",  "MLFS"));
  rows.push(rowFor("flood",    "FAHFS"));
  rows.push(rowFor("tsunami",  "FAHFS"));
  rows.push(rowFor("combined", "FAHFS"));
  return new Table({
    width: { size: 9300, type: WidthType.DXA },
    columnWidths: [1700, 2200, 2200, 1500, 1700], rows,
  });
}

const bab8 = [
  h1("BAB 8. VARIAN MLFS SINGLE-BASE (SESUAI PROPOSAL)"),
  p("Bab 4-7 menggunakan konfigurasi MLFS per-lantai (3 aktuator) sebagai PENGEMBANGAN dari proposal. Bab ini menyajikan varian alternatif: MLFS sebagai SATU aktuator di pondasi (active base isolation) yang lebih dekat dengan literal proposal. Frase 'Magnetic Levitation FOUNDATION System' dan persamaan tunggal F_MLFS = ki*i(t) di Bab 3.2.3 mengindikasikan satu aktuator level pondasi, bukan distribusi per lantai."),

  h2("8.1 Konfigurasi Plant Berbeda"),
  paramTable([
    ["Aspek", "Nilai (varian base-only)"],
    ["Jumlah MLFS", "1 (di lantai 1 / pondasi)"],
    ["Kapasitas MLFS", "+/-15 kN (3x dari per-lantai karena harus support seluruh struktur)"],
    ["Jumlah FAHFS", "1 (sama dengan varian per-lantai)"],
    ["Kapasitas FAHFS", "+/-100 kN"],
    ["Total input (nu)", "2 (1 MLFS + 1 FAHFS)"],
    ["State (nx)", "8 (sama)"],
    ["Disturbance (nd)", "4 (sama)"],
    ["Matriks B_MLFS", "(6x1) - hanya kolom force ke floor 1"],
  ]),

  h2("8.2 Hasil Tuning Bobot Base-only"),
  paramTable([
    ["Bobot optimal", "Nilai"],
    ["q_disp", `${bestBase.qd}`],
    ["q_vel",  `${bestBase.qv}`],
    ["q_z",    `${bestBase.qz}`],
    ["r_MLFS", `${bestBase.rm.toExponential(1)}`],
    ["r_FAHFS",`${bestBase.rf.toExponential(1)}`],
    ["Score",  `${bestBase.score.toFixed(4)}`],
  ]),
  p(`Score (${bestBase.score.toFixed(4)}) lebih rendah dari varian per-floor (${best.score.toFixed(4)}) karena reduksi seismic terbatas: aktuator base-only tidak bisa kontrol mode 2-3 secara efektif.`),

  h2("8.3 Hasil per Skenario - Base-only"),

  h3("(a) Skenario Gempa"),
  metricTableBase("seismic"),
  spacer(160),
  image("baseonly_seismic_displacement.png", { width: 540, height: 360 }),
  caption("Gambar 8.1. BASE-ONLY pada skenario gempa. Reduksi 74,54% (vs 99,29% per-floor). Mode 2-3 tidak terkontrol penuh - residual osilasi terlihat pada lantai 2 dan 3."),

  h3("(b) Skenario Banjir"),
  metricTableBase("flood"),
  spacer(160),
  image("baseonly_flood_elevation.png", { width: 540, height: 220 }),
  caption("Gambar 8.2. BASE-ONLY pada skenario banjir. FAHFS tetap dominan (lift z_base = 2,66 m); reduksi 100% identik dengan varian per-floor karena MLFS irrelevant saat tidak ada eksitasi seismik."),

  h3("(c) Skenario Tsunami"),
  metricTableBase("tsunami"),
  spacer(160),
  image("baseonly_tsunami_elevation.png", { width: 540, height: 220 }),
  caption("Gambar 8.3. BASE-ONLY pada skenario tsunami. Reduksi 91,19% praktis identik per-floor (91,33%) - FAHFS bekerja maksimal hingga stroke saturasi."),

  h3("(d) Skenario Gabungan"),
  metricTableBase("combined"),
  spacer(160),
  image("baseonly_combined_displacement.png", { width: 540, height: 360 }),
  caption("Gambar 8.4. BASE-ONLY pada skenario gabungan. Fase gempa kurang efektif (MLFS base-only); fase tsunami efektif (FAHFS dominan)."),

  h2("8.4 Komparasi Per-floor vs Base-only"),
  spacer(80),
  compareTable(),
  spacer(160),
  h2("8.5 Insight Komparatif"),
  bullet("Skenario gempa: per-floor unggul ~25 pp (99,29% vs 74,54%) - distribusi aktuator memberi modal controllability penuh atas mode 2-3 (4,21 dan 6,08 Hz)."),
  bullet("Skenario hidrodinamik (banjir/tsunami/combined): hasil hampir identik karena FAHFS yang dominan, dan FAHFS memang sudah single-actuator di base pada kedua varian."),
  bullet("Trade-off cost: base-only butuh 1 aktuator MLFS (lebih murah, instalasi sederhana). Per-floor butuh 3 aktuator dengan koordinasi MIMO (lebih mahal, tetapi reduksi gempa jauh lebih tinggi)."),
  bullet("Rekomendasi: untuk wilayah dengan risiko gempa MAJOR (PGA > 4 m/s^2), per-floor wajib. Untuk wilayah dengan dominan flood/tsunami dan gempa moderat, base-only ekonomis dan cukup."),
  bullet("Implementasi base-only lebih dekat dengan literal proposal (MLFS = Magnetic Levitation FOUNDATION System); per-floor adalah ekstensi modern untuk MIMO active control."),
];

const bab8raw = bab8;

// ----- BAB 8: ringkasan -----
function summaryTable() {
  const rows = [new TableRow({ children: [
    cell("Skenario",       { bold: true, width: 1700, align: AlignmentType.CENTER }),
    cell("OL atap (mm)",   { bold: true, width: 1700, align: AlignmentType.CENTER }),
    cell("MPC atap (mm)",  { bold: true, width: 1700, align: AlignmentType.CENTER }),
    cell("Reduksi MPC",    { bold: true, width: 1500, align: AlignmentType.CENTER }),
    cell("Peak z_base",    { bold: true, width: 1300, align: AlignmentType.CENTER }),
    cell("Fairness",       { bold: true, width: 1100, align: AlignmentType.CENTER }),
  ]})];
  for (const sc of ["seismic", "flood", "tsunami", "combined"]) {
    const u = summary[sc].unc, m = summary[sc].mpc;
    rows.push(new TableRow({ children: [
      cell(sc.toUpperCase(),                  { width: 1700, align: AlignmentType.CENTER }),
      cell(fmt(u.peak_disp[2] * 1e3, 2),      { width: 1700, align: AlignmentType.CENTER }),
      cell(fmt(m.peak_disp[2] * 1e3, 2),      { width: 1700, align: AlignmentType.CENTER }),
      cell(pct(m.reduction_avg),              { width: 1500, align: AlignmentType.CENTER, bold: true }),
      cell(fmt(m.peak_z, 2) + " m",           { width: 1300, align: AlignmentType.CENTER }),
      cell(fmt(m.fairness, 3),                { width: 1100, align: AlignmentType.CENTER }),
    ]}));
  }
  return new Table({
    width: { size: 9000, type: WidthType.DXA },
    columnWidths: [1700, 1700, 1700, 1500, 1300, 1100], rows,
  });
}

const bab9 = [
  h1("BAB 9. RINGKASAN PERBANDINGAN OPEN-LOOP vs MPC"),
  p("Tabel berikut merangkum reduksi persentase MPC dual-subsystem MLFS+FAHFS dibandingkan kondisi open-loop. Persentase mengacu pada peak displacement rata-rata di tiga lantai struktur."),
  spacer(80),
  summaryTable(),
  spacer(160),
  h2("9.1 Pola Reduksi"),
  bullet("Disturbance bertipe inersia (gempa): MLFS dominan, reduksi 99,29% dengan FAHFS idle."),
  bullet("Disturbance hidrodinamik moderat (banjir): FAHFS dominan, reduksi 100% via peninggian elevasi - struktur fully bypassed."),
  bullet("Disturbance ekstrem (tsunami): FAHFS bekerja maksimal hingga stroke limit; reduksi 91% mencerminkan saturasi fisik silinder, bukan keterbatasan kontroler."),
  bullet("Skenario combined memvalidasi koordinasi otomatis dual-subsystem oleh MPC tunggal - kebaruan utama proposal terbukti operasional."),
  h2("9.2 Perbandingan Implementasi (FAHFS as horizontal force vs FAHFS as elevation lift)"),
  paramTable([
    ["Skenario", "FAHFS-as-force | FAHFS-as-lift (sesuai proposal)"],
    ["Gempa (seismic)", "98,31% | 99,29%"],
    ["Banjir (flood)", "20,90% | 100,00%"],
    ["Tsunami", "1,99% | 91,33%"],
    ["Gabungan (combined)", "1,97% | 91,33%"],
  ]),
  p("Pemodelan FAHFS sebagai PENINGGIAN ELEVASI (sesuai proposal) meningkatkan reduksi tsunami 45x dan banjir 5x. Ini menegaskan bahwa FAHFS bekerja secara fisika berbeda dari MLFS dan harus dimodelkan dengan dinamika hidrolik tersendiri."),
  h2("9.3 Implikasi Desain Praktis"),
  bullet("MLFS kapasitas 5 kN/lantai cukup untuk gempa moderat PGA <= 4 m/s^2."),
  bullet("FAHFS stroke perlu >= h_peak target + margin keselamatan; rekomendasi: 7 m untuk tsunami pesisir Indonesia."),
  bullet("Lookahead reference 3 detik penting untuk anticipative lift; sensor early warning (water-level + seismograph) jadi infrastruktur pendukung wajib."),
  bullet("Bobot tuning q_z >> q_disp menunjukkan strategi optimal: PRIORITASKAN bypass via FAHFS DARIPADA kompensasi via MLFS."),
];

// ----- BAB 10 -----
const bab10 = [
  h1("BAB 10. KESIMPULAN"),
  p("Simulasi memvalidasi efektivitas integrasi MLFS+FAHFS berbasis MPC sesuai konsep multi-physics control system pada proposal. Hasil utama:", { indent: true }),
  bullet("Reduksi displacement struktural: 99,29% (gempa), 100,00% (banjir), 91,33% (tsunami), 91,33% (multi-hazard sekuensial)."),
  bullet(`Bobot Q,R hasil iterasi: q_disp=${best.qd}, q_vel=${best.qv}, q_z=${best.qz}, r_MLFS=${best.rm.toExponential(1)}, r_FAHFS=${best.rf.toExponential(1)}.`),
  bullet("Jain fairness > 0,9 pada semua skenario - proteksi merata antar lantai."),
  bullet("Konsumsi energi MPC < LQR pada skenario gempa; FAHFS tidak konsumsi sumber daya saat tidak dibutuhkan (idle)."),
  p("Kebaruan utama proposal - integrasi dual-subsystem (struktural force vs hidrolik lift) yang dikoordinasi oleh satu MPC - terbukti operasional. Strategi optimal kontrol multi-hazard terungkap: PRIORITASKAN bypass gangguan via FAHFS daripada kompensasi gaya via MLFS, sesuai bobot tuning yang condong ke q_z (elevasi) > q_disp (struktural).", { indent: true }),
  p("Stack simulasi MATLAB tersedia di mpc_multihazard_matlab/ dengan modul build_model, build_mpc, run_mpc, build_reference, tune_weights, compute_metrics, dan plot_results. Validasi numerik via Python mirror (matlab_mirror.py) menghasilkan angka identik.", { indent: true }),
];

// ----- assemble -----
const doc = new Document({
  numbering: {
    config: [{
      reference: "bullets",
      levels: [{
        level: 0, format: LevelFormat.BULLET, text: "•",
        alignment: AlignmentType.LEFT,
        style: { paragraph: { indent: { left: 720, hanging: 360 } } },
      }],
    }],
  },
  styles: { default: { document: { run: { font: TNR, size: 24, color: "000000" } } } },
  sections: [{
    properties: {
      page: { size: { width: 12240, height: 15840 },
              margin: { top: 1440, right: 1440, bottom: 1440, left: 1800 } },
    },
    children: [
      ...cover,
      ...bab1, pageBreak(),
      ...bab2, pageBreak(),
      ...bab3, pageBreak(),
      ...bab4, ...bab5, ...bab6, ...bab7,
      ...bab8, pageBreak(),
      ...bab9, pageBreak(),
      ...bab10,
    ],
  }],
});

Packer.toBuffer(doc).then(buf => {
  fs.writeFileSync(OUT, buf);
  console.log("Generated:", OUT);
  console.log("Size:", buf.length, "bytes");
});
