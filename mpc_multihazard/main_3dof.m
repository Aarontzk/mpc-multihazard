%% main_3dof.m
% Simulasi MPC multi-hazard — bangunan shear building 3-DOF
% m = 333.3 kg/lantai, eksitasi gempa pada frekuensi natural (resonansi)
%
% Requires: Optimization Toolbox (quadprog)
%           TIDAK perlu Control System Toolbox (ZOH dihitung manual)
%
% Jalankan: >> main_3dof

clear; clc; close all;

%% ── Parameter Struktural ─────────────────────────────────────────────────
m_floor = 333.3;
k1 = 1.8e5;  k2 = 8.0e4;  k3 = 5.0e4;   % kekakuan (N/m)
c1 = 500.0;  c2 = 400.0;  c3 = 300.0;   % redaman  (Ns/m)

M_mat = diag([m_floor, m_floor, m_floor]);

K_mat = [ k1+k2,  -k2,     0;
          -k2,    k2+k3,  -k3;
           0,     -k3,     k3];

C_mat = [ c1+c2,  -c2,     0;
          -c2,    c2+c3,  -c3;
           0,     -c3,     c3];

L_u = eye(3);                                % aktuator tiap lantai
L_d = [-m_floor,  1;                         % gempa (inersia) + banjir (lt.1)
       -m_floor,  0;
       -m_floor,  0];

%% ── Frekuensi Natural ────────────────────────────────────────────────────
Minv    = inv(M_mat);
eigvals = eig(Minv * K_mat);
omega_n = sqrt(sort(eigvals));
freq_hz = omega_n / (2*pi);
f1      = freq_hz(1);    % frekuensi fundamental

fprintf('Natural frequencies : %.3f  %.3f  %.3f Hz\n', freq_hz(1), freq_hz(2), freq_hz(3));
fprintf('Earthquake excitation at f1 = %.3f Hz  (resonance)\n\n', f1);

%% ── Matriks Sistem Kontinu ───────────────────────────────────────────────
n_dof   = 3;
n_state = 2 * n_dof;   % 6
n_in    = 3;
n_dist  = 2;

Z    = zeros(n_dof);
I3   = eye(n_dof);
A_c  = [Z,           I3;
        -Minv*K_mat, -Minv*C_mat];
B_c  = [zeros(n_dof, n_in); Minv * L_u];
E_c  = [zeros(n_dof, n_dist); Minv * L_d];

%% ── Diskritisasi ZOH (tanpa toolbox) ─────────────────────────────────────
Ts      = 0.05;
B_aug_c = [B_c, E_c];                        % augmentasi B dan E

[Ad, B_aug_d] = zoh_discretize(A_c, B_aug_c, Ts);
Bd = B_aug_d(:, 1:n_in);
Ed = B_aug_d(:, n_in+1:end);

%% ── Bobot MPC ────────────────────────────────────────────────────────────
Q     = diag([1e4, 1e4, 1e4, 1e2, 1e2, 1e2]);
R     = diag([1e-6, 1e-6, 1e-6]);
N_hor = 20;

u_max_v = [5000; 5000; 5000];
u_min_v = -u_max_v;

%% ── Bangun Matriks Prediksi MPC (sekali sebelum loop) ────────────────────
[Phi, Gamma, Psi, Q_bar, R_bar] = build_prediction_matrices(...
    Ad, Bd, Ed, Q, R, N_hor);

% Hessian tetap (tidak berubah tiap step)
H_qp = 2 * (Gamma' * Q_bar * Gamma + R_bar);
H_qp = (H_qp + H_qp') / 2;      % pastikan simetris numerik

lb_all = repmat(u_min_v, N_hor, 1);
ub_all = repmat(u_max_v, N_hor, 1);
qp_opts = optimoptions('quadprog', 'Display', 'off', ...
                        'Algorithm', 'interior-point-convex');

%% ── Waktu & Gangguan ─────────────────────────────────────────────────────
T_sim   = 60.0;
t_seq   = (0 : Ts : T_sim - Ts)';
T_steps = length(t_seq);

eq    = earthquake_disturbance(t_seq, 3.0, 5.0, 15.0, f1);
flood = flood_disturbance(t_seq, 2000.0, 35.0, 10.0, 0.05);
d_seq = [eq, flood];     % (T x 2)

x_ref = zeros(T_steps, n_state);
x0    = zeros(n_state, 1);

%% ── Simulasi MPC ─────────────────────────────────────────────────────────
fprintf('Running MPC simulation — 3-DOF, %d steps, N=%d...\n', T_steps, N_hor);

x_hist = zeros(T_steps+1, n_state);
u_hist = zeros(T_steps,   n_in);
x_hist(1,:) = x0';

for k = 1:T_steps
    % Window gangguan & referensi
    k_end = min(k + N_hor - 1, T_steps);
    d_win = d_seq(k:k_end, :);
    r_win = x_ref(k:k_end, :);

    pad = N_hor - size(d_win, 1);
    if pad > 0
        d_win = [d_win; zeros(pad, n_dist)];
        r_win = [r_win; repmat(x_ref(end,:), pad, 1)];
    end

    D_vec = reshape(d_win', N_hor * n_dist, 1);
    R_vec = reshape(r_win', N_hor * n_state, 1);

    % Gradient (berubah tiap step)
    bias  = Phi * x_hist(k,:)' + Psi * D_vec - R_vec;
    f_qp  = 2 * Gamma' * Q_bar * bias;

    % Solve QP
    U_opt = quadprog(H_qp, f_qp, [], [], [], [], lb_all, ub_all, [], qp_opts);
    if isempty(U_opt)
        U_opt = zeros(N_hor * n_in, 1);
    end

    u_opt = U_opt(1:n_in);
    u_hist(k,:) = u_opt';

    % Propagasi state
    x_hist(k+1,:) = (Ad * x_hist(k,:)' + Bd * u_opt + Ed * d_seq(k,:)')';
end

%% ── Baseline Uncontrolled ────────────────────────────────────────────────
fprintf('Running uncontrolled simulation...\n');
x_unc      = zeros(T_steps+1, n_state);
x_unc(1,:) = x0';
for k = 1:T_steps
    x_unc(k+1,:) = (Ad * x_unc(k,:)' + Ed * d_seq(k,:)')';
end

%% ── Metrik ───────────────────────────────────────────────────────────────
roof_col = 3;   % indeks DOF atap (lantai 3, 1-indexed)

peak_mpc  = max(abs(x_hist(:, roof_col))) * 1e3;
peak_unc  = max(abs(x_unc(:,  roof_col))) * 1e3;
rms_mpc   = rms(x_hist(:, roof_col)) * 1e3;
rms_unc   = rms(x_unc(:,  roof_col)) * 1e3;
pdr       = (peak_unc - peak_mpc) / peak_unc * 100;
vrr       = rms_mpc / rms_unc;
ctrl_peak = max(vecnorm(u_hist, 2, 2));

fprintf('\n========================================\n');
fprintf('  EVALUASI KINERJA MPC  [Proposal 3.8]\n');
fprintf('========================================\n');
fprintf('[3.8.1] Peak displacement MPC   : %8.3f mm\n', peak_mpc);
fprintf('        Peak displacement Unctrl: %8.3f mm\n', peak_unc);
fprintf('        Peak Reduction          : %8.1f %%\n', pdr);
fprintf('        RMS  Reduction          : %8.1f %%\n', (rms_unc-rms_mpc)/rms_unc*100);
fprintf('        VRR (MLFS)              : %8.3f\n',   vrr);
fprintf('[3.8.3] Control peak ||u||      : %8.1f N\n',  ctrl_peak);
fprintf('========================================\n');

%% ── Plot ─────────────────────────────────────────────────────────────────
t_full = (0:T_steps)' * Ts;
colors = lines(3);

figure('Position', [80 80 1100 750]);

% Displacement
subplot(3,1,1); hold on; grid on;
for i = 1:3
    plot(t_full, x_hist(:,i)*1e3, 'Color', colors(i,:), ...
         'DisplayName', sprintf('Floor %d (MPC)', i));
end
plot(t_full, x_unc(:,3)*1e3, 'r--', 'LineWidth', 1.2, ...
     'DisplayName', 'Floor 3 (Uncontrolled)');
ylabel('Displacement (mm)');
title(sprintf('3-DOF MPC  |  Earthquake %.3f Hz (resonance)  +  Flood', f1));
legend('Location','best','FontSize',8);

% Control
subplot(3,1,2); hold on; grid on;
for i = 1:3
    plot(t_seq, u_hist(:,i), 'Color', colors(i,:), ...
         'DisplayName', sprintf('u%d (Floor %d)', i, i));
end
ylabel('Control Force (N)');
legend('Location','best','FontSize',8);

% Disturbance
subplot(3,1,3); hold on; grid on;
plot(t_seq, eq,    'DisplayName', sprintf('Earthquake (m/s^2) @ %.3f Hz', f1));
plot(t_seq, flood, '--', 'DisplayName', 'Flood force (N)');
ylabel('Disturbance');
xlabel('Time (s)');
legend('Location','best');

saveas(gcf, 'results_3dof_matlab.png');
fprintf('Plot saved -> results_3dof_matlab.png\n');


%% ════════════════════════════════════════════════════════════════════════
%%  LOCAL FUNCTIONS
%% ════════════════════════════════════════════════════════════════════════

function [Ad, Bd_out] = zoh_discretize(Ac, Bc, Ts)
% ZOH discretization via matrix exponential — tidak butuh toolbox.
% expm([Ac, Bc; 0, 0] * Ts) = [Ad, Bd_integral; 0, I]
    n    = size(Ac, 1);
    m    = size(Bc, 2);
    M_aug = expm([Ac, Bc; zeros(m, n+m)] * Ts);
    Ad    = M_aug(1:n, 1:n);
    Bd_out = M_aug(1:n, n+1:end);
end


function [Phi, Gamma, Psi, Q_bar, R_bar] = build_prediction_matrices(...
    Ad, Bd, Ed, Q, R, N)
% Bangun matriks prediksi condensed MPC (hanya sekali sebelum loop).
%
%   X = Phi*x0 + Gamma*U + Psi*D
%   J = (X-R)'*Q_bar*(X-R) + U'*R_bar*U
%
    n  = size(Ad,1);
    m  = size(Bd,2);
    nd = size(Ed,2);

    % Precompute Ad^k untuk k=0..N
    Ad_pow = cell(N+1, 1);
    Ad_pow{1} = eye(n);
    for i = 1:N
        Ad_pow{i+1} = Ad_pow{i} * Ad;
    end

    Phi   = zeros(N*n,  n);
    Gamma = zeros(N*n,  N*m);
    Psi   = zeros(N*n,  N*nd);

    for i = 1:N
        ix = (i-1)*n+1 : i*n;
        Phi(ix, :) = Ad_pow{i+1};   % Ad^i

        for j = 1:i
            iu     = (j-1)*m  + 1 : j*m;
            id     = (j-1)*nd + 1 : j*nd;
            Ap_ij  = Ad_pow{i-j+1};   % Ad^(i-j)
            Gamma(ix, iu) = Ap_ij * Bd;
            Psi(ix, id)   = Ap_ij * Ed;
        end
    end

    Q_bar = kron(eye(N), Q);
    R_bar = kron(eye(N), R);
end


function d = earthquake_disturbance(t, magnitude, t_start, duration, freq_hz)
% Damped sine — percepatan tanah (m/s^2)
    omega = 2*pi*freq_hz;
    zeta  = 0.3;
    tau   = max(t - t_start, 0);
    mask  = (t >= t_start) & (t <= t_start + duration);
    d     = mask .* magnitude .* exp(-zeta*omega*tau) .* sin(omega*tau);
end


function d = flood_disturbance(t, peak, t_peak, t_rise, decay)
% Ramp naik + eksponensial turun — gaya fluida horizontal (N)
    t_start   = t_peak - t_rise;
    d         = zeros(size(t));
    rise_mask = (t >= t_start) & (t < t_peak);
    dec_mask  = t >= t_peak;
    d(rise_mask) = peak * (t(rise_mask) - t_start) / t_rise;
    d(dec_mask)  = peak * exp(-decay * (t(dec_mask) - t_peak));
end
