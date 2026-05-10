%% MPC Multi-Hazard Adaptive Control (augmented with proper FAHFS elevation)
%
% Plant: 3-DOF shear-building structure + 1-DOF FAHFS hydraulic foundation.
% Inputs: 3 MLFS forces (per floor) + 1 FAHFS lift force.
% MLFS handles structural disturbances (gempa); FAHFS lifts the foundation
% above predicted water height to bypass hydrodynamic loads (banjir/tsunami).

clear; close all; clc;
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), 'src')));

%% ---- Parameters ----
params = struct();
params.m    = [333.3; 333.3; 333.3];
params.k    = [1.5e5; 1.5e5; 1.5e5];
params.zeta = 0.02;
params.Ts   = 1e-3;

% FAHFS hydraulic platform
params.fahfs.m_b = 1500;          % foundation effective mass [kg]
params.fahfs.c_b = 1.5e4;          % damping [N s/m]
params.fahfs.k_b = 0;              % no passive spring (active hold)
params.fahfs.z_max = 5.0;          % max stroke [m]
params.fahfs.safety_margin = 0.5;  % lift to h+0.5 m
params.fahfs.lookahead = 3.0;      % anticipate water 3 s ahead [s]

% Geometry / fluid
params.geom.floor_height = 3.0;
params.geom.z = [3.0; 6.0; 9.0];
params.fluid.rho = 1000;
params.fluid.Cd  = 1.2;
params.fluid.A   = 12.0;

% Disturbances
params.seismic.PGA = 3.5;
params.seismic.f0  = 0.5;
params.seismic.f1  = 8.0;
params.flood.h_max  = 2.5;
params.flood.T_rise = 20;
params.flood.v_amp  = 1.5;
params.flood.f      = 0.05;
params.tsunami.h_peak = 4.5;
params.tsunami.v_peak = 6.0;
params.tsunami.tau    = 1.5;
params.tsunami.start  = 12;

% Actuator limits: 3 MLFS (+/-5 kN) + 1 FAHFS (+/-100 kN)
u_lim = [repmat([-5000, 5000], 3, 1); -1e5, 1e5];

T_sec = struct('seismic', 30, 'flood', 60, 'tsunami', 25, 'combined', 40);

sys = build_model(params);
x0  = zeros(sys.nx, 1);

fprintf('Natural freqs (Hz): %s\n', mat2str(sys.f_n', 4));
fprintf('Augmented state dim: %d, inputs: %d\n', sys.nx, sys.nu);

%% ---- Stage 1: tune on COMBINED scenario (covers both regimes) ----
fprintf('\n=== STAGE 1: Tuning weights on combined scenario ===\n');
T_tune = 8;
t_tune = 0:params.Ts:T_tune;
dist_tune = get_disturbances('combined', t_tune, sys, params);
ref_tune  = build_reference('combined', dist_tune, sys, params);
Tsim_tune = numel(t_tune) - 1;
base_tune = run_uncontrolled(sys, dist_tune, Tsim_tune, x0);

tune = tune_weights(sys, params, dist_tune, Tsim_tune, x0, u_lim, ...
                    base_tune, ref_tune);
Q_star = tune.Q;
R_star = tune.R;

%% ---- Stage 2: benchmark all scenarios ----
N_full = 20;
mpc = build_mpc(sys, Q_star, R_star, N_full, u_lim);

scenarios = {'seismic','flood','tsunami','combined'};
results = struct();

outdir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(outdir, 'dir'); mkdir(outdir); end

for s = 1:numel(scenarios)
    sc = scenarios{s};
    fprintf('\n=== STAGE 2: scenario "%s" ===\n', sc);
    t = 0:params.Ts:T_sec.(sc);
    dist = get_disturbances(sc, t, sys, params);
    ref  = build_reference(sc, dist, sys, params);
    Tsim = numel(t) - 1;

    unc = run_uncontrolled(sys, dist, Tsim, x0);
    lqr = run_lqr(sys, Q_star, R_star, dist, Tsim, x0, u_lim, ref);
    mp  = run_mpc(sys, mpc, dist, Tsim, x0, ref);

    m_unc = compute_metrics(unc, sys);
    m_lqr = compute_metrics(lqr, sys, unc, ref);
    m_mpc = compute_metrics(mp,  sys, unc, ref);

    print_metrics(sprintf('[%s] Uncontrolled', sc), m_unc, sys);
    print_metrics(sprintf('[%s] LQR',          sc), m_lqr, sys);
    print_metrics(sprintf('[%s] MPC',          sc), m_mpc, sys);

    results.(sc).runs = struct( ...
        'unc', struct('x', unc.x_log, 'u', unc.u_log, 't', unc.t), ...
        'lqr', struct('x', lqr.x_log, 'u', lqr.u_log, 't', lqr.t), ...
        'mpc', struct('x', mp.x_log,  'u', mp.u_log,  't', mp.t));
    results.(sc).metrics = struct('unc', m_unc, 'lqr', m_lqr, 'mpc', m_mpc);
    results.(sc).ref = ref;
end

save(fullfile(outdir, 'results.mat'), 'results', 'params', 'sys', ...
                                       'Q_star', 'R_star', 'u_lim', 'tune');
fprintf('\nDone. Saved -> %s\n', outdir);
