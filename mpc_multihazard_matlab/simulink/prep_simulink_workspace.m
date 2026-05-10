function prep_simulink_workspace(mlfs_mode, scenario)
% PREP_SIMULINK_WORKSPACE  Populate base workspace with Ad/Bd/Ed/Q/R/etc.
% so that the Simulink model can pick them up.
%
% Usage:
%   prep_simulink_workspace                       % per_floor + seismic
%   prep_simulink_workspace('per_floor','flood')
%   prep_simulink_workspace('base_only','combined')
%
% After this:
%   1. open_system('mpc_multihazard')
%   2. Click Run (or evaluate `sim('mpc_multihazard')`)
%   3. Inspect x_log, u_log in workspace.

if nargin < 1; mlfs_mode = 'per_floor'; end
if nargin < 2; scenario  = 'seismic'; end

addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src')));

% ---- Parameters (mirror of main.m) ----
params = struct();
params.m    = [333.3; 333.3; 333.3];
params.k    = [1.5e5; 1.5e5; 1.5e5];
params.zeta = 0.02;
params.Ts   = 1e-3;
params.fahfs.m_b = 1500;
params.fahfs.c_b = 1.5e4;
params.fahfs.k_b = 0;
params.fahfs.z_max = 5.0;
params.fahfs.safety_margin = 0.5;
params.fahfs.lookahead = 3.0;
params.geom.floor_height = 3.0;
params.geom.z = [3.0; 6.0; 9.0];
params.fluid.rho = 1000; params.fluid.Cd = 1.2; params.fluid.A = 12.0;
params.seismic.PGA = 3.5; params.seismic.f0 = 0.5; params.seismic.f1 = 8.0;
params.flood.h_max = 2.5; params.flood.T_rise = 20;
params.flood.v_amp = 1.5; params.flood.f = 0.05;
params.tsunami.h_peak = 4.5; params.tsunami.v_peak = 6.0;
params.tsunami.tau = 1.5; params.tsunami.start = 12.0;

T_sec = struct('seismic', 30, 'flood', 60, 'tsunami', 25, 'combined', 40);

% ---- Build plant ----
sys = build_model_dispatch(params, mlfs_mode);

% ---- Build disturbance + reference for chosen scenario ----
T_sim = T_sec.(scenario);
t_full = 0:params.Ts:T_sim;
dist = get_disturbances(scenario, t_full, sys, params);
ref  = build_reference(scenario, dist, sys, params);

% ---- Tuned weights (hardcoded from prior tuning) ----
n = sys.n; nu_mlfs = sys.nu - 1;
qd = 100; qv = 1000; qz = 1e5; rm = 1e-6; rf = 1e-8;
Q_struct = blkdiag(qd*eye(n), qv*eye(n));
Q_found  = diag([qz, 1]);
Q = blkdiag(Q_struct, Q_found);
R = diag([rm*ones(1, nu_mlfs), rf]);

% Actuator limits
if strcmp(mlfs_mode, 'per_floor')
    u_lim = [repmat([-5000, 5000], 3, 1); -1e5, 1e5];
else
    u_lim = [-15000, 15000; -1e5, 1e5];
end

% ---- Push to base workspace ----
assignin('base', 'params',    params);
assignin('base', 'sys',       sys);
assignin('base', 'Ad',        sys.Ad);
assignin('base', 'Bd',        sys.Bd);
assignin('base', 'Ed',        sys.Ed);
assignin('base', 'Q',         Q);
assignin('base', 'R',         R);
assignin('base', 'Ts',        params.Ts);
assignin('base', 'T_sim',     T_sim);
assignin('base', 'NU',        sys.nu);
assignin('base', 'ND',        sys.nd);
assignin('base', 'NX',        sys.nx);
assignin('base', 'u_lim',     u_lim);
assignin('base', 'dist_data', dist);
assignin('base', 'ref_data',  ref);
assignin('base', 'mlfs_mode', mlfs_mode);
assignin('base', 'scenario',  scenario);

% Build prebuilt MPC matrices and stash for mpc_step()
N = 20;
mpc = build_mpc(sys, Q, R, N, u_lim);
assignin('base', 'mpc_data',  mpc);

fprintf('Workspace prepared. mode=%s scenario=%s T_sim=%g s\n', ...
    mlfs_mode, scenario, T_sim);
fprintf('  nx=%d nu=%d nd=%d  Ad,Bd,Ed,Q,R,u_lim,dist_data,ref_data,mpc_data set.\n', ...
    sys.nx, sys.nu, sys.nd);
fprintf('Now run:  sim(''mpc_multihazard'');\n');
end

function sys = build_model_dispatch(params, mode)
% Wrapper because build_model.m currently always per_floor; if you also
% added an mlfs_mode param to MATLAB build_model.m, replace this with a
% direct call.  This local stub re-implements the base_only B matrix.
sys = build_model(params);
if strcmp(mode, 'base_only')
    n = sys.n;
    M = sys.M;
    L_u = zeros(n, 1); L_u(1) = 1;
    Z3 = zeros(n);
    B_struct_u = [Z3(:,1); M\L_u];   % 6x1
    Bc = [B_struct_u, zeros(2*n, 1);
          zeros(2, 1), [0; 1/sys.mb]];     % 8x2
    Ts = params.Ts;
    nx = sys.nx; nu = 2;
    Mu = expm([sys.Ac, Bc; zeros(nu, nx+nu)] * Ts);
    sys.Bd = Mu(1:nx, nx+1:end);
    sys.nu = nu;
    sys.nu_mlfs = 1;
    sys.idx_u_mlfs = 1;
    sys.idx_u_fahfs = 2;
end
end
