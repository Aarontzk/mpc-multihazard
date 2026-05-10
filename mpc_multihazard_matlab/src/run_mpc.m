function out = run_mpc(sys, mpc, dist, Tsim, x0, ref_traj)
% RUN_MPC  Closed-loop receding-horizon MPC with reference tracking and
% live recomputation of fluid disturbance vs current FAHFS elevation.
%
% Inputs:
%   sys, mpc                 from build_model / build_mpc
%   dist                     struct with .ag, .h, .v (length T+1+N samples)
%   Tsim                     number of simulation steps
%   x0                       initial state (nx x 1)
%   ref_traj                 (nx x T+1+N) reference trajectory; the FAHFS
%                            target z_base is set to max(h(t)+margin, 0)
%                            inside main; pass zeros for non-tracking.

nx = sys.nx; nu = sys.nu; nd = sys.nd; N = mpc.N;
Ts = sys.Ts;
n  = sys.n;
params = dist.params;

x_log = zeros(nx, Tsim+1);
u_log = zeros(nu, Tsim);
J_log = zeros(1,  Tsim);
x_log(:,1) = x0;

% Pad horizon-length tails so previews never overrun
ag_pad = [dist.ag, zeros(1, N)];
h_pad  = [dist.h,  repmat(dist.h(end),  1, N)];
v_pad  = [dist.v,  zeros(1, N)];
ref_pad = [ref_traj, repmat(ref_traj(:,end), 1, N)];

U_warm = zeros(nu*N, 1);

for k = 1:Tsim
    xk = x_log(:,k);
    z_now = xk(sys.idx_z);

    % Build N-step disturbance preview using the CURRENT z_base value.
    h_pre = h_pad(k:k+N-1);
    v_pre = v_pad(k:k+N-1);
    ag_pre = ag_pad(k:k+N-1);
    Ff_pre = compute_fluid_force(h_pre, v_pre, z_now, params);

    % Stack d_k = [ag; Ff_1; Ff_2; Ff_3]   (nd x N)
    Dmat = [ag_pre; Ff_pre];
    Dvec = Dmat(:);                          % column-major flatten matches build_mpc

    % Reference vector for horizon
    Rmat = ref_pad(:, k:k+N-1);
    Rvec = Rmat(:);

    % Linear cost gradient: f = 2 Gamma' Qbar (Phi*x0 + Gd*D - R_vec)
    diff = mpc.Phi * xk + mpc.Gd * Dvec - Rvec;
    f = 2 * mpc.Gamma' * mpc.Qbar * diff;

    [U, ~] = solve_box_qp(mpc.H, f, mpc.u_min, mpc.u_max, U_warm, []);
    uk = U(1:nu);

    % Apply with the actual disturbance at this step (using true current z_base)
    Ff_true = compute_fluid_force(h_pad(k), v_pad(k), z_now, params);
    dk = [ag_pad(k); Ff_true];
    x_log(:, k+1) = sys.Ad * xk + sys.Bd * uk + sys.Ed * dk;
    u_log(:, k) = uk;
    err = x_log(:, k+1) - ref_pad(:, k+1);
    J_log(k) = err' * mpc.Q * err + uk' * mpc.R * uk;

    U_warm = [U(nu+1:end); zeros(nu,1)];
end

out.x_log = x_log;
out.u_log = u_log;
out.J_log = J_log;
out.t = (0:Tsim)*Ts;
end
