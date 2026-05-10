function u = mpc_step(x, t, ag, h, v, r)
% MPC_STEP  Single-step receding-horizon MPC for use as Simulink MATLAB
% Function block. Reads global mpc_data from base workspace, maintains a
% persistent warm-start vector across calls.
%
% Inputs:
%   x  current state (8x1)
%   t  current time
%   ag, h, v   current disturbance scalars (sensor measurements)
%   r  reference vector (8x1)

persistent U_warm
persistent mpc_cached
persistent params_cached

if isempty(U_warm)
    mpc_cached = evalin('base', 'mpc_data');
    params_cached = evalin('base', 'params');
    U_warm = zeros(mpc_cached.nu * mpc_cached.N, 1);
end

mpc = mpc_cached;
N = mpc.N; nu = mpc.nu; nd = mpc.nd;
z_now = x(7);

% Build N-step preview using current sensor + frozen z_base assumption.
% Time horizon points: t, t+Ts, ..., t+(N-1)Ts.
ag_pre = ag * ones(1, N);
h_pre  = h  * ones(1, N);
v_pre  = v  * ones(1, N);
Ff_pre = compute_fluid_force(h_pre, v_pre, z_now, params_cached);
Dmat = [ag_pre; Ff_pre];
Dvec = Dmat(:);

Rmat = repmat(r, 1, N);
Rvec = Rmat(:);

diff = mpc.Phi * x + mpc.Gd * Dvec - Rvec;
f = 2 * mpc.Gamma' * mpc.Qbar * diff;

[U, info] = solve_box_qp(mpc.H, f, mpc.u_min, mpc.u_max, U_warm, []);
u = U(1:nu);
U_warm = [U(nu+1:end); zeros(nu, 1)];
end
