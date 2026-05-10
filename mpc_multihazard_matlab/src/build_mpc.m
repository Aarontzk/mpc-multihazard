function mpc = build_mpc(sys, Q, R, N, u_lim, x_lim)
% BUILD_MPC  Dense-form QP MPC matrices with reference tracking.
%
%   J = sum_{k=1}^N (x_k - r_k)' Q (x_k - r_k) + sum_{k=0}^{N-1} u_k' R u_k
%
% Inputs:
%   sys   discrete system from build_model
%   Q,R   weights
%   N     prediction horizon
%   u_lim (nu x 2) bounds [u_min, u_max]
%   x_lim (optional, nx x 2) state bounds; rows with NaN ignored
%
% Returns mpc struct used by run_mpc.

if nargin < 6 || isempty(x_lim); x_lim = nan(sys.nx, 2); end

Ad = sys.Ad; Bd = sys.Bd; Ed = sys.Ed;
nx = sys.nx; nu = sys.nu; nd = sys.nd;

Phi = zeros(nx*N, nx);
Gamma = zeros(nx*N, nu*N);
Gd = zeros(nx*N, nd*N);

A_pow = {eye(nx)};
for i = 1:N
    A_pow{end+1} = Ad * A_pow{end}; %#ok<AGROW>
end

for i = 1:N
    Phi((i-1)*nx + (1:nx), :) = A_pow{i+1};
    for j = 1:i
        rows = (i-1)*nx + (1:nx);
        cu = (j-1)*nu + (1:nu);
        cd = (j-1)*nd + (1:nd);
        Gamma(rows, cu) = A_pow{i-j+1} * Bd;
        Gd(rows, cd)    = A_pow{i-j+1} * Ed;
    end
end

Qbar = kron(eye(N), Q);
Rbar = kron(eye(N), R);
H = 2*(Gamma' * Qbar * Gamma + Rbar);
H = 0.5*(H + H');

u_min = repmat(u_lim(:,1), N, 1);
u_max = repmat(u_lim(:,2), N, 1);

% Optional state inequality constraints: x_min <= x_k <= x_max for selected indices
mask = ~isnan(x_lim(:,1)) & ~isnan(x_lim(:,2));
if any(mask)
    sel = find(mask);
    Cx = zeros(numel(sel)*N, nx*N);
    for i = 1:N
        Cx((i-1)*numel(sel) + (1:numel(sel)), (i-1)*nx + sel) = eye(numel(sel));
    end
else
    Cx = zeros(0, nx*N);
    sel = [];
end

mpc.Phi = Phi;
mpc.Gamma = Gamma;
mpc.Gd = Gd;
mpc.Qbar = Qbar;
mpc.Rbar = Rbar;
mpc.H = H;
mpc.u_min = u_min;
mpc.u_max = u_max;
mpc.N = N;
mpc.nu = nu;
mpc.nx = nx;
mpc.nd = nd;
mpc.Q = Q;
mpc.R = R;
mpc.Cx = Cx;
mpc.x_sel = sel;
mpc.x_lim = x_lim;
end
