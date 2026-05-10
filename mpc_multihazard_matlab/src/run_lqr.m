function out = run_lqr(sys, Q, R, dist, Tsim, x0, u_lim, ref_traj)
% RUN_LQR  Discrete LQR with feed-forward reference tracking and live
% fluid recomputation.

if exist('dlqr', 'file') == 2
    [Klqr, ~, ~] = dlqr(sys.Ad, sys.Bd, Q, R);
else
    Klqr = dlqr_iter(sys.Ad, sys.Bd, Q, R);
end

nx = sys.nx; nu = sys.nu;
x_log = zeros(nx, Tsim+1);
u_log = zeros(nu, Tsim);
x_log(:,1) = x0;
params = dist.params;

ag_pad = [dist.ag, zeros(1, 1)];
h_pad  = [dist.h,  dist.h(end)];
v_pad  = [dist.v,  0];

for k = 1:Tsim
    xk = x_log(:,k);
    z_now = xk(sys.idx_z);
    err = xk - ref_traj(:,k);
    uk = -Klqr * err;
    uk = min(max(uk, u_lim(:,1)), u_lim(:,2));
    Ff = compute_fluid_force(h_pad(k), v_pad(k), z_now, params);
    dk = [ag_pad(k); Ff];
    x_log(:,k+1) = sys.Ad*xk + sys.Bd*uk + sys.Ed*dk;
    u_log(:,k) = uk;
end

out.x_log = x_log;
out.u_log = u_log;
out.t = (0:Tsim)*sys.Ts;
out.K = Klqr;
end

function K = dlqr_iter(A, B, Q, R)
P = Q;
for it = 1:5000
    M = R + B'*P*B;
    Pn = A'*P*A - (A'*P*B) * (M \ (B'*P*A)) + Q;
    if norm(Pn - P, 'fro') < 1e-9 * max(1, norm(P,'fro')); P = Pn; break; end
    P = Pn;
end
K = (R + B'*P*B) \ (B'*P*A);
end
