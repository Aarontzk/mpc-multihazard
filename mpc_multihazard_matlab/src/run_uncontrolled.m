function out = run_uncontrolled(sys, dist, Tsim, x0)
% RUN_UNCONTROLLED  Open-loop response with live fluid recompute against
% (stationary) z_base = 0.  No actuator input.

nx = sys.nx; nu = sys.nu;
x_log = zeros(nx, Tsim+1);
u_log = zeros(nu, Tsim);
x_log(:,1) = x0;
params = dist.params;
ag_pad = [dist.ag, zeros(1,1)];
h_pad  = [dist.h,  dist.h(end)];
v_pad  = [dist.v,  0];
for k = 1:Tsim
    xk = x_log(:,k);
    z_now = xk(sys.idx_z);
    Ff = compute_fluid_force(h_pad(k), v_pad(k), z_now, params);
    dk = [ag_pad(k); Ff];
    x_log(:,k+1) = sys.Ad*xk + sys.Ed*dk;
end
out.x_log = x_log;
out.u_log = u_log;
out.t = (0:Tsim)*sys.Ts;
end
