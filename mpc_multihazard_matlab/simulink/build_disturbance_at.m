function [ag, h_now, v_now] = build_disturbance_at(t)
% BUILD_DISTURBANCE_AT  Sample disturbance at time t for the current
% scenario stored in the base workspace.
persistent dist_cached t_cached params_cached scenario_cached
if isempty(dist_cached)
    dist_cached  = evalin('base', 'dist_data');
    t_cached     = dist_cached.t;
    params_cached = evalin('base', 'params');
    scenario_cached = evalin('base', 'scenario');
end
% Linear interpolation of dist arrays
ag    = interp1(t_cached, dist_cached.ag, t, 'linear', 0);
h_now = interp1(t_cached, dist_cached.h,  t, 'linear', 0);
v_now = interp1(t_cached, dist_cached.v,  t, 'linear', 0);
end
