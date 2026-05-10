function dist = get_disturbances(scenario, t, sys, params)
% GET_DISTURBANCES  Build disturbance trajectory + return raw water profile
% so that the closed-loop simulator can recompute fluid forces against the
% live z_base during MPC execution.
%
% Returns dist struct with fields:
%   ag (1xN)  ground acceleration
%   h  (1xN)  water surface height above ground
%   v  (1xN)  flow velocity
%   nd       4 (a_g + 3 horizontal fluid forces)
%
% Use compute_fluid_force(h, v, z_base, params) to obtain per-floor force.

n = sys.n;
N = numel(t);
ag = zeros(1, N);
h  = zeros(1, N);
v  = zeros(1, N);

switch lower(scenario)
    case 'seismic'
        ag = seismic_signal(t, params);
    case 'flood'
        [h, v] = flood_profile(t, params);
    case 'tsunami'
        [h, v] = tsunami_profile(t, params);
    case 'combined'
        ag = seismic_signal(t, params);
        t_off = params.tsunami.start;
        mask = t >= t_off;
        if any(mask)
            t_loc = t(mask) - t_off;
            [h_loc, v_loc] = tsunami_profile(t_loc, params);
            h(mask) = h_loc;
            v(mask) = v_loc;
        end
    otherwise
        error('Unknown scenario: %s', scenario);
end

dist.t = t;
dist.ag = ag;
dist.h  = h;
dist.v  = v;
dist.n  = n;
dist.params = params;
dist.nd = 1 + n;
end

% -------------------------------------------------------------- helpers %
function ag = seismic_signal(t, p)
PGA = p.seismic.PGA;
f0  = p.seismic.f0;
f1  = p.seismic.f1;
T   = t(end);
phi = 2*pi*(f0*t + (f1-f0)/(2*T)*t.^2);
env = exp(-((t - 0.35*T)/(0.18*T)).^2);
ag  = PGA*env.*sin(phi) + 0.25*PGA*env.*sin(2*pi*15*t + 1.3);
end

function [h, v] = flood_profile(t, p)
h = p.flood.h_max * (1 - exp(-t/p.flood.T_rise));
v = p.flood.v_amp * sin(2*pi*p.flood.f * t);
end

function [h, v] = tsunami_profile(t, p)
tau = p.tsunami.tau;
front = sech((t - 3*tau)/tau).^2;
h = p.tsunami.h_peak * (front + 0.4*(1 - exp(-t/(2*tau))));
v = p.tsunami.v_peak * sech((t - 3*tau)/tau);
end
