function ref = build_reference(scenario, dist, sys, params)
% BUILD_REFERENCE  Construct (nx x N+1) reference trajectory for MPC.
%
% Structural states (q, q_dot) -> 0  (vibration suppression).
% FAHFS elevation z_base       -> max(h(t) + safety_margin, 0)
%   only for hydrodynamic-bearing scenarios; idle (=0) for pure seismic.
% z_base velocity              -> 0 (no oscillation about target).

N = numel(dist.t);
ref = zeros(sys.nx, N);

switch lower(scenario)
    case 'seismic'
        % all zeros
    case {'flood', 'tsunami', 'combined'}
        margin = params.fahfs.safety_margin;
        z_max  = params.fahfs.z_max;
        z_target = max(dist.h + margin, 0);
        % anticipate water arrival: lift early using a sliding lookahead max
        look = round(params.fahfs.lookahead / sys.Ts);
        z_anticipated = movmax(z_target, [0, look]);
        z_anticipated = min(z_anticipated, z_max);
        ref(sys.idx_z, :) = z_anticipated;
end
end
