function Ff = compute_fluid_force(h, v, z_base, params)
% COMPUTE_FLUID_FORCE  Per-floor horizontal hydrodynamic load given current
% water surface height h, flow velocity v, and FAHFS elevation z_base.
%
% Returns Ff (n x N) where N = length(h).
% Per-floor submerged depth d_i = clip(h - z_base - (i-1)*floor_h, 0, floor_h).

floor_h = params.geom.floor_height;
n = numel(params.geom.z);
N = numel(h);
rho = params.fluid.rho;
Cd  = params.fluid.Cd;
A   = params.fluid.A;
g   = 9.81;
w   = A / floor_h;     % facade strip width

if numel(z_base) == 1
    z_base = z_base * ones(1, N);
end
h = h(:).';
v = v(:).';
z_base = z_base(:).';

Ff = zeros(n, N);
for i = 1:n
    z_bot = (i-1) * floor_h;
    d_i = max(min(h - z_base - z_bot, floor_h), 0);
    drag  = 0.5 * rho * Cd * (w * d_i) .* v.^2 .* sign(v);
    hydro = 0.5 * rho * g  * (d_i.^2) * w;
    Ff(i, :) = drag + hydro;
end
end
