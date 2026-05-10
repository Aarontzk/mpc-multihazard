function sys = build_model(params)
% BUILD_MODEL  Augmented multi-physics plant: 3-DOF structure + 1-DOF FAHFS.
%
% State (8x1):
%   x = [q1 q2 q3 q1' q2' q3' z_base z_base']
% Input (4x1):
%   u = [u_MLFS_1 u_MLFS_2 u_MLFS_3 u_FAHFS]
% Disturbance (1+n=4):
%   d = [a_g; F_f1; F_f2; F_f3]
%
% Structural sub-system (horizontal):
%   M q'' + C q' + K q = -M*1*a_g + L_u u_MLFS + F_fluid_horiz(q, z_base, h, v)
% Foundation sub-system (vertical, FAHFS hydraulic platform):
%   m_b z'' + c_b z' + k_b z = u_FAHFS
%
% The fluid force on each floor depends on z_base (nonlinear coupling). The
% MPC treats fluid force as a parameter-dependent disturbance refreshed each
% step in run_mpc.

m = params.m(:);  k = params.k(:);
n = numel(m);

M = diag(m);
K = zeros(n);
for i = 1:n
    if i < n
        K(i,i)   = k(i) + k(i+1);
        K(i,i+1) = -k(i+1);
        K(i+1,i) = -k(i+1);
    else
        K(i,i)   = k(i);
    end
end
K(1,1) = k(1) + k(2);

% Rayleigh damping on structure
omega = sqrt(eig(K, M)); omega = sort(omega);
w1 = omega(1); w2 = omega(min(2,end));
zeta = params.zeta;
alpha = 2*zeta*w1*w2/(w1+w2);
beta_d = 2*zeta/(w1+w2);
Cdamp = alpha*M + beta_d*K;

% Foundation parameters
mb = params.fahfs.m_b;
cb = params.fahfs.c_b;
kb = params.fahfs.k_b;

% --- assemble continuous augmented state-space ---
nx_struct = 2*n;            % 6
nx = nx_struct + 2;         % +2 for [z_base; z_base_dot]
nu_mlfs = n;                % 3 MLFS forces
nu = nu_mlfs + 1;           % 4 (3 MLFS + 1 FAHFS)
nd = 1 + n;                 % 4 (a_g + per-floor fluid)

Z3 = zeros(n);  I3 = eye(n);

A_struct = [Z3, I3; -M\K, -M\Cdamp];                    % (6x6)
A_found  = [0 1; -kb/mb, -cb/mb];                       % (2x2)
Ac = blkdiag(A_struct, A_found);                        % (8x8)

% Inputs:
% MLFS forces enter structural acceleration only.
B_struct_u = [Z3; M\I3];                                % (6x3)
B_struct_F = zeros(nx_struct, 1);                       % MLFS does not act on foundation
B_found_u  = zeros(2, nu_mlfs);                         % FAHFS does not enter struct
B_found_F  = [0; 1/mb];                                 % FAHFS lifts foundation
Bc = [B_struct_u, B_struct_F;
      B_found_u,  B_found_F];                            % (8x4)

% Disturbances:
%   col 1 = a_g (ground accel) acts on EVERY structural mass
%   cols 2..n+1 = per-floor horizontal fluid force, divided by m_i
ones_v = ones(n,1);
E_struct_eq = [zeros(n,1); -ones_v];                    % (6x1)
E_struct_F  = [Z3; M\I3];                               % (6x3)
E_found     = zeros(2, nd);                             % foundation indep of horiz dist
Ec = [E_struct_eq, E_struct_F;
      E_found];                                          % (8x4)

Cc = eye(nx);

% Discretize (ZOH) for u and d separately
Ts = params.Ts;
Ad = expm(Ac*Ts);
Mu = expm([Ac, Bc; zeros(nu, nx+nu)]*Ts);
Bd = Mu(1:nx, nx+1:end);
Md = expm([Ac, Ec; zeros(nd, nx+nd)]*Ts);
Ed = Md(1:nx, nx+1:end);

% Pack
sys.M = M;  sys.K = K;  sys.C = Cdamp;
sys.mb = mb; sys.cb = cb; sys.kb = kb;
sys.Ac = Ac; sys.Bc = Bc; sys.Ec = Ec; sys.Cc = Cc;
sys.Ad = Ad; sys.Bd = Bd; sys.Ed = Ed; sys.Cd = Cc;
sys.n = n;
sys.nx = nx;          % 8
sys.nu = nu;          % 4
sys.nd = nd;          % 4
sys.nx_struct = nx_struct;
sys.idx_q  = 1:n;
sys.idx_qd = n+1:2*n;
sys.idx_z  = 2*n + 1;
sys.idx_zd = 2*n + 2;
sys.idx_u_mlfs = 1:n;
sys.idx_u_fahfs = n + 1;
sys.Ts = Ts;
sys.omega = omega;
sys.f_n = omega/(2*pi);
end
