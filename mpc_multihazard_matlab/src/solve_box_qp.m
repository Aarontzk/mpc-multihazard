function [U, info] = solve_box_qp(H, f, lb, ub, U0, ctx)
% SOLVE_BOX_QP  Box-constrained QP:  min 0.5 U' H U + f' U  s.t. lb <= U <= ub
%
% Tries quadprog (Optimization Toolbox); falls back to projected accelerated
% gradient (FISTA) — no toolbox required. The chosen backend is cached
% across calls in `ctx` to avoid repeated capability probes.
%
% Inputs:
%   H, f      QP cost (H must be symmetric PD).
%   lb, ub    Box bounds (column vectors, same length as U).
%   U0        Warm start.
%   ctx       struct with persistent fields (.backend, .L, .opts).
%
% Outputs:
%   U         Solution.
%   info      struct: .iters, .backend, .ctx (updated cache).

if nargin < 6 || isempty(ctx); ctx = struct(); end

% Backend selection (one-shot)
if ~isfield(ctx, 'backend') || isempty(ctx.backend)
    if exist('quadprog', 'file') == 2 && exist('optimoptions', 'file') == 2
        try
            ctx.opts = optimoptions('quadprog', 'Display','off', ...
                'Algorithm','interior-point-convex', ...
                'OptimalityTolerance',1e-7, 'StepTolerance',1e-9);
            ctx.backend = 'quadprog';
        catch
            ctx.backend = 'fista';
        end
    elseif exist('quadprog', 'file') == 2
        % Old MATLAB / Octave optim package: use legacy options struct
        ctx.opts = struct('Display','off');
        ctx.backend = 'quadprog_legacy';
    else
        ctx.backend = 'fista';
    end
end

switch ctx.backend
    case 'quadprog'
        [U, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, U0, ctx.opts);
        if exitflag <= 0 || isempty(U)
            [U, sub] = fista_box(H, f, lb, ub, U0, ctx);
            ctx = sub.ctx;
            info.iters = sub.iters;
            info.backend = 'fista (fallback)';
        else
            info.iters = NaN;
            info.backend = 'quadprog';
        end

    case 'quadprog_legacy'
        [U, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, U0, ctx.opts);
        if exitflag <= 0 || isempty(U)
            [U, sub] = fista_box(H, f, lb, ub, U0, ctx);
            ctx = sub.ctx;
            info.iters = sub.iters;
            info.backend = 'fista (fallback)';
        else
            info.iters = NaN;
            info.backend = 'quadprog_legacy';
        end

    case 'fista'
        [U, sub] = fista_box(H, f, lb, ub, U0, ctx);
        ctx = sub.ctx;
        info.iters = sub.iters;
        info.backend = 'fista';
end

info.ctx = ctx;
end

% --------------------------------------------------------------------- %
function [U, sub] = fista_box(H, f, lb, ub, U0, ctx)
% Projected accelerated gradient (FISTA) for strongly convex box-QP.
%   - Lipschitz constant L = lambda_max(H).
%   - Backtracking disabled for speed; we cache L on first call.
% Convergence rate O(1/k^2); typically <= 200 iters at 1e-7 KKT residual.

if ~isfield(ctx, 'L') || isempty(ctx.L)
    % Power iteration for spectral norm (avoids full eig on big H)
    n = size(H,1);
    v = randn(n,1); v = v/norm(v);
    L_est = 0;
    for it = 1:50
        v_next = H*v;
        L_new  = norm(v_next);
        if abs(L_new - L_est) < 1e-6*L_new; break; end
        v = v_next / max(L_new, eps);
        L_est = L_new;
    end
    ctx.L = max(L_est, 1) * 1.05;  % small safety margin
end

L  = ctx.L;
U  = max(min(U0, ub), lb);
Y  = U;
t  = 1;
max_iter = 400;
tol = 1e-7;

for k = 1:max_iter
    U_prev = U;
    grad   = H*Y + f;
    U      = max(min(Y - grad/L, ub), lb);
    t_new  = (1 + sqrt(1 + 4*t*t))/2;
    Y      = U + ((t - 1)/t_new) * (U - U_prev);
    t      = t_new;

    % KKT-style stopping: projected gradient norm
    res = norm(U - max(min(U - (H*U + f)/L, ub), lb));
    if res < tol*max(1, norm(U)); break; end
end

sub.iters = k;
sub.ctx = ctx;
end
