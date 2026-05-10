function res = tune_weights(sys, p, dist, Tsim, x0, u_lim, baseline, ref, opts)
% TUNE_WEIGHTS  Iterative search over (q_disp, q_vel, q_z, r_mlfs, r_fahfs)
% balancing reduction, fairness, energy, saturation, and elevation tracking.
%
% Score:
%   S = w_red*reduction + w_fair*fairness - w_eng*E_norm - w_sat*sat
%       + w_z*(1 - z_err_norm)
%   where z_err_norm = z_track_rmse / max_h (only for flood/tsunami refs).

if nargin < 9 || isempty(opts); opts = struct(); end
opts = setdef(opts, 'grid_q_disp',  logspace(2, 5, 4));
opts = setdef(opts, 'grid_q_vel',   logspace(0, 3, 3));
opts = setdef(opts, 'grid_q_z',     [1e3, 1e5, 1e7]);
opts = setdef(opts, 'grid_r_mlfs',  [1e-6, 1e-4]);
opts = setdef(opts, 'grid_r_fahfs', [1e-8, 1e-6]);
opts = setdef(opts, 'horizon',      15);
opts = setdef(opts, 'w_red',        0.35);
opts = setdef(opts, 'w_fair',       0.20);
opts = setdef(opts, 'w_eng',        0.10);
opts = setdef(opts, 'w_sat',        0.05);
opts = setdef(opts, 'w_z',          0.30);
opts = setdef(opts, 'verbose',      true);

n = sys.n;
nu = sys.nu;
nx = sys.nx;
E_max = sum((u_lim(:,2)).^2) * Tsim * sys.Ts;
z_ref_max = max(ref(sys.idx_z, 1:Tsim+1));
if z_ref_max < 1e-6; z_ref_max = 1; end

best.score = -inf;
log_rows = {};

for qd_v = opts.grid_q_disp
    for qv_v = opts.grid_q_vel
        for qz_v = opts.grid_q_z
            for rm_v = opts.grid_r_mlfs
                for rf_v = opts.grid_r_fahfs
                    Q_struct = blkdiag(qd_v*eye(n), qv_v*eye(n));
                    Q_found  = diag([qz_v, 1]);
                    Q = blkdiag(Q_struct, Q_found);
                    R = diag([rm_v*ones(1,n), rf_v]);

                    mpc = build_mpc(sys, Q, R, opts.horizon, u_lim);
                    trial = run_mpc(sys, mpc, dist, Tsim, x0, ref);
                    met = compute_metrics(trial, sys, baseline, ref);

                    red    = max(min(met.reduction_avg/100, 1), -1);
                    fair   = met.fairness;
                    e_norm = met.ctrl_energy / E_max;
                    sat    = mean(any(abs(trial.u_log) >= 0.99*u_lim(:,2), 1));
                    if isnan(met.z_track_rmse) || z_ref_max == 0
                        z_quality = 1;
                    else
                        z_quality = max(0, 1 - met.z_track_rmse / z_ref_max);
                    end

                    score = opts.w_red*red + opts.w_fair*fair ...
                          - opts.w_eng*e_norm - opts.w_sat*sat ...
                          + opts.w_z*z_quality;

                    log_rows{end+1} = [qd_v, qv_v, qz_v, rm_v, rf_v, ...
                                       met.reduction_avg, fair, e_norm, sat, ...
                                       z_quality, score]; %#ok<AGROW>

                    if score > best.score
                        best.score = score;
                        best.Q = Q; best.R = R;
                        best.qd = qd_v; best.qv = qv_v; best.qz = qz_v;
                        best.rm = rm_v; best.rf = rf_v;
                        best.metrics = met; best.run = trial;
                    end
                end
            end
        end
    end
end

if opts.verbose
    fprintf(['Best: q_d=%.3g q_v=%.3g q_z=%.3g r_m=%.3g r_f=%.3g\n' ...
             '  reduction=%.2f%%  fair=%.3f  z_qual=%.3f  score=%.4f\n'], ...
        best.qd, best.qv, best.qz, best.rm, best.rf, ...
        best.metrics.reduction_avg, best.metrics.fairness, ...
        max(0, 1 - best.metrics.z_track_rmse / max(z_ref_max,1)), ...
        best.score);
end

res = best;
res.log = vertcat(log_rows{:});
res.log_header = {'q_d','q_v','q_z','r_m','r_f','red(%)','fair', ...
                  'E_norm','sat','z_qual','score'};
end

function s = setdef(s, f, v)
if ~isfield(s, f) || isempty(s.(f)); s.(f) = v; end
end
