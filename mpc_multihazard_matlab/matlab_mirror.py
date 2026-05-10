"""
Python mirror of MATLAB code (augmented FAHFS variant).

State (8): [q1 q2 q3 q1' q2' q3' z_base z_base']
Input (4): [u_MLFS_1 u_MLFS_2 u_MLFS_3 u_FAHFS]
Disturbance (4): [a_g; F_f1; F_f2; F_f3] - fluid recomputed online vs z_base.
"""
import json
from pathlib import Path
import numpy as np
from scipy.linalg import expm, solve_discrete_are


# ---------------------------------------------------------- plant
def build_model(p, mlfs_mode="per_floor"):
    """mlfs_mode: 'per_floor' -> 3 MLFS forces (one per story);
                  'base_only' -> 1 MLFS force at floor 1 (active base isolation)."""
    m = np.asarray(p["m"], float); k = np.asarray(p["k"], float)
    n = m.size
    M = np.diag(m); K = np.zeros((n, n))
    for i in range(n):
        if i < n - 1:
            K[i, i] = k[i] + k[i + 1]
            K[i, i + 1] = -k[i + 1]; K[i + 1, i] = -k[i + 1]
        else:
            K[i, i] = k[i]
    K[0, 0] = k[0] + k[1]
    omega = np.sqrt(np.sort(np.linalg.eigvalsh(np.linalg.solve(M, K))))
    w1, w2 = omega[0], omega[1]
    zeta = p["zeta"]
    a = 2 * zeta * w1 * w2 / (w1 + w2); b = 2 * zeta / (w1 + w2)
    Cd_struct = a * M + b * K

    mb = p["fahfs"]["m_b"]; cb = p["fahfs"]["c_b"]; kb = p["fahfs"]["k_b"]

    Z3 = np.zeros((n, n)); I3 = np.eye(n)
    A_struct = np.block([[Z3, I3], [-np.linalg.solve(M, K), -np.linalg.solve(M, Cd_struct)]])
    A_found = np.array([[0, 1], [-kb / mb, -cb / mb]])
    Ac = np.block([[A_struct, np.zeros((2 * n, 2))], [np.zeros((2, 2 * n)), A_found]])

    if mlfs_mode == "per_floor":
        nu_mlfs = n
        B_struct_u = np.block([[Z3], [np.linalg.solve(M, I3)]])  # 6x3
    elif mlfs_mode == "base_only":
        nu_mlfs = 1
        # single MLFS force injected at floor 1 (base active isolation)
        L_u = np.zeros((n, 1)); L_u[0, 0] = 1.0
        B_struct_u = np.block([[np.zeros((n, 1))], [np.linalg.solve(M, L_u)]])  # 6x1
    else:
        raise ValueError(f"unknown mlfs_mode {mlfs_mode}")

    B_struct_F = np.zeros((2 * n, 1))
    B_found_u = np.zeros((2, nu_mlfs))
    B_found_F = np.array([[0.0], [1.0 / mb]])
    Bc = np.block([[B_struct_u, B_struct_F], [B_found_u, B_found_F]])  # 8 x (nu_mlfs+1)

    ones_v = np.ones(n)
    E_struct_eq = np.concatenate([np.zeros(n), -ones_v]).reshape(-1, 1)
    E_struct_F = np.block([[Z3], [np.linalg.solve(M, I3)]])
    E_found = np.zeros((2, 1 + n))
    Ec = np.block([[E_struct_eq, E_struct_F], [E_found]])  # 8x4

    Ts = p["Ts"]; nx = 2 * n + 2; nu = nu_mlfs + 1; nd = 1 + n
    Ad = expm(Ac * Ts)
    Mu = expm(np.block([[Ac, Bc], [np.zeros((nu, nx + nu))]]) * Ts)
    Bd = Mu[:nx, nx:]
    Md = expm(np.block([[Ac, Ec], [np.zeros((nd, nx + nd))]]) * Ts)
    Ed = Md[:nx, nx:]

    return dict(M=M, K=K, C=Cd_struct, mb=mb, cb=cb, kb=kb,
                Ad=Ad, Bd=Bd, Ed=Ed, n=n, nx=nx, nu=nu, nd=nd, Ts=Ts,
                idx_q=slice(0, n), idx_qd=slice(n, 2 * n),
                idx_z=2 * n, idx_zd=2 * n + 1,
                nu_mlfs=nu_mlfs, mlfs_mode=mlfs_mode,
                omega=omega, fn=omega / (2 * np.pi))


# ---------------------------------------------------------- fluid
def fluid_force(h, v, z_base, p):
    floor_h = p["geom"]["floor_height"]
    n = len(p["geom"]["z"]); rho = p["fluid"]["rho"]; Cd = p["fluid"]["Cd"]
    A = p["fluid"]["A"]; g = 9.81; w = A / floor_h
    h = np.atleast_1d(h); v = np.atleast_1d(v)
    if np.isscalar(z_base): z_base = np.full_like(h, z_base, dtype=float)
    else: z_base = np.atleast_1d(z_base)
    Ff = np.zeros((n, h.size))
    for i in range(n):
        z_bot = i * floor_h
        d_i = np.clip(h - z_base - z_bot, 0, floor_h)
        drag = 0.5 * rho * Cd * (w * d_i) * v ** 2 * np.sign(v)
        hydro = 0.5 * rho * g * d_i ** 2 * w
        Ff[i] = drag + hydro
    return Ff


# ---------------------------------------------------------- disturbances
def seismic(t, p):
    PGA = p["seismic"]["PGA"]; f0 = p["seismic"]["f0"]; f1 = p["seismic"]["f1"]
    T = t[-1]
    phi = 2 * np.pi * (f0 * t + (f1 - f0) / (2 * T) * t ** 2)
    env = np.exp(-(((t - 0.35 * T) / (0.18 * T)) ** 2))
    return PGA * env * np.sin(phi) + 0.25 * PGA * env * np.sin(2 * np.pi * 15 * t + 1.3)


def get_disturbances(scenario, t, p):
    n = len(p["geom"]["z"]); N = t.size
    ag = np.zeros(N); h = np.zeros(N); v = np.zeros(N)
    if scenario == "seismic":
        ag = seismic(t, p)
    elif scenario == "flood":
        h = p["flood"]["h_max"] * (1 - np.exp(-t / p["flood"]["T_rise"]))
        v = p["flood"]["v_amp"] * np.sin(2 * np.pi * p["flood"]["f"] * t)
    elif scenario == "tsunami":
        tau = p["tsunami"]["tau"]; sech = lambda x: 1 / np.cosh(x)
        h = p["tsunami"]["h_peak"] * (sech((t - 3 * tau) / tau) ** 2 + 0.4 * (1 - np.exp(-t / (2 * tau))))
        v = p["tsunami"]["v_peak"] * sech((t - 3 * tau) / tau)
    elif scenario == "combined":
        ag = seismic(t, p)
        t0 = p["tsunami"]["start"]; mask = t >= t0
        if mask.any():
            tl = t[mask] - t0; tau = p["tsunami"]["tau"]; sech = lambda x: 1 / np.cosh(x)
            h[mask] = p["tsunami"]["h_peak"] * (sech((tl - 3 * tau) / tau) ** 2 + 0.4 * (1 - np.exp(-tl / (2 * tau))))
            v[mask] = p["tsunami"]["v_peak"] * sech((tl - 3 * tau) / tau)
    else:
        raise ValueError(scenario)
    return dict(t=t, ag=ag, h=h, v=v, params=p, n=n)


def build_reference(scenario, dist, sys, p):
    nx = sys["nx"]; N = dist["t"].size
    ref = np.zeros((nx, N))
    if scenario in ("flood", "tsunami", "combined"):
        margin = p["fahfs"]["safety_margin"]; z_max = p["fahfs"]["z_max"]
        z_target = np.maximum(dist["h"] + margin, 0.0)
        # sliding lookahead max (anticipation)
        look = int(round(p["fahfs"]["lookahead"] / sys["Ts"]))
        z_anticip = np.array([z_target[i:min(i + look + 1, N)].max() for i in range(N)])
        z_anticip = np.minimum(z_anticip, z_max)
        ref[sys["idx_z"], :] = z_anticip
    return ref


# ---------------------------------------------------------- MPC
def build_mpc(sys, Q, R, N, u_lim):
    Ad, Bd, Ed = sys["Ad"], sys["Bd"], sys["Ed"]
    nx, nu, nd = sys["nx"], sys["nu"], sys["nd"]
    A_pow = [np.eye(nx)]
    for _ in range(N + 1):
        A_pow.append(A_pow[-1] @ Ad)
    Phi = np.zeros((nx * N, nx)); Gamma = np.zeros((nx * N, nu * N)); Gd = np.zeros((nx * N, nd * N))
    for i in range(1, N + 1):
        Phi[(i - 1) * nx:i * nx] = A_pow[i]
        for j in range(1, i + 1):
            r = slice((i - 1) * nx, i * nx)
            cu = slice((j - 1) * nu, j * nu); cd = slice((j - 1) * nd, j * nd)
            Gamma[r, cu] = A_pow[i - j] @ Bd
            Gd[r, cd] = A_pow[i - j] @ Ed
    Qbar = np.kron(np.eye(N), Q); Rbar = np.kron(np.eye(N), R)
    H = 2 * (Gamma.T @ Qbar @ Gamma + Rbar); H = 0.5 * (H + H.T)
    L = np.linalg.eigvalsh(H).max() * 1.05
    return dict(Phi=Phi, Gamma=Gamma, Gd=Gd, Qbar=Qbar, Rbar=Rbar, H=H,
                u_min=np.tile(u_lim[:, 0], N), u_max=np.tile(u_lim[:, 1], N),
                Q=Q, R=R, N=N, nu=nu, nx=nx, nd=nd, L=L)


def fista_box(H, f, lb, ub, U0, L, max_iter=300, tol=1e-7):
    U = np.clip(U0, lb, ub); Y = U.copy(); t = 1.0
    for _ in range(max_iter):
        Up = U; grad = H @ Y + f
        U = np.clip(Y - grad / L, lb, ub)
        tn = (1 + np.sqrt(1 + 4 * t * t)) / 2
        Y = U + ((t - 1) / tn) * (U - Up); t = tn
        res = np.linalg.norm(U - np.clip(U - (H @ U + f) / L, lb, ub))
        if res < tol * max(1.0, np.linalg.norm(U)): break
    return U


def run_mpc(sys, mpc, dist, Tsim, x0, ref):
    nx, nu, nd = sys["nx"], sys["nu"], sys["nd"]; N = mpc["N"]; Ts = sys["Ts"]
    p = dist["params"]; n = sys["n"]
    x_log = np.zeros((nx, Tsim + 1)); u_log = np.zeros((nu, Tsim))
    x_log[:, 0] = x0
    ag_pad = np.concatenate([dist["ag"], np.zeros(N)])
    h_pad  = np.concatenate([dist["h"], np.full(N, dist["h"][-1])])
    v_pad  = np.concatenate([dist["v"], np.zeros(N)])
    ref_pad = np.concatenate([ref, np.tile(ref[:, -1:], (1, N))], axis=1)
    U_warm = np.zeros(nu * N)
    for k in range(Tsim):
        xk = x_log[:, k]; z_now = xk[sys["idx_z"]]
        h_pre = h_pad[k:k + N]; v_pre = v_pad[k:k + N]; ag_pre = ag_pad[k:k + N]
        Ff_pre = fluid_force(h_pre, v_pre, z_now, p)
        Dmat = np.vstack([ag_pre[None, :], Ff_pre])
        Dvec = Dmat.T.reshape(-1)
        Rmat = ref_pad[:, k:k + N]
        Rvec = Rmat.T.reshape(-1)
        diff = mpc["Phi"] @ xk + mpc["Gd"] @ Dvec - Rvec
        f = 2 * mpc["Gamma"].T @ mpc["Qbar"] @ diff
        U = fista_box(mpc["H"], f, mpc["u_min"], mpc["u_max"], U_warm, mpc["L"])
        uk = U[:nu]
        Ff_true = fluid_force(h_pad[k], v_pad[k], z_now, p).ravel()
        dk = np.concatenate([[ag_pad[k]], Ff_true])
        x_log[:, k + 1] = sys["Ad"] @ xk + sys["Bd"] @ uk + sys["Ed"] @ dk
        u_log[:, k] = uk
        U_warm = np.concatenate([U[nu:], np.zeros(nu)])
    return dict(x_log=x_log, u_log=u_log, t=np.arange(Tsim + 1) * Ts)


def run_lqr(sys, Q, R, dist, Tsim, x0, u_lim, ref):
    Ad, Bd = sys["Ad"], sys["Bd"]; p = dist["params"]
    P = solve_discrete_are(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
    nx, nu = sys["nx"], sys["nu"]
    x_log = np.zeros((nx, Tsim + 1)); u_log = np.zeros((nu, Tsim)); x_log[:, 0] = x0
    for k in range(Tsim):
        xk = x_log[:, k]; z_now = xk[sys["idx_z"]]
        err = xk - ref[:, k]
        uk = -K @ err
        uk = np.clip(uk, u_lim[:, 0], u_lim[:, 1])
        Ff = fluid_force(dist["h"][k], dist["v"][k], z_now, p).ravel()
        dk = np.concatenate([[dist["ag"][k]], Ff])
        x_log[:, k + 1] = Ad @ xk + Bd @ uk + sys["Ed"] @ dk
        u_log[:, k] = uk
    return dict(x_log=x_log, u_log=u_log, t=np.arange(Tsim + 1) * sys["Ts"])


def run_uncontrolled(sys, dist, Tsim, x0):
    Ad, Ed = sys["Ad"], sys["Ed"]; p = dist["params"]
    nx, nu = sys["nx"], sys["nu"]
    x_log = np.zeros((nx, Tsim + 1)); u_log = np.zeros((nu, Tsim)); x_log[:, 0] = x0
    for k in range(Tsim):
        xk = x_log[:, k]; z_now = xk[sys["idx_z"]]
        Ff = fluid_force(dist["h"][k], dist["v"][k], z_now, p).ravel()
        dk = np.concatenate([[dist["ag"][k]], Ff])
        x_log[:, k + 1] = Ad @ xk + Ed @ dk
    return dict(x_log=x_log, u_log=u_log, t=np.arange(Tsim + 1) * sys["Ts"])


# ---------------------------------------------------------- metrics
def compute_metrics(out, sys, baseline=None, ref=None):
    n = sys["n"]; Ts = sys["Ts"]
    q = out["x_log"][sys["idx_q"]]; qd = out["x_log"][sys["idx_qd"]]
    zb = out["x_log"][sys["idx_z"]]
    peak_disp = np.max(np.abs(q), axis=1)
    drift = q.copy(); drift[1:] = q[1:] - q[:-1]
    peak_drift = np.max(np.abs(drift), axis=1)
    qdd = np.hstack([np.zeros((n, 1)), np.diff(qd, axis=1) / Ts])
    peak_acc = np.max(np.abs(qdd), axis=1)
    rms_disp = np.sqrt(np.mean(q ** 2, axis=1))
    settling = np.zeros(n)
    for i in range(n):
        thr = 0.05 * peak_disp[i]
        idx = np.where(np.abs(q[i]) > thr)[0]
        settling[i] = idx[-1] * Ts if idx.size else 0.0
    if out["u_log"].size == 0:
        ctrl_energy = 0.0; peak_force = np.zeros(sys["nu"])
    else:
        ctrl_energy = float(np.sum(out["u_log"] ** 2) * Ts)
        peak_force = np.max(np.abs(out["u_log"]), axis=1)
    fairness = float((peak_drift.sum() ** 2) / (n * np.sum(peak_drift ** 2) + 1e-12))
    if baseline is not None:
        bp = np.max(np.abs(baseline["x_log"][sys["idx_q"]]), axis=1)
        reduction = 100 * (bp - peak_disp) / np.maximum(bp, 1e-12)
        red_avg = float(np.mean(reduction))
    else:
        reduction = np.full(n, np.nan); red_avg = float("nan")
    out_m = dict(peak_disp=peak_disp, peak_drift=peak_drift, peak_acc=peak_acc,
                 rms_disp=rms_disp, settling=settling, ctrl_energy=ctrl_energy,
                 peak_force=peak_force, fairness=fairness,
                 reduction=reduction, reduction_avg=red_avg,
                 peak_z=float(zb.max()), final_z=float(zb[-1]))
    if ref is not None:
        z_ref = ref[sys["idx_z"]]
        out_m["z_track_rmse"] = float(np.sqrt(np.mean((zb - z_ref) ** 2)))
        out_m["z_track_peak_err"] = float(np.max(np.abs(zb - z_ref)))
    return out_m


# ---------------------------------------------------------- tuning
def tune_weights(sys, p, dist, Tsim, x0, u_lim, baseline, ref,
                 grid_qd=None, grid_qv=None, grid_qz=None,
                 grid_rm=None, grid_rf=None, horizon=15,
                 w_red=0.35, w_fair=0.20, w_eng=0.10, w_sat=0.05, w_z=0.30):
    if grid_qd is None: grid_qd = np.logspace(2, 5, 4)
    if grid_qv is None: grid_qv = np.logspace(0, 3, 3)
    if grid_qz is None: grid_qz = [1e3, 1e5, 1e7]
    if grid_rm is None: grid_rm = [1e-6, 1e-4]
    if grid_rf is None: grid_rf = [1e-8, 1e-6]
    n = sys["n"]; nu_mlfs = sys["nu_mlfs"]
    E_max = float(np.sum(u_lim[:, 1] ** 2) * Tsim * sys["Ts"])
    z_ref_max = max(ref[sys["idx_z"]].max(), 1e-9)
    best = dict(score=-np.inf); log = []
    for qd in grid_qd:
        for qv in grid_qv:
            for qz in grid_qz:
                for rm in grid_rm:
                    for rf in grid_rf:
                        Q_struct = np.block([[qd * np.eye(n), np.zeros((n, n))],
                                             [np.zeros((n, n)), qv * np.eye(n)]])
                        Q_found = np.diag([qz, 1.0])
                        Q = np.block([[Q_struct, np.zeros((2 * n, 2))],
                                      [np.zeros((2, 2 * n)), Q_found]])
                        R = np.diag([rm] * nu_mlfs + [rf])
                        mpc = build_mpc(sys, Q, R, horizon, u_lim)
                        trial = run_mpc(sys, mpc, dist, Tsim, x0, ref)
                        m = compute_metrics(trial, sys, baseline, ref)
                        red = max(min(m["reduction_avg"] / 100, 1), -1)
                        e_norm = m["ctrl_energy"] / E_max
                        sat = float(np.mean(np.any(np.abs(trial["u_log"]) >= 0.99 * u_lim[:, 1, None], axis=0)))
                        z_qual = max(0.0, 1 - m["z_track_rmse"] / z_ref_max)
                        score = w_red * red + w_fair * m["fairness"] - w_eng * e_norm - w_sat * sat + w_z * z_qual
                        log.append((qd, qv, qz, rm, rf, m["reduction_avg"], m["fairness"],
                                    e_norm, sat, z_qual, score))
                        if score > best["score"]:
                            best.update(score=score, qd=qd, qv=qv, qz=qz, rm=rm, rf=rf,
                                        Q=Q, R=R, metrics=m, run=trial)
    return best, log


# ---------------------------------------------------------- main
def main(mlfs_mode="per_floor", suffix=None):
    if suffix is None:
        suffix = "" if mlfs_mode == "per_floor" else f"_{mlfs_mode}"
    p = dict(
        m=[333.3, 333.3, 333.3], k=[1.5e5, 1.5e5, 1.5e5], zeta=0.02, Ts=5e-3,
        fahfs=dict(m_b=1500, c_b=1.5e4, k_b=0.0, z_max=5.0,
                   safety_margin=0.5, lookahead=3.0),
        geom=dict(floor_height=3.0, z=[3.0, 6.0, 9.0]),
        fluid=dict(rho=1000, Cd=1.2, A=12.0),
        seismic=dict(PGA=3.5, f0=0.5, f1=8.0),
        flood=dict(h_max=2.5, T_rise=20, v_amp=1.5, f=0.05),
        tsunami=dict(h_peak=4.5, v_peak=6.0, tau=1.5, start=12.0),
    )
    if mlfs_mode == "per_floor":
        u_lim = np.array([[-5000, 5000], [-5000, 5000], [-5000, 5000], [-1e5, 1e5]])
    elif mlfs_mode == "base_only":
        # Single MLFS at base; raise its limit since it carries the whole structure.
        u_lim = np.array([[-15000, 15000], [-1e5, 1e5]])
    else:
        raise ValueError(mlfs_mode)

    sys = build_model(p, mlfs_mode=mlfs_mode)
    print(f"=== mode={mlfs_mode} ===")
    print(f"Natural freq (Hz): {np.round(sys['fn'], 4)}")
    print(f"nx={sys['nx']}  nu={sys['nu']}  nd={sys['nd']}")

    x0 = np.zeros(sys["nx"])
    T_sec = dict(seismic=20, flood=40, tsunami=15, combined=25)

    # tune on combined scenario (covers both regimes)
    print("\n=== tuning weights ...")
    T_tune = 8
    t_tune = np.arange(0, T_tune, p["Ts"])
    d_tune = get_disturbances("combined", t_tune, p)
    r_tune = build_reference("combined", d_tune, sys, p)
    Tsim_tune = t_tune.size - 1
    base_tune = run_uncontrolled(sys, d_tune, Tsim_tune, x0)
    best, _ = tune_weights(sys, p, d_tune, Tsim_tune, x0, u_lim, base_tune, r_tune)
    print(f"best: q_d={best['qd']:.3g} q_v={best['qv']:.3g} q_z={best['qz']:.3g} "
          f"r_m={best['rm']:.3g} r_f={best['rf']:.3g}  score={best['score']:.4f}")
    Q_star = best["Q"]; R_star = best["R"]
    mpc_full = build_mpc(sys, Q_star, R_star, 20, u_lim)

    out_all = {}
    for sc, T in T_sec.items():
        print(f"\n[{sc}] running ...")
        t = np.arange(0, T, p["Ts"])
        dist = get_disturbances(sc, t, p)
        ref = build_reference(sc, dist, sys, p)
        Tsim = t.size - 1
        unc = run_uncontrolled(sys, dist, Tsim, x0)
        lqr = run_lqr(sys, Q_star, R_star, dist, Tsim, x0, u_lim, ref)
        mpc = run_mpc(sys, mpc_full, dist, Tsim, x0, ref)
        m_unc = compute_metrics(unc, sys)
        m_lqr = compute_metrics(lqr, sys, unc, ref)
        m_mpc = compute_metrics(mpc, sys, unc, ref)
        out_all[sc] = dict(
            unc=dict(metrics=m_unc, x=unc["x_log"], u=unc["u_log"], t=unc["t"]),
            lqr=dict(metrics=m_lqr, x=lqr["x_log"], u=lqr["u_log"], t=lqr["t"]),
            mpc=dict(metrics=m_mpc, x=mpc["x_log"], u=mpc["u_log"], t=mpc["t"]),
            ref=ref, dist=dict(t=dist["t"], ag=dist["ag"], h=dist["h"], v=dist["v"]),
        )
        print(f"  unc peak roof  = {m_unc['peak_disp'][2]*1e3:.2f} mm")
        print(f"  MPC peak roof  = {m_mpc['peak_disp'][2]*1e3:.2f} mm  ({m_mpc['reduction_avg']:.2f}% avg)")
        if sc != "seismic":
            print(f"  MPC z_base peak={m_mpc['peak_z']:.2f} m  z_track_rmse={m_mpc['z_track_rmse']:.3f} m")

    outdir = Path(__file__).parent / "results"; outdir.mkdir(exist_ok=True)
    np.savez(outdir / f"matlab_mirror{suffix}.npz",
             natural_freq=sys["fn"],
             **{f"{sc}_{tag}_x": out_all[sc][tag]["x"] for sc in out_all for tag in ("unc", "lqr", "mpc")},
             **{f"{sc}_{tag}_u": out_all[sc][tag]["u"] for sc in out_all for tag in ("unc", "lqr", "mpc")},
             **{f"{sc}_t": out_all[sc]["mpc"]["t"] for sc in out_all},
             **{f"{sc}_ref": out_all[sc]["ref"] for sc in out_all},
             **{f"{sc}_h": out_all[sc]["dist"]["h"] for sc in out_all})

    summary = {sc: {tag: {k: (v.tolist() if hasattr(v, "tolist") else v)
                          for k, v in out_all[sc][tag]["metrics"].items()}
                    for tag in ("unc", "lqr", "mpc")}
               for sc in out_all}
    summary["_meta"] = dict(natural_freq_hz=sys["fn"].tolist(), Ts=p["Ts"],
                            mlfs_mode=mlfs_mode,
                            nu_mlfs=sys["nu_mlfs"],
                            Q_star_diag=np.diag(Q_star).tolist(),
                            R_star_diag=np.diag(R_star).tolist(),
                            u_lim=u_lim.tolist(),
                            best=dict(qd=float(best["qd"]), qv=float(best["qv"]),
                                      qz=float(best["qz"]), rm=float(best["rm"]),
                                      rf=float(best["rf"]), score=float(best["score"])))
    (outdir / f"summary{suffix}.json").write_text(json.dumps(summary, indent=2))
    print(f"\nSaved -> {outdir} (suffix='{suffix}')")


if __name__ == "__main__":
    import sys as _sys
    mode = _sys.argv[1] if len(_sys.argv) > 1 else "per_floor"
    main(mlfs_mode=mode)
