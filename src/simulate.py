# src/simulate.py
from __future__ import annotations
import numpy as np
from scipy.integrate import solve_ivp

from .params import Params
from .profiles import rho_profile, r_des_from_rho, u_ff_from_rho, sat
from .controllers import lqr_gain, smc_basic_u, smc_eqcorr_u
from .model import bicycle_AB


def make_time(p: Params) -> np.ndarray:
    # inclusive end so plots match notebook style
    return np.arange(p.t0, p.tf + p.dt, p.dt)


def _solve(ode, p: Params, t: np.ndarray, x0: np.ndarray):
    """
    Key detail: curvature + steering inputs are step-like (discontinuous).
    Some implicit solvers may skip evaluating the RHS inside those intervals.
    We therefore cap max_step so the solver must traverse each segment.
    """
    sol = solve_ivp(
        ode,
        (p.t0, p.tf),
        x0,
        t_eval=t,
        method="LSODA",        # robust for stiff/nonstiff switching
        max_step=0.01,         # IMPORTANT: do not let solver skip step intervals
        rtol=1e-7,
        atol=1e-9,
    )
    if sol.status < 0:
        raise RuntimeError(f"solve_ivp failed: {sol.message}")
    return sol


def simulate_feedforward(p: Params) -> dict:
    t = make_time(p)
    A, B = bicycle_AB(p)

    rho = rho_profile(t, p)
    r_des = r_des_from_rho(rho, p)
    u_ff = u_ff_from_rho(rho, p)

    def ode(ts, x):
        u = float(np.interp(ts, t, u_ff))
        return (A @ x.reshape(2, 1) + B * u).flatten()

    x0 = np.array([0.0, 0.0], dtype=float)
    sol = _solve(ode, p, t, x0)

    return {
        "t": t,
        "A": A,
        "B": B,
        "rho": rho,
        "r_des": r_des,
        "u_ff": u_ff,
        "Vy": sol.y[0],
        "r": sol.y[1],
    }


def simulate_lqr(p: Params, Q=None, R=None) -> dict:
    base = simulate_feedforward(p)
    t, A, B = base["t"], base["A"], base["B"]
    r_des, u_ff = base["r_des"], base["u_ff"]

    if Q is None:
        Q = np.diag([10.0, 50.0])
    if R is None:
        R = np.array([[1.0]])

    K = lqr_gain(A, B, Q, R)

    def ode(ts, x):
        r_des_t = float(np.interp(ts, t, r_des))
        u_ff_t = float(np.interp(ts, t, u_ff))

        e = np.array([[x[0] - 0.0],
                      [x[1] - r_des_t]])
        u_corr = float((-(K @ e)).item())
        u_total = sat(u_ff_t + u_corr, p.delta_max)

        return (A @ x.reshape(2, 1) + B * u_total).flatten()

    x0 = np.array([0.0, 0.0], dtype=float)
    sol = _solve(ode, p, t, x0)

    Vy = sol.y[0]
    r = sol.y[1]

    u_corr_arr = np.zeros_like(t, dtype=float)
    u_total_arr = np.zeros_like(t, dtype=float)
    for i, (Vy_i, r_i) in enumerate(zip(Vy, r)):
        e = np.array([[Vy_i],
                      [r_i - r_des[i]]])
        u_corr = float((-(K @ e)).item())
        u_corr_arr[i] = u_corr
        u_total_arr[i] = sat(u_ff[i] + u_corr, p.delta_max)

    return {
        **base,
        "K": K,
        "Vy_lqr": Vy,
        "r_lqr": r,
        "u_corr_lqr": u_corr_arr,
        "u_total_lqr": u_total_arr,
    }


def simulate_smc_basic(p: Params, lambda_s=5.0, k_s=5.0, phi=0.02) -> dict:
    base = simulate_feedforward(p)
    t, A, B = base["t"], base["A"], base["B"]
    r_des, u_ff = base["r_des"], base["u_ff"]

    def ode(ts, x):
        r_des_t = float(np.interp(ts, t, r_des))
        u_ff_t = float(np.interp(ts, t, u_ff))

        e1 = float(x[0])              # Vy - 0
        e2 = float(x[1] - r_des_t)    # r - r_des
        u_smc = smc_basic_u(e1, e2, lambda_s, k_s, phi)
        u_total = sat(u_ff_t + u_smc, p.delta_max)

        return (A @ x.reshape(2, 1) + B * u_total).flatten()

    x0 = np.array([0.0, 0.0], dtype=float)
    sol = _solve(ode, p, t, x0)

    Vy = sol.y[0]
    r = sol.y[1]

    u_smc_arr = np.zeros_like(t, dtype=float)
    u_total_arr = np.zeros_like(t, dtype=float)
    for i, (Vy_i, r_i) in enumerate(zip(Vy, r)):
        e1 = float(Vy_i)
        e2 = float(r_i - r_des[i])
        u_smc = smc_basic_u(e1, e2, lambda_s, k_s, phi)
        u_smc_arr[i] = u_smc
        u_total_arr[i] = sat(u_ff[i] + u_smc, p.delta_max)

    return {
        **base,
        "Vy_smc": Vy,
        "r_smc": r,
        "u_smc": u_smc_arr,
        "u_total_smc": u_total_arr,
    }


def simulate_smc_eqcorr(p: Params, lambda_s=5.0, k_s=1.0, phi=0.05) -> dict:
    base = simulate_feedforward(p)
    t, A, B = base["t"], base["A"], base["B"]
    r_des, u_ff = base["r_des"], base["u_ff"]

    # Baseline consistent with step curvature: treat r_des_dot ~ 0
    r_des_dot = np.zeros_like(t, dtype=float)

    def ode(ts, x):
        r_des_t = float(np.interp(ts, t, r_des))
        r_des_dot_t = float(np.interp(ts, t, r_des_dot))
        u_ff_t = float(np.interp(ts, t, u_ff))

        u_eq_corr, u_sw, u_corr_total = smc_eqcorr_u(
            A=A, B=B, x=np.asarray(x, dtype=float),
            r_des=r_des_t, r_des_dot=r_des_dot_t,
            u_ff=u_ff_t,
            lambda_s=lambda_s, k_s=k_s, phi=phi,
        )
        u_total = sat(u_ff_t + u_corr_total, p.delta_max)
        return (A @ np.asarray(x, dtype=float).reshape(2, 1) + B * u_total).flatten()

    x0 = np.array([0.0, 0.0], dtype=float)
    sol = _solve(ode, p, t, x0)

    Vy = sol.y[0]
    r = sol.y[1]

    u_eqcorr_arr = np.zeros_like(t, dtype=float)
    u_sw_arr = np.zeros_like(t, dtype=float)
    u_total_arr = np.zeros_like(t, dtype=float)
    for i, (Vy_i, r_i) in enumerate(zip(Vy, r)):
        u_eq_corr, u_sw, u_corr_total = smc_eqcorr_u(
            A=A, B=B, x=np.array([Vy_i, r_i], dtype=float),
            r_des=float(r_des[i]), r_des_dot=float(r_des_dot[i]),
            u_ff=float(u_ff[i]),
            lambda_s=lambda_s, k_s=k_s, phi=phi,
        )
        u_eqcorr_arr[i] = u_eq_corr
        u_sw_arr[i] = u_sw
        u_total_arr[i] = sat(u_ff[i] + u_corr_total, p.delta_max)

    return {
        **base,
        "Vy_smc_eqcorr": Vy,
        "r_smc_eqcorr": r,
        "u_eq_corr": u_eqcorr_arr,
        "u_sw": u_sw_arr,
        "u_total_smc_eqcorr": u_total_arr,
    }