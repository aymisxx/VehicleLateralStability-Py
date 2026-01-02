# src/controllers.py
from __future__ import annotations
import numpy as np
from scipy.linalg import solve_continuous_are

def lqr_gain(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Continuous-time LQR gain K (1x2)."""
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)
    return K

def smc_basic_u(e1: float, e2: float, lambda_s: float, k_s: float, phi: float) -> float:
    """Basic SMC corrective input u_smc."""
    s = e2 + lambda_s * e1
    sat_bl = np.clip(s / phi, -1.0, 1.0)
    return float(-k_s * sat_bl)

def smc_eqcorr_u(
    A: np.ndarray,
    B: np.ndarray,
    x: np.ndarray,
    r_des: float,
    r_des_dot: float,
    u_ff: float,
    lambda_s: float,
    k_s: float,
    phi: float,
) -> tuple[float, float, float]:
    """
    SMC with equivalent correction about feedforward:
      u_total = u_ff + u_eq_corr + u_sw
    Returns (u_eq_corr, u_sw, u_corr_total) where u_corr_total = u_eq_corr + u_sw
    """
    # e = [Vy - 0, r - r_des]
    e = np.array([[float(x[0])],
                  [float(x[1]) - float(r_des)]])

    C = np.array([[lambda_s, 1.0]])  # (1,2)
    s = float((C @ e).item())

    # x_des_dot = [0, r_des_dot]^T
    x_des_dot = np.array([[0.0],
                          [float(r_des_dot)]])

    # Ax_term = A x + B u_ff - x_des_dot
    xcol = np.array([[float(x[0])], [float(x[1])]])
    Ax_term = A @ xcol + B * float(u_ff) - x_des_dot
    CB = float((C @ B).item())

    u_eq_corr = 0.0 if abs(CB) < 1e-9 else float(-(C @ Ax_term).item() / CB)

    sat_bl = np.clip(s / phi, -1.0, 1.0)
    u_sw = float(-k_s * sat_bl)

    return u_eq_corr, u_sw, (u_eq_corr + u_sw)