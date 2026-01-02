# src/profiles.py
from __future__ import annotations
import numpy as np
from .params import Params

def rho_profile(t: np.ndarray, p: Params) -> np.ndarray:
    """Piecewise curvature profile rho(t) [1/m]: 0 -> +rho -> 0 -> -rho -> 0."""
    rho = np.zeros_like(t, dtype=float)
    rho[(t >= p.t1) & (t < p.t2)] = +p.rho_amp
    rho[(t >= p.t3) & (t < p.t4)] = -p.rho_amp
    return rho

def r_des_from_rho(rho: np.ndarray, p: Params) -> np.ndarray:
    """Desired yaw rate: r_des = Vx * rho."""
    return p.Vx * rho

def u_ff_from_rho(rho: np.ndarray, p: Params) -> np.ndarray:
    """Feedforward steering: u_ff = L * rho."""
    return p.L * rho

def sat(u: float, umax: float) -> float:
    """Scalar saturation returning Python float."""
    return float(np.clip(u, -umax, umax))