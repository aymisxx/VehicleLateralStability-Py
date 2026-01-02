# src/metrics.py
from __future__ import annotations
import numpy as np
import pandas as pd

def rms(x: np.ndarray) -> float:
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(x * x)))

def metric_pack(name, Vy_sig, r_sig, u_total_sig, r_des_sig, dt):
    Vy_sig = np.asarray(Vy_sig, dtype=float)
    r_sig = np.asarray(r_sig, dtype=float)
    u_total_sig = np.asarray(u_total_sig, dtype=float)
    r_des_sig = np.asarray(r_des_sig, dtype=float)

    e_r = r_sig - r_des_sig
    du_dt = np.diff(u_total_sig) / float(dt)

    return {
        "Controller": name,
        "RMS(|r-r_des|) [rad/s]": rms(e_r),
        "Max(|r-r_des|) [rad/s]": float(np.max(np.abs(e_r))),
        "RMS(|Vy|) [m/s]": rms(Vy_sig),
        "Max(|Vy|) [m/s]": float(np.max(np.abs(Vy_sig))),
        "RMS(|u|) [rad]": rms(u_total_sig),
        "Max(|u|) [rad]": float(np.max(np.abs(u_total_sig))),
        "RMS(|du/dt|) [rad/s]": rms(du_dt) if du_dt.size else np.nan,
    }

def build_metrics_table(p_dt: float, r_des, ff, lqr, smc_basic, smc_eqcorr) -> pd.DataFrame:
    rows = [
        metric_pack("Feedforward", ff["Vy"], ff["r"], ff["u_ff"], r_des, p_dt),
        metric_pack("LQR", lqr["Vy_lqr"], lqr["r_lqr"], lqr["u_total_lqr"], r_des, p_dt),
        metric_pack("SMC-basic", smc_basic["Vy_smc"], smc_basic["r_smc"], smc_basic["u_total_smc"], r_des, p_dt),
        metric_pack("SMC-eq-corr", smc_eqcorr["Vy_smc_eqcorr"], smc_eqcorr["r_smc_eqcorr"], smc_eqcorr["u_total_smc_eqcorr"], r_des, p_dt),
    ]
    return pd.DataFrame(rows)