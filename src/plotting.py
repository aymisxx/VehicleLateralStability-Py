# src/plotting.py
from __future__ import annotations
import os
import numpy as np

# Headless-safe backend (so saving plots never hangs)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def ensure_dir(path: str) -> str:
    os.makedirs(path, exist_ok=True)
    return path

def savefig(path: str):
    plt.tight_layout()
    plt.savefig(path, dpi=200)
    plt.close()
    print(f"Saved: {path}")

def plot_profile(t, rho, u_ff, outdir=None):
    plt.figure()
    plt.plot(t, rho)
    plt.xlabel("Time (s)")
    plt.ylabel("Curvature ρ(t) [1/m]")
    plt.title("Curvature Profile")
    plt.grid(True)
    if outdir:
        savefig(os.path.join(outdir, "01_curvature.png"))

    plt.figure()
    plt.plot(t, u_ff)
    plt.xlabel("Time (s)")
    plt.ylabel("Feedforward Steering u_ff(t) [rad]")
    plt.title("Feedforward Steering (Ackerman approx)")
    plt.grid(True)
    if outdir:
        savefig(os.path.join(outdir, "02_u_ff.png"))

def plot_comparison(t, r_des, ff, lqr, smc, smc_eq, outdir=None):
    plt.figure()
    plt.plot(t, r_des, "--", label="Desired r_des")
    plt.plot(t, ff["r"], label="Feedforward only")
    plt.plot(t, lqr["r_lqr"], label="LQR")
    plt.plot(t, smc["r_smc"], label="SMC (basic)")
    plt.plot(t, smc_eq["r_smc_eqcorr"], label="SMC (eq-corr)")
    plt.xlabel("Time (s)")
    plt.ylabel("Yaw rate (rad/s)")
    plt.title("Yaw Rate Tracking Comparison")
    plt.legend()
    plt.grid(True)
    if outdir:
        savefig(os.path.join(outdir, "03_yaw_compare.png"))

    plt.figure()
    plt.plot(t, ff["Vy"], label="Feedforward only")
    plt.plot(t, lqr["Vy_lqr"], label="LQR")
    plt.plot(t, smc["Vy_smc"], label="SMC (basic)")
    plt.plot(t, smc_eq["Vy_smc_eqcorr"], label="SMC (eq-corr)")
    plt.xlabel("Time (s)")
    plt.ylabel("Lateral velocity Vy (m/s)")
    plt.title("Lateral Velocity Comparison")
    plt.legend()
    plt.grid(True)
    if outdir:
        savefig(os.path.join(outdir, "04_vy_compare.png"))