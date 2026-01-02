# scripts/run_all.py
from __future__ import annotations

# Make repo-root importable so "import src.*" works from anywhere
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

import pandas as pd

from src.params import Params
from src.simulate import (
    simulate_feedforward,
    simulate_lqr,
    simulate_smc_basic,
    simulate_smc_eqcorr,
)
from src.metrics import build_metrics_table
from src.plotting import ensure_dir, plot_profile, plot_comparison

def main():
    p = Params()
    outdir = ensure_dir("results")

    ff = simulate_feedforward(p)
    lqr = simulate_lqr(p)
    smc = simulate_smc_basic(p)
    smc_eq = simulate_smc_eqcorr(p)

    # Plots
    plot_profile(ff["t"], ff["rho"], ff["u_ff"], outdir=outdir)
    plot_comparison(ff["t"], ff["r_des"], ff, lqr, smc, smc_eq, outdir=outdir)

    # Metrics table
    df = build_metrics_table(p.dt, ff["r_des"], ff, lqr, smc, smc_eq)

    pd.set_option("display.precision", 6)
    print("\nMetrics table:\n")
    print(df.to_string(index=False))

    print("\nSorted by RMS(|r-r_des|):\n")
    print(df.sort_values("RMS(|r-r_des|) [rad/s]").to_string(index=False))

if __name__ == "__main__":
    main()