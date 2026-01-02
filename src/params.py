# src/params.py
from __future__ import annotations
from dataclasses import dataclass

@dataclass
class Params:
    # Vehicle parameters
    m: float = 1500.0     # kg
    Iz: float = 3000.0    # kg*m^2
    lf: float = 1.2       # m
    lr: float = 1.6       # m
    Caf: float = 80000.0  # N/rad
    Car: float = 80000.0  # N/rad
    Vx: float = 15.0      # m/s (constant)

    # Simulation
    t0: float = 0.0
    tf: float = 25.0
    dt: float = 0.001

    # Curvature profile
    rho_amp: float = 0.01
    t1: float = 5.0
    t2: float = 10.0
    t3: float = 15.0
    t4: float = 20.0

    # Actuator saturation
    delta_max: float = 0.5  # rad (~28.6 deg)

    @property
    def L(self) -> float:
        return self.lf + self.lr