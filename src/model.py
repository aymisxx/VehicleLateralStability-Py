# src/model.py
from __future__ import annotations
import numpy as np
from .params import Params

def bicycle_AB(p: Params) -> tuple[np.ndarray, np.ndarray]:
    """
    Linear bicycle model (small angles), states x=[Vy, r]^T, input u=delta.
    x_dot = A x + B u
    """
    m, Iz = p.m, p.Iz
    lf, lr = p.lf, p.lr
    Caf, Car = p.Caf, p.Car
    Vx = p.Vx

    if Vx <= 0:
        raise ValueError("Vx must be > 0 for bicycle model.")

    a11 = -(Caf + Car) / (m * Vx)
    a12 = -Vx - (Caf * lf - Car * lr) / (m * Vx)
    a21 = -(Caf * lf - Car * lr) / (Iz * Vx)
    a22 = -(Caf * lf**2 + Car * lr**2) / (Iz * Vx)

    b1 = Caf / m
    b2 = Caf * lf / Iz

    A = np.array([[a11, a12],
                  [a21, a22]], dtype=float)
    B = np.array([[b1],
                  [b2]], dtype=float)
    return A, B