# **VehicleLateralStability-Py**

Linear Vehicle Lateral Dynamics:

**Feedforward, LQR, and Sliding Mode Control (Python Implementation)**

---

## Abstract

This project presents a Python-based study of **vehicle lateral stability control** based on a linear bicycle model.

Feedforward steering, Linear Quadratic Regulator (LQR), and Sliding Mode Control (SMC) strategies are implemented and compared under identical road curvature disturbances.

The objective is to evaluate **yaw-rate tracking performance**, **lateral velocity regulation**, **control effort**, and **robustness trade-offs** through reproducible numerical simulations. The codebase is modular and designed for extension to more advanced models and controllers.

---

## Introduction

Vehicle lateral dynamics are fundamental to **handling stability, safety, and autonomous driving systems**.  
Accurate yaw-rate tracking and lateral motion regulation help maintain path-following performance during curvature changes.

This project models lateral vehicle behavior using a **linear bicycle model** and compares three controller classes:

- **Feedforward steering** based on road curvature (Ackermann-style approximation)
- **LQR** for optimal state feedback under nominal dynamics
- **SMC** for robustness-oriented feedback

All controllers are evaluated under the same curvature profile to enable fair comparison.

---

## Core Concepts and Mathematical Model

### 1) Linear Bicycle Model

States and input:

- $V_y$: lateral velocity (m/s)  
- $r$: yaw rate (rad/s)  
- $\delta$: front steering angle (rad)

State vector:

$$
x = \begin{bmatrix} V_y \\ r \end{bmatrix}
$$

Continuous-time dynamics:

$$
\dot{x} = A x + B\,\delta
$$

The matrices $A$ and $B$ depend on vehicle parameters (mass, yaw inertia, axle distances, cornering stiffnesses) and the constant longitudinal speed $U_x$.

---

### 2) Road Curvature, Reference, and Feedforward Steering

Given road curvature command $\rho(t)$ (1/m):

Desired yaw rate:

$$
r_{\text{des}}(t) = U_x\,\rho(t)
$$

Feedforward steering (simple geometric approximation):

$$
\delta_{\text{ff}}(t) = L\,\rho(t), \quad L = l_f + l_r
$$

Feedforward provides the baseline steering required for constant curvature, but does not correct transient deviations.

---

### 3) LQR Feedback

LQR minimizes the quadratic cost:

$$
J = \int_0^{\infty} \left( x^\top Q x + u^\top R u \right) dt
$$

Tracking form used here (reference in yaw-rate, $V_{y,\text{des}}=0$):

$$
\delta(t) = \delta_{\text{ff}}(t) - K\,\big(x(t) - x_{\text{des}}(t)\big), 
\quad
x_{\text{des}}(t) =
\begin{bmatrix}
0 \\ r_{\text{des}}(t)
\end{bmatrix}
$$

LQR typically achieves strong tracking under nominal dynamics, at the cost of increased steering activity.

---

### 4) Sliding Mode Control (SMC)

Sliding surface:

$$
s = (r - r_{\text{des}}) + \lambda\,(V_y - V_{y,\text{des}})
$$

Total steering input:

$$
\delta(t) = \delta_{\text{ff}}(t) + \delta_{\text{eq}}(t) + \delta_{\text{sw}}(t)
$$

A saturation boundary layer is used for the switching term to reduce chattering:

$$
\delta_{\text{sw}}(t) = -k\,\mathrm{sat}\left(\frac{s}{\phi}\right)
$$

SMC emphasizes robustness to modeling uncertainty and disturbances, but can suffer from aggressive control action and actuator saturation.

---

## Folder Structure

```text
VehicleLateralStability-Py/
  requirements.txt                    # dependencies
  README.md                           # this file/text
  notebooks/
    VehicleLateralStability-Py.ipynb  # jupyter notebook implementation
    VehicleLateralStability-Py.pdf    # pdf-output of the notebook
  src/
    params.py
    profiles.py
    model.py
    controllers.py
    simulate.py
    metrics.py
    plotting.py
  scripts/
    run_all.py                        # main script
  results/
    01_curvature.png
    02_u_ff.png
    03_yaw_compare.png
    04_vy_compare.png
```

---

## How to Run

### 1) Create a virtual environment

```bash
python -m venv venv
```

### 2) Activate the environment

**Windows**
```bash
venv\Scripts\activate
```

**Linux / macOS**
```bash
source venv/bin/activate
```

### 3) Install dependencies

```bash
pip install -r requirements.txt
```

### 4) Run all simulations + save results

```bash
python scripts/run_all.py
```

Outputs:
- Figures saved into `results/`
- Metrics table printed to the terminal

---

## Results and Discussion

### Feedforward (baseline)
- Produces the required steering for constant curvature via $\delta_{\text{ff}} = L\rho$.
- Tracks $r_{\text{des}}$ only approximately and shows transient tracking error during curvature transitions.
- Lowest steering effort and smoothest steering profile.

### LQR (optimal under nominal dynamics)
- Achieves the best yaw-rate tracking among the tested controllers.
- Uses additional steering correction to reject transient error, increasing steering activity and control rate.
- Performance depends on how accurately the linear model matches the true vehicle.

### SMC (robust control under constraints)
- In this setup, SMC prioritizes robustness / sliding-surface convergence rather than minimizing yaw-rate tracking error directly.
- Switching action can drive high control rates; with actuator saturation, the controller may not realize the intended sliding behavior.
- The “equivalent correction” variant reduces chattering (lower control rate) but does not necessarily improve tracking accuracy, especially when reference commands are step-like and steering authority is limited.

**Key takeaway:** LQR excels at tracking under the nominal model, while SMC highlights robustness–performance trade-offs and sensitivity to actuator limits.

---

## Conclusion

This project delivers a complete, reproducible Python implementation of vehicle lateral dynamics and classical controllers:

- Feedforward provides a clean baseline.
- LQR offers superior tracking performance under nominal assumptions.
- SMC provides a robustness-oriented alternative, with performance strongly influenced by actuator saturation and reference shape.

This repository serves as a **baseline implementation** for vehicle lateral control studies and can be readily extended toward nonlinear tire models, gain scheduling, MPC, or preview-based reference generation.

---

**Author: Ayushman Mishra**

**LinkedIn:** https://www.linkedin.com/in/aymisxx/

**GitHub:** https://github.com/aymisxx

---