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

## Mathematical Modeling

This project models vehicle lateral motion using a **linear bicycle model** with **front steering angle** as the control input. The complete control-oriented formulation combines:

1. a linearized lateral-yaw vehicle model,
2. path-following error dynamics,
3. curvature-based desired-state generation, and
4. feedback control laws built on those dynamics.

### 1) States, Input, and Outputs

The vehicle states are chosen as lateral velocity and yaw rate:

$$
x = \begin{bmatrix} V_y \\ r \end{bmatrix}
$$

where:

- $V_y$ is the lateral velocity of the vehicle body,
- $r$ is the yaw rate.

The control input is the front steering angle:

$$
u = \delta
$$

The continuous-time state-space model is written as:

$$
\dot{x} = A x + B u
$$

$$
y = Cx + Du
$$

The report defines the output matrices as:

$$
C =
\begin{bmatrix}
0 & 1 \\
1 & 0
\end{bmatrix}
$$

$$
D =
\begin{bmatrix}
0 \\
0
\end{bmatrix}
$$

Thus, the output vector is:

$$
y =
\begin{bmatrix}
r \\
V_y
\end{bmatrix}
$$

which simply means the model output contains the same two state variables, reordered.

### 2) Governing Vehicle Dynamics

The modeling starts from planar force and moment balance for a four-wheel vehicle. The appendix lists the governing equations for longitudinal/lateral motion and yaw dynamics, together with the steering-induced force terms. These equations are then reduced into a linear control-oriented model for lateral stability analysis.

The final controller design uses the compact form:

$$
\dot{x} = A x + B u
$$

$$(x = [V_y \ \ r]^T)$$

### 3) State-Space Matrices

The report provides symbolic expressions for the system matrices in terms of vehicle parameters.

#### System matrix $A$

$$
A =
\begin{bmatrix}
-\dfrac{2(C_{\alpha f}+C_{\alpha r})}{V_x m} &
-\dfrac{2(C_{\alpha f}l_f - C_{\alpha r}l_r)}{V_x m} - V_x \\
-\dfrac{2(C_{\alpha f}l_f - C_{\alpha r}l_r)}{V_x I_z} &
-\dfrac{2(C_{\alpha f}l_f^2 + C_{\alpha r}l_r^2)}{V_x I_z}
\end{bmatrix}
$$

#### Input matrix $B$

$$
B =
\begin{bmatrix}
-\dfrac{2C_{\alpha f}}{m} \\
-\dfrac{2C_{\alpha f}l_f}{I_z}
\end{bmatrix}
$$

#### Output matrices $C$ and $D$

$$
C =
\begin{bmatrix}
0 & 1 \\
1 & 0
\end{bmatrix}
$$

$$
D =
\begin{bmatrix}
0 \\
0
\end{bmatrix}
$$

Here:

- $C_{\alpha f}$ and $C_{\alpha r}$ are the front and rear cornering stiffnesses,
- $l_f$ and $l_r$ are the distances from the center of gravity to the front and rear axles,
- $m$ is the vehicle mass,
- $I_z$ is the yaw moment of inertia,
- $V_x$ is the constant longitudinal speed.

These matrices are obtained by substituting the vehicle parameters into the linearized lateral-yaw dynamics.

### 4) Physical Parameters Used

The report uses the following vehicle parameters:

$$
C_{\alpha f} = 60000 \ \text{N/rad}, \qquad
C_{\alpha r} = 60000 \ \text{N/rad}
$$

$$
l_f = 1.4 \ \text{m}, \qquad
l_r = 1.65 \ \text{m}
$$

$$
V_x = 25 \ \text{m/s}
$$

$$
m = 1830 \ \text{kg}, \qquad
I_z = 2324 \ \text{kg}\cdot\text{m}^2
$$

Substituting these constants into the symbolic model yields the numerical state-space matrices used in simulation and controller design.

### 5) Path-Following Error Dynamics

To connect the vehicle body dynamics to path tracking, the report defines two path-following errors:

- $e_1$: lateral path error,
- $e_2$: yaw-angle / heading-related error.

Their dynamics are given by:

$$
\dot{e}_1 = v_x \sin(e_2) + v_y \cos(e_2)
$$

$$
\dot{e}_2 = r - \rho v_x
$$

where:

- $v_x$ is the longitudinal velocity,
- $v_y$ is the lateral velocity,
- $r$ is the yaw rate,
- $\rho$ is the road curvature.

These equations describe how the vehicle deviates from the desired path in both position and heading.

### 6) Desired State Generation

The desired signals used by the controllers are defined as:

$$
V_{y,\text{desired}} = 0
$$

$$
r_{\text{desired}} = \rho v_x - k_2(e_2 + k_1 e_1)
$$

This means:

- the desired lateral velocity is zero,
- the desired yaw rate is based on the road curvature term $\rho v_x$,
- and additional path-tracking correction is introduced through the error terms $e_1$ and $e_2$.

So the controller is designed not only to stabilize the vehicle, but also to regulate it relative to a desired curved path.

### 7) Feedforward Steering Input

The report also defines a feedforward steering input based on curvature:

$$
u_{\text{ff}} = L \rho,
\qquad
L = l_f + l_r
$$

This provides a baseline steering command from simple path geometry. In other words, for a desired road curvature $\rho$, the feedforward term supplies the nominal steering angle needed to follow that curvature before any feedback correction is applied.

### 8) Interpretation of $A$, $B$, $C$, and $D$

The matrices have clear physical meaning:

- **$A$** represents the internal lateral-yaw dynamics of the vehicle, including the coupling between lateral velocity and yaw rate.
- **$B$** captures how front steering angle influences lateral motion and yaw motion.
- **$C$** selects the output variables of interest from the state vector.
- **$D$** is zero because the chosen state-space model has no direct feedthrough from steering input to output.

This linear structure is the foundation used for controller design in the project.

### 9) Controllability, Observability, and Stability

Using the state-space model, the report computes the controllability and observability matrices and concludes that both are full rank. Therefore, the system is both controllable and observable, and the realization is minimal.

The report also applies a Lyapunov stability analysis and states that the system is asymptotically stable under the modeled conditions.

### 10) Modeling Summary

Overall, the project combines:

- a **two-state linear lateral-yaw vehicle model**,
- a **path-following error model**,
- a **curvature-based desired yaw-rate reference**,
- a **feedforward steering baseline**, and
- **feedback control design** based on the resulting states and errors.

This modeling framework enables a fair comparison of feedforward steering, LQR, and Sliding Mode Control for vehicle lateral stability under a shared road-curvature reference.

---

## Folder Structure

```text
VehicleLateralStability-Py/
  requirements.txt                    # dependencies
  README.md                           # this file/text
  VehicleLateralStability-Py.pdf      # pdf-output of the jupyter notebook implementation
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

![PPO Trajectory](results/03_yaw_compare.png) ![PPO Trajectory](results/04_vy_compare.png)

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
