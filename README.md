# Spacecraft Attitude Control & Bayesian Optimization Tuning

This project implements a high-fidelity simulation of a rigid-body spacecraft in Low Earth Orbit (LEO) and utilizes **Bayesian Optimization** to autonomously tune a Proportional-Derivative (PD) control system. The simulation accounts for complex aerodynamic disturbances based on spacecraft geometry and orbital conditions.

## 1. Mathematical Modeling of Dynamics

### Rotational Kinematics (Quaternions)
To avoid the mathematical singularity known as "Gimbal Lock," orientation is represented using unit quaternions $\mathbf{q} = [q_w, q_x, q_y, q_z]^T$. The time evolution of the attitude is governed by:

$$\dot{\mathbf{q}} = \frac{1}{2} \begin{bmatrix} 0 & -\omega_x & -\omega_y & -\omega_z \\ \omega_x & 0 & \omega_z & -\omega_y \\ \omega_y & -\omega_z & 0 & \omega_x \\ \omega_z & \omega_y & -\omega_x & 0 \end{bmatrix} \mathbf{q}$$

where $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^T$ is the angular velocity in the body frame.

### Rigid Body Dynamics
The rotational motion is dictated by Euler’s equations for a rigid body:

$$\mathbf{I} \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{I} \boldsymbol{\omega}) = \boldsymbol{\tau}_{aero} + \boldsymbol{\tau}_{ctrl}$$

Where:
- $\mathbf{I}$ is the **Inertia Tensor**.
- $\boldsymbol{\tau}_{aero}$ represents environmental disturbance torques.
- $\boldsymbol{\tau}_{ctrl}$ is the applied control torque.

## 2. Environmental Disturbance: Aerodynamic Torque
The simulation calculates torque based on a 6-face geometry model. For each face $i$, the force $\mathbf{F}_i$ and resulting torque $\boldsymbol{\tau}_i$ are:

$$\mathbf{F}_i = -\frac{1}{2} \rho v^2 C_d (A_i \cos \theta_i) \mathbf{\hat{v}}$$

$$\boldsymbol{\tau}_{aero} = \sum_{i=1}^{6} \mathbf{r}_i \times \mathbf{F}_i$$

where $\rho$ is atmospheric density, $v$ is orbital velocity, $A_i$ is face area, and $\mathbf{r}_i$ is the vector from the center of mass to the center of pressure of face $i$.

## 3. Control Strategy: PD Feedback
The controller aims to drive the spacecraft to the identity quaternion $[1, 0, 0, 0]^T$. The control torque is defined as:

$$\boldsymbol{\tau}_{ctrl} = -K_p \mathbf{q}_{vec} - K_d \boldsymbol{\omega}$$

- **Proportional Term ($K_p$):** Acts as a virtual spring. We use the vector part of the quaternion $\mathbf{q}_{vec} = [q_x, q_y, q_z]^T$ as the 3D error signal.
- **Derivative Term ($K_d$):** Acts as a virtual damper (friction) to dissipate kinetic energy and prevent oscillation.

## 4. Bayesian Optimization Tuning
Instead of manual gain scheduling, the project uses Bayesian Optimization to find the optimal $K_p$ and $K_d$ that minimize a cost function $J$ (total integrated attitude error).

### The Surrogate Model (Gaussian Process)
The algorithm treats the cost function as a Gaussian Process (GP). For a set of tested gains $\mathbf{X}$ and results $\mathbf{y}$, the predicted mean $\mu$ and uncertainty $\sigma$ at a new point $\mathbf{x}_*$ are:

$$\mu(\mathbf{x}_*) = \mathbf{k}_*^T (\mathbf{K} + \sigma_n^2 \mathbf{I})^{-1} \mathbf{y}$$

$$\sigma^2(\mathbf{x}_*) = k(\mathbf{x}_*, \mathbf{x}_*) - \mathbf{k}_*^T (\mathbf{K} + \sigma_n^2 \mathbf{I})^{-1} \mathbf{k}_*$$

The **Kernel Function** $k(\mathbf{x}, \mathbf{x}')$ (typically Squared Exponential) defines how the "influence" of a test point spreads through the search space.

### The Acquisition Function (Expected Improvement)
The algorithm decides where to sample next by maximizing the Expected Improvement (EI):

$$EI(\mathbf{x}) = (y^+ - \mu(\mathbf{x})) \Phi(Z) + \sigma(\mathbf{x}) \phi(Z)$$

where $Z = \frac{y^+ - \mu(\mathbf{x})}{\sigma(\mathbf{x})}$.

- **Exploitation:** $(y^+ - \mu(\mathbf{x})) \Phi(Z)$ targets areas with low predicted cost.
- **Exploration:** $\sigma(\mathbf{x}) \phi(Z)$ targets areas with high uncertainty (the "fog").

## 5. Summary of Dynamic Regimes
The simulator identifies four distinct physical states based on geometry and mass distribution:

| Geometry Type | Dominant State | Physical Driver |
| :--- | :--- | :--- |
| **Symmetric / Low Offset** | Stable Libration | Restoring Torque (The Spring) |
| **Intermediate Axis Heavy** | Chaotic Tumbling | Inertia Cross-Coupling (The Saddle) |
| **Asymmetric (Anemometer)** | Constant Acceleration | Net Work Extraction (The Motor) |
| **High Spin / Max Axis** | Precessional Beats | Gyroscopic Stiffness (The Gyro) |

### Case Studies
1.  **Stable Weather Vane:** High symmetry with a stabilizing $X$-offset ($off_x = -3$). Acts as a restoring "spring" around the velocity vector.
2.  **Chaotic Tumbling:** Asymmetric inertia ($a=10, b=2, c=5$) with offset center of pressure, leading to non-periodic motion and strange attractors.
3.  **Constant Acceleration (Windmill):** Uses a large paddle offset ($paddle_{off} = 25$) to extract work from the flow, causing continuous angular acceleration.
4.  **Precessional Beats:** Highly symmetric large bus ($a, b, c = 100$) where conservation of momentum causes energy to "beat" between axes.