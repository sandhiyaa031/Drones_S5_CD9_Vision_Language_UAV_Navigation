# Mathematical Formulation of the GPS-Denied Navigation Stack

## 1. Visual Odometry & Epipolar Geometry

### 1.1 Essential Matrix & Epipolar Constraint
For a calibrated camera with intrinsic matrix $\mathbf{K}$, normalized image coordinates $\mathbf{x} = \mathbf{K}^{-1} \mathbf{p}$ satisfy the epipolar constraint between two consecutive frames:

$$\mathbf{x}_2^T \mathbf{E} \mathbf{x}_1 = 0$$

where the **Essential Matrix** $\mathbf{E} \in \mathbb{R}^{3 \times 3}$ is defined by the relative rotation $\mathbf{R} \in \text{SO}(3)$ and translation vector $\mathbf{t} \in \mathbb{R}^3$:

$$\mathbf{E} = [\mathbf{t}]_\times \mathbf{R} = \begin{bmatrix} 0 & -t_z & t_y \\ t_z & 0 & -t_x \\ -t_y & t_x & 0 \end{bmatrix} \mathbf{R}$$

### 1.2 5-Point Algorithm & Singular Value Decomposition
$\mathbf{E}$ has 5 degrees of freedom and singular values $[\sigma, \sigma, 0]$. Using Nister's 5-point algorithm with RANSAC:
1. Estimate $\mathbf{E}$ from 2D point correspondences.
2. Compute SVD: $\mathbf{E} = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^T$, where $\mathbf{\Sigma} = \text{diag}(1, 1, 0)$.
3. Four possible $(\mathbf{R}, \mathbf{t})$ candidate solutions are recovered using:
   $$\mathbf{W} = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}, \quad \mathbf{R}_1 = \mathbf{U}\mathbf{W}\mathbf{V}^T, \quad \mathbf{R}_2 = \mathbf{U}\mathbf{W}^T\mathbf{V}^T, \quad [\mathbf{t}]_\times = \mathbf{U} \mathbf{Z} \mathbf{U}^T$$
4. **Cheirality check**: Select the unique $(\mathbf{R}, \mathbf{t})$ pair that places triangulated 3D points in front of both cameras ($Z > 0$).

---

## 2. 15-State Error-State Extended Kalman Filter (ES-EKF)

### 2.1 State Representation
The state is divided into **Nominal State** $\mathbf{x}$ and **Error State** $\delta \mathbf{x} \in \mathbb{R}^{15}$:

$$\mathbf{x} = \begin{bmatrix} \mathbf{p} \\ \mathbf{v} \\ \mathbf{q} \\ \mathbf{b}_a \\ \mathbf{b}_g \end{bmatrix} \in \mathbb{R}^3 \times \mathbb{R}^3 \times \mathbb{H} \times \mathbb{R}^3 \times \mathbb{R}^3, \quad \delta \mathbf{x} = \begin{bmatrix} \delta \mathbf{p} \\ \delta \mathbf{v} \\ \delta \boldsymbol{\theta} \\ \delta \mathbf{b}_a \\ \delta \mathbf{b}_g \end{bmatrix} \in \mathbb{R}^{15}$$

### 2.2 Continuous-Time IMU Kinematics
Given measured acceleration $\mathbf{a}_m$ and angular rate $\boldsymbol{\omega}_m$:

$$\dot{\mathbf{p}} = \mathbf{v}$$
$$\dot{\mathbf{v}} = \mathbf{R}(\mathbf{q}) (\mathbf{a}_m - \mathbf{b}_a - \mathbf{w}_a) + \mathbf{g}$$
$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega}_m - \mathbf{b}_g - \mathbf{w}_g \end{bmatrix}$$
$$\dot{\mathbf{b}}_a = \mathbf{w}_{ba}, \quad \dot{\mathbf{b}}_g = \mathbf{w}_{bg}$$

### 2.3 Discrete-Time Error Covariance Propagation
The discrete error-state Jacobian $\mathbf{F}_x$ and process noise $\mathbf{Q}_k$:

$$\mathbf{F}_x = \begin{bmatrix} 
\mathbf{I}_3 & \mathbf{I}_3 \Delta t & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{I}_3 & -\mathbf{R}(\mathbf{q}) [\mathbf{a}_m - \mathbf{b}_a]_\times \Delta t & -\mathbf{R}(\mathbf{q}) \Delta t & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{I}_3 - [\boldsymbol{\omega}_m - \mathbf{b}_g]_\times \Delta t & \mathbf{0} & -\mathbf{I}_3 \Delta t \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3 & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_3
\end{bmatrix}$$

$$\mathbf{P}_k = \mathbf{F}_x \mathbf{P}_{k-1} \mathbf{F}_x^T + \mathbf{Q}_k$$

### 2.4 Measurement Update (Visual Odometry & Rangefinder)
For innovation residual $\mathbf{y} = \mathbf{z} - h(\mathbf{x})$:
1. **Innovation Covariance**: $\mathbf{S} = \mathbf{H} \mathbf{P} \mathbf{H}^T + \mathbf{R}$
2. **Mahalanobis Gate**: $d_M^2 = \mathbf{y}^T \mathbf{S}^{-1} \mathbf{y} < \chi^2_{\text{threshold}}$
3. **Kalman Gain**: $\mathbf{K} = \mathbf{P} \mathbf{H}^T \mathbf{S}^{-1}$
4. **State Injection**: $\delta \mathbf{x} = \mathbf{K} \mathbf{y} \implies \mathbf{p} \leftarrow \mathbf{p} + \delta \mathbf{p}, \; \mathbf{q} \leftarrow \mathbf{q} \otimes \exp(\delta \boldsymbol{\theta} / 2)$
5. **Joseph-Form Covariance Update**: $\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H}) \mathbf{P} (\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T$

---

## 3. Trajectory Evaluation Metrics (ATE & RPE)

### 3.1 Absolute Trajectory Error (ATE)
Measures global consistency between estimated trajectory $\mathbf{P}_{\text{est}, 1:N}$ and ground truth $\mathbf{P}_{\text{gt}, 1:N}$:

$$\mathbf{F}_i = \mathbf{P}_{\text{gt}, i}^{-1} \mathbf{S} \mathbf{P}_{\text{est}, i}$$
$$\text{ATE RMSE} = \sqrt{\frac{1}{N} \sum_{i=1}^N \|\text{trans}(\mathbf{F}_i)\|^2}$$

### 3.2 Drift Rate Percentage
$$\text{Drift Rate} = \left( \frac{\text{ATE RMSE}}{\sum_{i=1}^{N-1} \|\mathbf{p}_{\text{gt}, i+1} - \mathbf{p}_{\text{gt}, i}\|} \right) \times 100\%$$
Target benchmark for Member 4 stack: **$< 3.0\%$**.
