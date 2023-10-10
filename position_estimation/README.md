# Vehicle Position Estimation with Extended Kalman Filter (EKF)
In this section, we try to estimate the position of vehicle along a trajectory based on the motion model and the available measurements using Extended Kalman filter as state estimator since the model equations are nonlinear.
The measurements are collected with a LIDAR sensor showing the landmarks in the environment nd also the bearing measurements. 

## Motion Model
The motion model allows us to use Kalman filter. To do so, we assume that the vehicle gets linear and angular velocity as inputs and outputs the 2D position of the vehicle. This model can written as following:

$$\begin{align}
\mathbf{x}_{k} &= \mathbf{x}_{k-1} + T
\begin{bmatrix}
\cos\theta_{k-1} &0 \\
\sin\theta_{k-1} &0 \\
0 &1
\end{bmatrix}
\left(
\begin{bmatrix}
v_k \\
\omega_k
\end{bmatrix}
+ \mathbf{w}_k
\right)
\, , \, \, \, \, \, \mathbf{w}_k = \mathcal{N}\left(\mathbf{0}, \mathbf{Q}\right)
\end{align}$$

where $\mathbf{x}_k = \left[ x \, y \, \theta \right]^T$ is the current state vector representing the current position of the vehicle, $v_k$ and $\omega_k$ are linear and angular velocity inputs, and $\mathbf{w}_k$ is the process noise with zero mean and covariance matrix of $Q$.

## Measurement Model
The measurement model representes the relationship between the vehicle pose and the LIDAR and bearing measurements, $\mathbf{y}^l_k = \left[r \, \phi \right]^T$, with the following equations:
## Introduction
-----

In this assignment you will recursively estimate the position of a vehicle along a trajectory using available measurements and a motion model. 

The vehicle is equipped with a very simple type of LIDAR sensor, which returns range and bearing measurements corresponding to individual landmarks in the environment. The global positions of the landmarks are assumed to be known beforehand. We will also assume known data association, that is, which measurment belong to which landmark.

## Motion and Measurement Models
-----

### Motion Model

The vehicle motion model recieves linear and angular velocity odometry readings as inputs, and outputs the state (i.e., the 2D pose) of the vehicle:

\begin{align}
\mathbf{x}_{k} &= \mathbf{x}_{k-1} + T
\begin{bmatrix}
\cos\theta_{k-1} &0 \\
\sin\theta_{k-1} &0 \\
0 &1
\end{bmatrix}
\left(
\begin{bmatrix}
v_k \\
\omega_k
\end{bmatrix}
+ \mathbf{w}_k
\right)
\, , \, \, \, \, \, \mathbf{w}_k = \mathcal{N}\left(\mathbf{0}, \mathbf{Q}\right)
\end{align}

- $\mathbf{x}_k = \left[ x \, y \, \theta \right]^T$ is the current 2D pose of the vehicle
- $v_k$ and $\omega_k$ are the linear and angular velocity odometry readings, which we use as inputs to the model

The process noise $\mathbf{w}_k$ has a (zero mean) normal distribution with a constant covariance $\mathbf{Q}$.

### Measurement Model

The measurement model relates the current pose of the vehicle to the LIDAR range and bearing measurements $\mathbf{y}^l_k = \left[r \, \phi \right]^T$.

$$\begin{align}
\mathbf{y}^l_k =
\begin{bmatrix}
\sqrt{(x_l - x_k - d\cos\theta_{k})^2 + (y_l - y_k - d\sin\theta_{k})^2} \\
arctan\left((y_l - y_k - d\sin\theta_{k})/(x_l - x_k - d\cos\theta_{k})\right) - \theta_k
\end{bmatrix}
+
\mathbf{n}^l_k
\, , \, \, \, \, \, \mathbf{n}^l_k = \mathcal{N}\left(\mathbf{0}, \mathbf{R}\right)
\end{align}$$

where $x_l$ and $y_l$ are the ground truth coordinates of the landmark $l$, $x_k$ and $y_k$ and $\theta_{k}$ are the current pose of the vehicle, $d$ is the known distance between robot center and LIDAR laser rangefinder.
The landmark measurement noise $\mathbf{n}^l_k$ has a normal distribution with zero mean and a constant covariance $\mathbf{R}$.

## Methodology Description
We use extended kalman filter for pose estimation of the vehicle as it tries to linearize the model at a specific working point while applying the prediction step to produce a state and a correction step to correct the pose estimates. In other words, in the prediction step, we use motion model and the inputs to find the current state of the vehicle considering the process noise with a constant variance. In the correction step, we use a LIDAR and bearing measurements to correc the state estimatins considering the measurement noise with a constant covariance matrix. 

