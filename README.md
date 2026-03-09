# Truck-N-Trailer

# Goal

Reverse park a model truck and trailer into a parking spot.

# Physical Setup

* 3d printed truck and trailer. rear wheel drive with dc motor and steering linkage with servo.
* Onboard esp32 controlling motors.
* Birds eye view mounted webcam.
* Visually defined parking space, ArUco markers on top of truck and trailer.
* Encoders on steering and axle.

# CV

Video from the aerial camera goes to laptop running python script with OpenCV. Script will find coordinate positions of the ArUco markers and calculate $(x, y, \theta_t, \theta_l)$.

# MPC

### Kinematic Model

State:
$$
q=\begin{bmatrix}x & y & \theta_t & \theta_l & v & \phi\end{bmatrix}^\top,
\quad
u=\begin{bmatrix}a & \dot{\phi}\end{bmatrix}^\top
$$
Dynamics:
$$
\begin{aligned}
\dot{x} &= v \cos(\theta_t) \\
\dot{y} &= v \sin(\theta_t) \\
\dot{\theta}_t &= \frac{v}{L} \tan(\phi) \\
\dot{\theta}_l &= \frac{v}{d} \sin(\theta_t - \theta_l) \\
\dot{v} &= a \\
\dot{\phi} &= \dot{\phi}
\end{aligned}
$$

### Input

* Current state:
  * Position $(x, y)$ of trucks rear axle
  * Orientation $(\theta_t, \theta_l)$
  * Velocity $(v)$
  * Steering angle $(\phi)$
* Desired state

### Output

* Acceleration $(a)$
* Steering rate $(\dot{\phi})$

### Cost function

* Squared difference between current and desired state at end of planning horizon
* Control effort

### Constraints

* Steering angle limits
* Acceleration and steering rate limits
* Velocity limit
* Jackknife constraint
* Obstacle avoidance

# PID Motor Control

### Velocity

```
v_desired(k) = v(k-1) + a(k) * dt
error = v_desired - v_measured
PWM_output = Kp*e + Ki*∫e dt + Kd*de/dt
```

### Steering

```
φ_desired(k) = φ(k-1) + φ̇(k) * dt
error = φ_desired - φ_measured
servo_PWM = Kp * error
```

# Possible additions

* Add constraints to not hit the sides of the parking spot
* Run mpc on the esp instead of laptop
