# Truck-N-Trailer

# Goal

Reverse park a model truck and trailer into a parking spot.

# Physical Setup

* 3d printed truck and trailer. rear wheel drive with dc motor and steering linkage with servo.
* Onboard esp32 controlling motors.
* Birds eye view mounted webcam.
* Visually defined parking space, ArUco markers on top of truck and trailer.
* Encoders on steering and axle to send current steering angle and speed to esp.

# CV

Video from the aerial camera goes to laptop running python script with OpenCV. Script will find coordinate positions of the ArUco markers, calculate $(x, y, \theta_t, \theta_l)$ and stream to esp via usbc cable or UDP over wifi

# MPC

### Kinematic Model

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

* Discrete time kinematic model
* Current state vector:
  * Position $(x, y)$ of trucks rear axle
  * Orientation $(\theta_t, \theta_l)$
  * Velocity $(v)$
  * Steering angle $(\phi)$
  * Desired position/orientation
  * Location of obstacles (sides of parking spot)

### Output

* Acceleration $(a)$
* Steering rate $(\dot{\phi})$

### Cost function

* Squared difference between current and desired state (in parking spot)
* Control effort (encourage smooth control)

### Constraints

* Steering angle limits
* Acceleration and steering rate limits
* Velocity limit
* Jackknife constraint
* Obstacle avoidance (dont hit sides of parking spot)

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
