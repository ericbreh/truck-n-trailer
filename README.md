# Truck-N-Trailer

# Goal

Reverse park a model truck and trailer into a parking spot.

# Physical Setup

* Two independent rear drive wheels, each with a brushed DC motor and encoder.
* Front caster balls.
* Single axle trailer, hitch point on top of trucks rear axle.
* Onboard esp32 controlling motors.
* Birds eye view mounted webcam with ArUco markers on truck, trailer, and parking spot.

# CV

Video from the aerial camera goes to laptop running python script with OpenCV. Script will find coordinate positions of the ArUco markers and calculate $(x, y, \theta_t, \theta_l)$.

# MPC

### Kinematic Model

#### State

$$
q=\begin{bmatrix}x & y & \theta_t & \theta_l & v & \omega\end{bmatrix}^\top,
\quad
u=\begin{bmatrix}a & \alpha\end{bmatrix}^\top
$$

#### Dynamics

$$
\begin{aligned}
\dot{x} &= v \cos(\theta_t) \\
\dot{y} &= v \sin(\theta_t) \\
\dot{\theta}_t &= \omega \\
\dot{\theta}_l &= \frac{v}{d} \sin(\theta_t - \theta_l) \\
\dot{v} &= a \\
\dot{\phi} &= \alpha
\end{aligned}
$$
* $d$: Length of trailer from hitch to wheels.

### Input

* Current state:
 	* Position $(x, y)$ of trucks rear axle
 	* Orientation $(\theta_t, \theta_l)$
 	* Linear velocity $(v)$
 	* Angular velocity $(\omega)$
* Desired state

### Output

* Linear acceleration $(a)$
* Angular acceleration $(\alpha)$

### Cost function

* Squared difference between current and desired state at end of planning horizon.
* Control effort.

### Constraints

* Linear acceleration and angular acceleration limits.
* Velocity limit.
* Jackknife constraint.

# Motor Control

### Body to Wheel Conversion

$$
\begin{aligned}
v_{desired}(k) = v(k-1) + a(k) \cdot dt \\
\omega_{desired}(k) = \omega(k-1) + \alpha(k) \cdot dt \\
v_{L\_target} = v_{desired} - \frac{\omega_{desired} \cdot W}{2} \\
v_{R\_target} = v_{desired} + \frac{\omega_{desired} \cdot W}{2} \\
\end{aligned}
$$

### Left Motor PID
* $error_L = v_{L\_target} - v_{L\_measured}$
* $PWM_L = Kp \cdot error_L + Ki \cdot \int error_L dt + Kd \cdot \frac{d(error_L)}{dt}$

### Right Motor PID
* $error_R = v_{R\_target} - v_{R\_measured}$
* $PWM_R = Kp \cdot error_R + Ki \cdot \int error_R dt + Kd \cdot \frac{d(error_R)}{dt}$

# Possible additions

* Add constraints to not hit the sides of the parking spot
* Run mpc on the esp instead of laptop
