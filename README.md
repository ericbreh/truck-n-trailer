# Truck-N-Trailer

# Goal
Reverse park a model truck and trailer into a parking spot.

# Physical Setup
* esp32 https://learn.adafruit.com/adafruit-esp32-feather-v2/pinouts.
* Independent rear drive wheels, each with a brushed DC motor https://www.pololu.com/product/2215 and encoder https://www.pololu.com/product/3081.
* Motor driver https://www.pololu.com/product/2130
* Caster balls for front wheels.
* Single axle trailer, hitch point on top of trucks rear axle.
* Birds eye view mounted webcam with ArUco markers on truck, trailer, and parking spot.

# High Level Control
Reads $(x, y, \theta_t, \theta_l)$ from cv and $(v, \omega)$ from esp. Runs mpc over time horizon N=50 with dt=0.1s. Takes the first output $(a, \alpha)$ and applies forward Euler to calculate $v_{L\_target}$ and $v_{R\_target}$ which are sent to the esp at 10hz.

## CV
Video from the aerial camera goes to laptop running python script with OpenCV. 
Script will find coordinate positions of the ArUco markers and calculate $(x, y, \theta_t, \theta_l)$.

## MPC
### Kinematic Model

#### State:
$$
q=\begin{bmatrix}x & y & \theta_t & \theta_l & v & \omega\end{bmatrix}^\top,
\quad
u=\begin{bmatrix}a & \alpha\end{bmatrix}^\top
$$
#### Dynamics:
$$
\begin{aligned}
\dot{x} &= v \cos(\theta_t) \\
\dot{y} &= v \sin(\theta_t) \\
\dot{\theta}_t &= \omega \\
\dot{\theta}_l &= \frac{v}{d} \sin(\theta_t - \theta_l) \\
\dot{v} &= a \\
\dot{\omega} &= \alpha
\end{aligned}
$$
- $d$: Length of trailer from hitch to wheels.
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
* Terminal cost - squared difference between current and desired state at end of planning horizon.
* Stage cost - squared difference between current and desired position on each time step.
* Control effort.
### Constraints
* Linear acceleration and angular acceleration limits.
* Velocity limit.
* Jackknife constraint.
### Body to Wheel Conversion:
$$
\begin{aligned}
v_{desired}(k) = v(k-1) + a(k) \cdot dt \\
\omega_{desired}(k) = \omega(k-1) + \alpha(k) \cdot dt \\
v_{L\_target} = v_{desired} - \frac{\omega_{desired} \cdot W}{2} \\
v_{R\_target} = v_{desired} + \frac{\omega_{desired} \cdot W}{2} \\
\end{aligned}
$$

# Firmware
## Tasks
1. Control
	- Higher priority
	- Triggered by GPTimer interrupt (100hz)
	- Reads encoders, calculated PID, sends motor command
	- $error = v_{target} - v_{measured}$
	- $PWM = Kp \cdot error + Ki \cdot \int error dt + Kd \cdot \frac{d(error)}{dt}$
2. Communication
	- Lower priority
	- Triggered by UART when it sees the end of a command
	- Parses the command, updates the target rpm variables

# GUI
 - Send manual commands
 - Auto park button
 - Display CV, trajectory, if auto parking is feasible, current readouts from sensors

# Potential Improvements
- Sensor on hitch angle. Send interrupt when jackknife to kill motors
- Pinning tasks to cores
- Constrain MPC to not hit obstacles
