# Inverted Pendulum Balancing with PID Control

A MATLAB simulation of a **single inverted pendulum on a cart** using PID control for pendulum stabilization and an outer-loop controller for cart centering.

The project demonstrates how feedback control can stabilize an inherently unstable inverted pendulum while simultaneously keeping the cart near the center of its track.

## Overview

An inverted pendulum is a classic control systems problem in which a pendulum must be maintained in its unstable upright equilibrium position by moving the cart beneath it.

This implementation uses a **cascaded control architecture** consisting of:

* An **outer-loop cart position controller** that determines the desired pendulum angle.
* An **inner-loop PID controller** that balances the pendulum around that desired angle.
* Control-force saturation and integral anti-windup for more realistic controller behavior.
* A MATLAB animation showing the cart and pendulum response in real time.
* Post-simulation plots for analyzing system performance.

The pendulum angle convention is:

* `θ = 0°` → pendulum is upright
* `θ > 0°` → pendulum tilts to the right
* `θ < 0°` → pendulum tilts to the left

## Control Architecture

The controller uses two feedback loops:

```text
          Desired Cart
          Position = 0
               |
               v
      +------------------+
      |   Outer Loop     |
      | Cart Centering   |
      +------------------+
               |
               | θ_ref
               v
      +------------------+
      |    Inner Loop    |
      |  PID Controller  |
      +------------------+
               |
               | Force u
               v
      +------------------+
      | Cart + Pendulum  |
      |     Dynamics     |
      +------------------+
          |          |
          | x        | θ
          +----------+
           Feedback
```

### Outer Loop — Cart Centering

The outer loop monitors the cart position and velocity and generates a desired pendulum angle:

```matlab
theta_ref = -Kx*cart_pos - Kxd*cart_vel;
```

If the cart moves away from the center, the controller commands a small pendulum lean in the opposite direction. The inner controller then moves the cart underneath the pendulum, gradually returning the cart toward `x = 0`.

The desired angle is limited to ±10° to prevent the centering controller from requesting excessive pendulum angles.

Controller gains:

```matlab
Kx  = 0.20;
Kxd = 0.47;
```

## Inner Loop — PID Pendulum Controller

The inner loop controls the pendulum angle using a PID controller.

The tracking error is defined as:

```text
error = θ - θ_ref
```

The applied cart force is calculated using:

```text
u = Kp × error + Ki × integral(error) + Kd × error_dot
```

The controller gains used in the simulation are:

| Gain | Value |
| ---- | ----: |
| Kp   |    57 |
| Ki   |   1.2 |
| Kd   |    20 |

The integral error is clamped to reduce integral windup, while the controller output is limited to:

```text
-30 N ≤ u ≤ 30 N
```

## System Model

The simulation uses a teaching-oriented dynamic model of the cart-pendulum system.

### Cart Dynamics

The cart acceleration is approximated by:

```text
ẍ = u / (M + m)
```

where:

* `u` = controller force
* `M` = cart mass
* `m` = pendulum mass

### Pendulum Dynamics

The pendulum angular acceleration is modeled as:

```text
θ̈ = (g/l)θ - (1/l)ẍ
```

The first term represents the destabilizing effect of gravity around the upright equilibrium. The second represents the effect of cart acceleration on the pendulum.

The simulation numerically integrates these accelerations to update the four system states:

```text
x₁ = Cart position
x₂ = Cart velocity
x₃ = Pendulum angle
x₄ = Pendulum angular velocity
```

## System Parameters

| Parameter                |     Value |
| ------------------------ | --------: |
| Gravity                  | 9.81 m/s² |
| Cart mass                |    1.0 kg |
| Pendulum mass            |    0.2 kg |
| Pendulum length          |     0.5 m |
| Simulation timestep      |   0.005 s |
| Simulation duration      |      12 s |
| Initial cart position    |       0 m |
| Initial cart velocity    |     0 m/s |
| Initial pendulum angle   |        4° |
| Initial angular velocity |   0 rad/s |
| Maximum control force    |     ±30 N |

The simulation therefore begins with the pendulum displaced **4° from the upright position**, requiring the controller to stabilize it.

## Simulation

During execution, MATLAB displays an animated representation of the system.

The animation includes:

* Cart
* Wheels
* Pendulum rod
* Pendulum mass
* Ground reference
* Center reference line

Real-time information is also displayed for:

```text
Time
Cart Position
Pole Angle
Desired Angle
Control Force
```

This makes it possible to visually observe how the controller moves the cart underneath the pendulum to maintain balance.

## Performance Plots

After the animation finishes, the program generates four plots for evaluating the controller.

### Cart Position

Shows whether the outer-loop controller successfully keeps the cart near:

```text
x = 0 m
```

### Pendulum Angle

Compares the actual pendulum angle with the desired angle generated by the cart-centering controller.

### Cart Velocity

Shows how quickly the cart moves while balancing and repositioning the pendulum.

### Control Force

Shows the force generated by the PID controller throughout the simulation.

These plots make it easier to evaluate stability, transient response, cart drift, and controller effort.

## Requirements

* MATLAB
* No additional MATLAB toolboxes are required for the core simulation.

## Running the Project

1. Clone or download this repository.

2. Open MATLAB.

3. Navigate to the project directory.

4. Open:

```text
cart_at_center_balancing_pole.m
```

5. Run the script.

MATLAB will first display the animated inverted pendulum simulation. Once the simulation completes, a second figure will display the system performance plots.

## Project Structure

```text
.
├── cart_at_center_balancing_pole.m
└── README.md
```

## Key Concepts Demonstrated

This project demonstrates several fundamental robotics and control concepts:

* PID control
* Cascaded control loops
* Feedback control
* Inverted pendulum stabilization
* Cart position regulation
* Dynamic system modeling
* Numerical integration
* Actuator saturation
* Integral anti-windup
* MATLAB simulation and visualization

## Future Improvements

The project can be extended by:

* Implementing the full nonlinear cart-pole equations of motion.
* Comparing PID control with LQR control.
* Implementing state-space control.
* Adding disturbance rejection tests.
* Introducing measurement noise.
* Using a Kalman filter for state estimation.
* Automatically evaluating settling time and overshoot.
* Implementing the controller on a physical cart-pole system.

## Author

**Nnenna Onwuka**

Mechanical Engineering | Robotics | Controls | Machine Learning

GitHub: `Nne1509`
