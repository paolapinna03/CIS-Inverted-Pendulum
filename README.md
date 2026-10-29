# Inverted Pendulum Control

Control-system design and simulation for an **inverted pendulum** and a **cart-pole swing-up problem**, implemented in MATLAB and Simulink.

The project develops progressively from linear feedback control and state estimation to nonlinear trajectory optimization. It includes **discrete-time LQR control**, **observer design**, model discretization, actuator constraints, and an **iterative Linear Quadratic Regulator (iLQR)** for cart-pole swing-up.

## Project Overview

The inverted pendulum is a classical unstable control problem: without active feedback, the upright equilibrium cannot be maintained. This repository studies the problem in two stages.

```text
Part I
Inverted pendulum
      ↓
linearization
      ↓
discretization
      ↓
LQR feedback control
      ↓
state observer
      ↓
Simulink simulation

Part II
Cart-pole system
      ↓
nonlinear dynamics
      ↓
trajectory linearization
      ↓
iLQR optimization
      ↓
swing-up trajectory
      ↓
feedback gains + constrained simulation
```

The project therefore covers both **local stabilization around an equilibrium** and **nonlinear trajectory optimization over a finite horizon**.

## Part I — Inverted Pendulum Stabilization

The first part considers an actuated pendulum with state

```text
x = [theta, omega]^T
```

where `theta` is the angular position and `omega` the angular velocity.

The workflow implemented in `pendulum_script.m` is:

1. define the physical and model parameters;
2. linearize the pendulum dynamics around the upright equilibrium;
3. convert the continuous-time state-space model to discrete time using zero-order hold;
4. design a discrete-time **Linear Quadratic Regulator (LQR)**;
5. design a **state observer** using pole placement;
6. simulate the closed-loop system in Simulink.

### Discrete-time control

The continuous model is represented as

```text
x_dot = A_c x + B_c u
```

and discretized with sampling period

```text
Ts = 0.02 s
```

using MATLAB's `c2d` function.

The feedback law is obtained with

```matlab
K = dlqr(Ad, Bd, Q, R);
```

with the controller acting on regularly sampled measurements and the input applied through a zero-order hold.

### State estimation

Only the pendulum angle is directly measured in the observer setup:

```matlab
C = [1 0];
```

A discrete observer gain is obtained through pole placement:

```matlab
L = place(Ad', C', [0.5 0.6])';
```

This allows the angular velocity to be reconstructed from the measured output and the system model.

### Model uncertainty

The code keeps separate parameter structures for the **physical system** and the **controller model**. This makes it possible to study how the controller and observer behave when the model parameters differ from the simulated plant.

## Part II — Cart-Pole Swing-Up with iLQR

The second part extends the problem to a nonlinear cart-pole system with state

```text
x = [p, v, theta, omega]^T
```

where:

- `p` is the cart position;
- `v` is the cart velocity;
- `theta` is the pendulum angle;
- `omega` is the angular velocity.

The system starts with the pendulum hanging downward and the objective is to generate a control trajectory that brings it close to the unstable upright configuration.

## Nonlinear Trajectory Optimization

The swing-up controller is implemented with **iterative LQR (iLQR)**.

The algorithm repeatedly:

1. simulates a nominal trajectory using the nonlinear dynamics;
2. linearizes the dynamics along the current trajectory;
3. computes a backward LQR pass;
4. produces a new candidate trajectory through a forward pass;
5. performs a line search over the control update;
6. accepts the update only if it decreases the trajectory cost;
7. repeats until convergence or until no improving step is found.

The main implementation is contained in:

```text
Project_Part2/ilqr.m
```

with supporting functions for the backward pass, forward rollout, trajectory linearization, discrete dynamics and cost evaluation.

## Cost Function

The trajectory optimization uses a finite-horizon quadratic objective of the form

```text
J = sum(x_k^T Q x_k + u_k^T R u_k) + x_N^T Q_f x_N
```

with a relatively small running-state penalty and a stronger terminal cost to encourage the system to finish near the desired upright state.

The implementation tracks the objective across accepted iLQR iterations through `costHistory`, which makes convergence directly inspectable.

## Control Constraints

The swing-up problem explicitly includes actuator saturation:

```text
|u| <= Fmax
```

with

```text
Fmax = 20 N
```

in the cart-pole setup.

The initial control sequence and every forward-pass update are saturated to remain within the actuator limits.

The setup script also checks quantities such as:

- maximum applied force;
- maximum cart displacement;
- final cart position and velocity;
- final pendulum angle and angular velocity.

## Local LQR Stabilization

In addition to the nonlinear swing-up trajectory, the cart-pole setup derives a linearized continuous-time model around the upright configuration and converts it to discrete time.

A discrete LQR gain is then computed using weighted penalties on cart position, cart velocity, pendulum angle and angular velocity.

This provides a local feedback controller around the unstable equilibrium and complements the global swing-up trajectory produced by iLQR.

## Simulation

The project uses **Simulink** models for closed-loop simulation and visualization.

Main models include:

```text
actuated_pendulum.slx
Project_Part2/cartpole.slx
```

The MATLAB scripts configure the model parameters, controller gains and initial conditions before calling the simulations.

## Repository Structure

```text
.
├── pendulum_script.m              # linear model, discretization, LQR and observer
├── observer_design.mlx            # observer-design analysis
├── pendulum_control.mlx           # pendulum control analysis
├── actuated_pendulum.slx          # Simulink model for the actuated pendulum
├── run_simulator.m                # simulation utility
├── Question2PART1/                # additional Part I files
│
├── Project_Part2/
│   ├── setup_cartpole.m           # cart-pole parameters and experiment setup
│   ├── cartpole_dynamics.m        # nonlinear cart-pole dynamics
│   ├── discrete_step.m            # discrete nonlinear dynamics step
│   ├── linearize_trajectory.m     # trajectory-dependent linearization
│   ├── backward_lqr.m             # iLQR backward pass
│   ├── forward_pass.m             # nonlinear rollout and line-search candidate
│   ├── ilqr.m                     # iterative LQR optimization
│   ├── controller.m               # control logic
│   ├── experiments_ilqr_4.m       # iLQR experiments
│   └── cartpole.slx               # Simulink cart-pole model
│
└── Evaluation 1.2.2 .pdf          # project evaluation material
```

## Running Part I

Open MATLAB in the repository root and run:

```matlab
pendulum_script
```

The script defines the physical parameters, builds the discrete state-space model, computes the LQR and observer gains, and launches the `actuated_pendulum` Simulink model.

## Running Part II

Move to the second project directory and run:

```matlab
cd Project_Part2
setup_cartpole
```

The script initializes the nonlinear cart-pole model, computes the local LQR controller, optimizes the swing-up trajectory with iLQR, plots the resulting state/control trajectories and launches the Simulink simulation.

## Technology Stack

- **MATLAB**
- **Simulink / Simscape**
- **State-space modeling**
- **Discrete-time control**
- **Linear Quadratic Regulation (LQR)**
- **State observers**
- **Pole placement**
- **Nonlinear dynamics**
- **Iterative LQR (iLQR)**
- **Trajectory optimization**
- **Constrained optimal control**

## Control Perspective

The project illustrates the difference between two complementary control settings.

For small deviations around the upright equilibrium, a linearized model and LQR controller provide an efficient stabilization strategy. The swing-up task, however, requires reasoning over the full nonlinear dynamics and a finite trajectory, motivating the use of iLQR.

The repository therefore combines **classical state-space control, state estimation and nonlinear optimal control** within the same inverted-pendulum framework.
