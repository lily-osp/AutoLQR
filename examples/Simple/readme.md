# Simple Example

This example demonstrates basic LQR control for a single-state system, such as position control with one input.

## Overview

The example controls a simulated single-state system (position only) with natural decay and shows how to:
- Initialize the AutoLQR controller
- Set up system matrices (A, B) and cost matrices (Q, R)
- Compute optimal gains
- Run a control loop with state feedback

## System Model

- **State**: Position (1 state)
- **Control**: Single input (motor speed/force)
- **Dynamics**: Position with friction/decay factor (A = 0.95)

## Features

1. **Basic LQR Control**: Demonstrates the fundamental LQR workflow
2. **Feedforward Gain**: Calculates feedforward compensation for steady-state tracking
3. **Convergence Estimation**: Estimates time to reach target position
4. **Controllability Check**: Verifies the system is controllable before operation
5. **Adaptive Control Toggle**: Optional gain boost for large errors
6. **Interactive Commands**: Serial commands to toggle features and change setpoints

## Serial Commands

- `a` - Toggle adaptive control (gain boost for large errors)
- `f` - Toggle feedforward compensation
- `n` - Set new random setpoint

## Tuning

Adjust the following to change controller behavior:
- `Q[0]` - Position error cost (higher = faster response)
- `R[0]` - Control effort cost (higher = smoother but slower)
- `A[0]` - System decay factor (models friction/damping)
- `B[0]` - Control input effectiveness

## Usage

1. Upload to Arduino
2. Open Serial Monitor at 115200 baud
3. Observe position tracking to target
4. Use commands to experiment with different modes
