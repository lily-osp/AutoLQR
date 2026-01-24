# Intermediate Example

This example demonstrates a two-state system (position and velocity) with multiple control modes and performance tracking.

## Overview

Controls a simulated position-velocity system with features including:
- Multiple control modes (normal, precise, efficient, smooth)
- Disturbance rejection testing
- Performance metrics tracking
- Feedforward compensation

## System Model

- **States**: Position and velocity (2 states)
- **Control**: Single input
- **Dynamics**: Discrete-time model with velocity damping

```
State: [position, velocity]
A = [1.0, 0.1]    // Position integrates velocity
    [0.0, 0.98]   // Velocity with slight damping

B = [0.005]       // Small direct effect on position
    [0.1]         // Primary effect on velocity
```

## Control Modes

The example includes four pre-configured control modes:

| Mode | Q (position) | Q (velocity) | R | Behavior |
|------|-------------|--------------|---|----------|
| Normal | 10.0 | 1.0 | 0.1 | Balanced response |
| Precise | 30.0 | 1.0 | 0.1 | Tighter position tracking |
| Efficient | 5.0 | 0.5 | 0.1 | Lower energy usage |
| Smooth | 10.0 | 1.0 | 0.5 | Gentler control actions |

## Features

1. **Mode Switching**: Change control behavior on-the-fly by recomputing gains
2. **Disturbance Testing**: Apply velocity disturbances to test rejection
3. **Performance Metrics**: Tracks average error, max error, and oscillation count
4. **Expected Cost**: Displays LQR cost function value

## Serial Commands

- `d` - Apply disturbance (velocity impulse for 20 steps)
- `m` - Cycle through control modes
- `r` - Reset system state and metrics

## Tuning Guide

To modify controller behavior:
- Increase `Q[0]` for faster position tracking
- Increase `Q[3]` for more velocity damping
- Increase `R[0]` for smoother, less aggressive control
- Modify mode-specific Q/R arrays for custom behavior

## Usage

1. Upload to Arduino
2. Open Serial Monitor at 115200 baud
3. Watch position converge to setpoint (5.0)
4. Press `m` to try different control modes
5. Press `d` to test disturbance rejection
