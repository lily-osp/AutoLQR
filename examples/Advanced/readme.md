# Advanced Example

This example demonstrates LQR control for an inverted pendulum system with four states, multiple control strategies, and advanced features.

## Overview

Simulates and controls an inverted pendulum on a cart, showcasing:
- Four-state system control
- Multiple control modes with gain scheduling
- Swing-up controller for large angles
- System identification
- Disturbance rejection
- Fallback to pre-computed gains

## System Model

The inverted pendulum is linearized around the upright equilibrium:

- **States**: Cart position, cart velocity, pendulum angle, angular velocity (4 states)
- **Control**: Force applied to cart (1 input)
- **Parameters**:
  - Pendulum mass: 0.1 kg
  - Cart mass: 0.5 kg
  - Pendulum length: 0.5 m
  - Control rate: 100 Hz

## Control Modes

| Mode | Description | Q Emphasis |
|------|-------------|------------|
| Balance | Keep pendulum upright | Angle error |
| Position Tracking | Move cart to target | Position error |
| Energy Saving | Minimize control effort | Lower all costs |
| Swing-Up | Bring pendulum from down to up | Energy-based control |

## Features

1. **Gain Scheduling**: Different Q/R matrices for each mode, gains recomputed on mode change
2. **Swing-Up Control**: Energy-based controller for large angles, switches to LQR near upright
3. **System Identification**: Excites system with sinusoidal input to estimate parameters
4. **Fallback Gains**: Pre-computed gains used if online computation fails
5. **Disturbance Testing**: Apply force impulse to test recovery
6. **Performance Metrics**: Tracks angle error, energy usage, recovery count

## Serial Commands

- `1` - Balance mode (default)
- `2` - Position tracking mode (moves cart to x=1.0)
- `3` - Energy saving mode
- `4` - Swing-up mode (starts from bottom)
- `d` - Apply disturbance force
- `i` - Run system identification
- `f` - Toggle between computed and fallback gains

## Key Concepts Demonstrated

### Gain Scheduling
```cpp
controller.setCostMatrices(Q_position, R);
controller.computeGains();  // Recompute for new weights
```

### Swing-Up Control
Uses energy-based control when angle is large:
```cpp
float energy = 0.5 * m * l * l * omega^2 + m * g * l * (cos(theta) - 1);
float control = sign(energyError * omega * cos(theta));
```

### Fallback Gains
```cpp
float fallbackK[] = {-1.0, -1.5, 20.0, 3.5};
controller.setGains(fallbackK);
```

## Usage

1. Upload to Arduino
2. Open Serial Monitor at 115200 baud
3. Watch pendulum balance (starts with 0.1 rad tilt)
4. Try different modes with number keys
5. Test disturbance rejection with `d`
