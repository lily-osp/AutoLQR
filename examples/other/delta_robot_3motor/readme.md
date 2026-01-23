# Delta Robot 3-Motor LQR Control

This example demonstrates how to control 3 independent DC motors for a Delta Robot using the AutoLQR library.

## Approach

For a Delta Robot with 3 identical, mechanically independent DC motors, the recommended approach is to use 3 separate AutoLQR instances rather than a single MIMO controller:

```cpp
AutoLQR motor1Controller(2, 1);  // 2 states, 1 control
AutoLQR motor2Controller(2, 1);
AutoLQR motor3Controller(2, 1);
```

### Why Separate Controllers

1. Each motor is mechanically independent (no coupled dynamics)
2. Memory efficient - no large 6x6 matrices required
3. Works within library limits (matrix inversion up to 3x3)
4. Easier to tune individual motors if needed
5. Cleaner code architecture

### Why NOT a Single MIMO Controller

A single controller with STATE_SIZE=2 and CONTROL_SIZE=3 would mathematically imply that all 3 motors affect the same 2 state variables. This does not represent 3 independent motors.

A proper MIMO formulation would require:

- STATE_SIZE = 6 (position and velocity for each motor)
- CONTROL_SIZE = 3
- 6x6 A matrix, 6x3 B matrix

This exceeds the current library's matrix inversion capability and provides no practical benefit for independent motors.

## Control Architecture

```
[User Input: X,Y,Z] --> [Inverse Kinematics] --> [theta1, theta2, theta3]
                                                        |
                        +-------------------------------+
                        |               |               |
                        v               v               v
                   [Motor 1 LQR]  [Motor 2 LQR]  [Motor 3 LQR]
                        |               |               |
                        v               v               v
                      PWM1            PWM2            PWM3
```

## Hardware Requirements

- Arduino Mega 2560 (or similar with sufficient pins and memory)
- 3x DC geared motors with quadrature encoders
- 3x Motor drivers (L298N, TB6612, or similar H-bridge)
- Delta robot mechanical structure

## Pin Configuration

| Motor | PWM Pin | DIR Pin | Encoder A | Encoder B |
| ----- | ------- | ------- | --------- | --------- |
| 1     | 9       | 8       | 2 (INT)   | 22        |
| 2     | 10      | 11      | 3 (INT)   | 23        |
| 3     | 12      | 13      | 18 (INT)  | 24        |

## Delta Robot Parameters

Adjust these constants to match your robot geometry:

```cpp
#define BASE_RADIUS 100.0      // Distance from center to motor axis (mm)
#define EFFECTOR_RADIUS 30.0   // Distance from center to effector joint (mm)
#define ARM_LENGTH 150.0       // Upper arm length (mm)
#define FOREARM_LENGTH 250.0   // Lower arm/rod length (mm)
```

## Usage

1. Upload the sketch to your Arduino Mega
2. Open Serial Monitor at 115200 baud
3. Send target coordinates as comma-separated values: `X,Y,Z`
4. Example: `0,0,-200` moves to position (0, 0, -200) mm

## Tuning

The LQR controller can be tuned by adjusting the Q and R matrices:

```cpp
// State cost matrix Q
Q[0] = 150.0f;  // Position error cost (higher = faster tracking)
Q[3] = 15.0f;   // Velocity error cost (higher = more damping)

// Control cost matrix R
R[0] = 1.0f;    // Control effort cost (higher = smoother but slower)
```

### Tuning Guidelines

- Increase Q[0] for tighter position tracking
- Increase Q[3] to reduce overshoot
- Increase R[0] for smoother motion (less aggressive)
- Adjust velocity damping factor A[3] based on motor characteristics

## Troubleshooting

**Oscillation/Shaking**: Reduce Q[0] or increase R[0]

**Slow Response**: Increase Q[0] or reduce R[0]

**Overshoot**: Increase Q[3] (velocity cost)

**Position Not Reached**: Check encoder wiring and COUNTS_PER_REV value
