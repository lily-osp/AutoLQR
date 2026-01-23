# Quadcopter Single-Axis Stabilization Example

LQR-based attitude control for a single axis (roll or pitch) of a quadcopter or similar multirotor vehicle.

## System Model

The angular dynamics are modeled as a second-order system:

- **State Vector**: `[angle, angular_velocity]` in degrees and degrees/second
- **Control Input**: Differential motor thrust

### State-Space Representation

```
x(k+1) = A * x(k) + B * u(k)
```

Where:
- `x = [theta, omega]` (angle and angular rate)
- `u = differential_thrust` (difference between left/right motors)

The control produces torque which generates angular acceleration:
```
tau = arm_length * thrust_coefficient * u
alpha = tau / moment_of_inertia
```

## Hardware Requirements

| Component | Description | Pin |
|-----------|-------------|-----|
| IMU | MPU6050 (I2C) | SDA/SCL |
| Left Motor ESC | PWM signal | D9 |
| Right Motor ESC | PWM signal | D10 |
| Status LED | Armed indicator | D13 |

### Wiring Diagram

```
Arduino           MPU6050
--------          -------
3.3V/5V  ------>  VCC
GND      ------>  GND
A4 (SDA) <-----> SDA
A5 (SCL) <-----> SCL

Arduino           ESC (Left)
--------          ----------
D9       ------>  Signal
GND      ------>  GND

Arduino           ESC (Right)
--------          ----------
D10      ------>  Signal
GND      ------>  GND
```

## Configuration

### Physical Parameters

Adjust these for your specific aircraft:

```cpp
const float momentOfInertia = 0.01f;  // kg*m^2
const float armLength = 0.15f;         // Motor arm length (m)
const float thrustCoeff = 0.00001f;    // Thrust per PWM unit
```

### Control Limits

```cpp
const float maxAngle = 45.0f;        // Maximum allowed angle (degrees)
const float maxAngularRate = 300.0f;  // Maximum angular rate (deg/s)
```

### IMU Filter

The example uses a complementary filter:

```cpp
const float filterAlpha = 0.98f;  // Higher = trust gyro more
```

## Serial Commands

| Command | Description | Example |
|---------|-------------|---------|
| `A` | Arm motors | - |
| `D` | Disarm motors | - |
| `T[value]` | Set base throttle (1200-1800) | `T1400` |
| `R[angle]` | Set target angle (-30 to 30) | `R10` |
| `C` | Recalibrate gyro | - |
| `S` | Print detailed status | - |

## Safety Features

1. Automatic disarm on angle/rate limit exceeded
2. Gyro calibration required before arming
3. Control output limiting
4. Motor PWM clamping

## Tuning

### Cost Matrices

```cpp
Q[0] = 100.0f;  // Angle error weight
Q[3] = 10.0f;   // Rate error weight
R[0] = 1.0f;    // Control effort weight
```

Tuning guidelines:
- Increase `Q[0]` for faster angle correction (may increase oscillation)
- Increase `Q[3]` to dampen oscillations
- Increase `R[0]` for smoother motor commands (slower response)

### System Identification

To estimate physical parameters:

1. Measure vehicle mass and arm lengths
2. Calculate moment of inertia: `I = m * r^2` (simplified)
3. Measure thrust at known throttle settings
4. Observe natural damping from test flights

## Operation Procedure

1. Power on with vehicle level
2. Wait for initialization and gyro calibration
3. Send `A` to arm motors
4. Gradually increase throttle with `T[value]`
5. Test angle commands with `R[angle]`
6. Send `D` to disarm when done

## Limitations

This example is for educational purposes and demonstrates single-axis control only. A full flight controller requires:

- Full 3-axis attitude control (roll, pitch, yaw)
- Sensor fusion (accelerometer + gyroscope + magnetometer)
- Position control (GPS, barometer)
- Proper failsafe mechanisms
- RC receiver integration
- Battery monitoring

## References

- Linear Quadratic Regulator design for multirotor vehicles
- Complementary filter for attitude estimation
- MPU6050 datasheet and register map
