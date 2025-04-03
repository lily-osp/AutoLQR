This example implements a complete motor position control system using the AutoLQR library. Here's a breakdown of how it works:

### Hardware Components
- DC motor with an encoder (1200 counts per revolution)
- H-bridge motor driver connected to PWM and direction pins
- Potentiometer to set the target position
- Arduino board for control

### Key Features

1. **Sensor Integration**: Uses quadrature encoder feedback to measure motor position and velocity with interrupt-based counting for accuracy.

2. **Actuator Control**: Generates PWM and direction signals to control a DC motor through an H-bridge driver.

3. **User Input**: Reads a potentiometer to set the target position, allowing real-time adjustment.

4. **State Estimation**: Calculates position and velocity from encoder data.

5. **LQR Controller**: Implements a full state-feedback controller with:
   - Properly discretized system model (A and B matrices)
   - Tunable cost matrices (Q and R)
   - Real-time control signal calculation

6. **Performance Monitoring**: Logs position, velocity, error, and expected cost to serial monitor.

7. **Timing Control**: Maintains consistent control loop frequency.

### System Model

The example uses a second-order model for the motor:
- States: position (revolutions) and velocity (revolutions/second)
- Control input: motor voltage (applied via PWM)
- Discretized dynamics with velocity damping factor

### Adjusting the Controller

You can tune the controller by modifying:
1. The Q matrix - higher values for position error will make the system respond faster
2. The R matrix - higher values will make control smoother but slower
3. The motor model parameters in the A and B matrices
