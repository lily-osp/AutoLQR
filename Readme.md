# AutoLQR

A lightweight Linear Quadratic Regulator (LQR) controller library for Arduino and compatible microcontrollers. AutoLQR provides an easy-to-use implementation of LQR control theory for small embedded systems.

## Overview

Linear Quadratic Regulators are a form of optimal control that balances system performance against control effort. The AutoLQR library handles the complex matrix operations required to compute optimal feedback gains, allowing you to implement sophisticated control systems on resource-constrained platforms.

### Key Features

- **Minimal Dependencies**: Designed to work with the Arduino core libraries only
- **Memory Efficient**: Dynamic allocation of matrices based on system size
- **Flexible System Sizing**: Support for systems with multiple states and inputs
- **Simplified API**: Easy-to-use interface for setting up system dynamics and cost matrices
- **Pre-computed Gains**: Option to use offline-computed gains for complex systems
- **Optimal Control**: Implementation of discrete-time LQR control theory
- **Matrix Operations**: Built-in matrix utilities for embedded control applications

## Installation

### Arduino IDE

1. Download this repository as a ZIP file
2. In the Arduino IDE, navigate to Sketch > Include Library > Add .ZIP Library
3. Select the downloaded ZIP file
4. After installation, go to File > Examples > AutoLQR to view the examples

### Manual Installation

1. Download or clone this repository
2. Copy the contents to your Arduino libraries directory (usually `Documents/Arduino/libraries/`)

## Theory of Operation

### LQR Control Theory

The Linear Quadratic Regulator solves the optimal control problem by minimizing the infinite horizon cost function:

$$J = \sum_{k=0}^{\infty} (x_k^T Q x_k + u_k^T R u_k)$$

where:
- $x_k$ is the state vector at time k
- $u_k$ is the control input at time k
- $Q$ is the state cost matrix (penalizes state error)
- $R$ is the control cost matrix (penalizes control effort)

For a discrete-time linear system represented by:

$$x_{k+1} = A x_k + B u_k$$

The LQR controller produces a control law:

$$u_k = -K x_k$$

where K is the optimal feedback gain matrix computed by solving the Discrete Algebraic Riccati Equation (DARE).

### Discrete Algebraic Riccati Equation

The AutoLQR library solves the DARE iteratively:

$$P = A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A + Q$$

Once P converges, the optimal gain matrix K is calculated as:

$$K = (R + B^T P B)^{-1} B^T P A$$

## Using the Library

### System Initialization

```cpp
// Define system size
#define STATE_SIZE 2    // Number of state variables
#define CONTROL_SIZE 1  // Number of control inputs

// Create controller instance
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);
```

### Defining System Dynamics

The system dynamics are defined by two matrices:
- State matrix A describes how the system evolves on its own
- Input matrix B describes how control inputs affect the system

For example, a simple position-velocity system with discretized dynamics:

```cpp
// For a system with dt = 0.1s
float A[STATE_SIZE * STATE_SIZE] = {
  1.0, 0.1,  // Position += velocity * dt
  0.0, 0.95  // Velocity with damping factor
};

float B[STATE_SIZE * CONTROL_SIZE] = {
  0.005,  // Effect of control on position (0.5*dt^2)
  0.1     // Effect of control on velocity (dt)
};

// Set matrices in controller
controller.setStateMatrix(A);
controller.setInputMatrix(B);
```

### Defining Cost Parameters

The cost matrices Q and R define the trade-off between performance and control effort:
- Q penalizes state errors (higher values = faster response)
- R penalizes control effort (higher values = smoother control)

```cpp
// Q matrix - state cost
float Q[STATE_SIZE * STATE_SIZE] = {
  10.0, 0.0,  // Position error cost
  0.0,  1.0   // Velocity error cost
};

// R matrix - control cost
float R[CONTROL_SIZE * CONTROL_SIZE] = {
  0.1  // Control effort cost
};

// Set cost matrices
controller.setCostMatrices(Q, R);
```

### Computing Optimal Gains

Once the system is defined, compute the optimal gains:

```cpp
if (controller.computeGains()) {
  Serial.println("Gain computation successful");
} else {
  Serial.println("Error: Failed to compute gains");
}
```

### Using the Controller

In each control loop:

```cpp
// 1. Calculate state error (current state - desired state)
float state_error[STATE_SIZE] = {
  current_position - desired_position,
  current_velocity - desired_velocity
};

// 2. Update the controller with current state error
controller.updateState(state_error);

// 3. Calculate optimal control input
float control[CONTROL_SIZE];
controller.calculateControl(control);

// 4. Apply control to your system
motor_power = control[0];
```

### Using Pre-computed Gains

For complex systems or microcontrollers with limited computational resources:

```cpp
// Pre-computed optimal gain matrix (can be calculated offline)
float K[CONTROL_SIZE * STATE_SIZE] = {
  4.47, 2.28  // Example values
};

// Set pre-computed gains
controller.setGains(K);
```

## Advanced Features

### Feedforward Control

For setpoint tracking, you can calculate feedforward gains:

```cpp
float desired_state[STATE_SIZE] = {target_position, 0.0};
float feedforward[CONTROL_SIZE];

controller.estimateFeedforwardGain(feedforward, desired_state);

// Apply both feedback and feedforward control
float u_total = control[0] + feedforward[0];
```

### Convergence Time Estimation

Estimate how long the system will take to converge:

```cpp
float convergence_time = controller.estimateConvergenceTime(0.05); // 5% threshold
if (convergence_time > 0) {
  Serial.print("Estimated convergence time: ");
  Serial.println(convergence_time);
}
```

### Expected Cost Calculation

Calculate the expected cost from the current state:

```cpp
float expected_cost = controller.calculateExpectedCost();
if (expected_cost >= 0) {
  Serial.print("Expected cost: ");
  Serial.println(expected_cost);
}
```

### Checking System Controllability

Verify if your system is controllable:

```cpp
if (controller.isSystemControllable()) {
  Serial.println("System is controllable");
} else {
  Serial.println("Warning: System may not be controllable");
}
```

## Matrix Representation

All matrices are represented as flattened 1D arrays in row-major order:

```
[0,0] [0,1] [0,2]     A[0] A[1] A[2]
[1,0] [1,1] [1,2]  →  A[3] A[4] A[5]
[2,0] [2,1] [2,2]     A[6] A[7] A[8]
```

For example, a 3×3 state matrix A would be stored in a 9-element array.

## Performance Considerations

### Memory Usage

Memory allocation is proportional to:
- State matrix A: stateSize² floats
- Input matrix B: stateSize × controlSize floats
- Cost matrices Q and R: stateSize² and controlSize² floats
- Internal buffers for matrix operations

For a 2-state, 1-control system, approximately 72 bytes of RAM are used.

### Computational Complexity

- Matrix multiplication: O(n³) for n×n matrices
- DARE solver: O(k×n³) where k is the number of iterations
- Systems with more than 3 states may be computationally intensive

### Optimization Tips

- Pre-compute gains for larger systems
- Use fixed-point arithmetic for memory-constrained systems
- Consider decreasing DARE solver iteration count for faster computation
- Tune Q and R matrices to achieve desired control performance

## Example Applications

The AutoLQR library is suitable for a variety of control applications:

1. **Motor Position Control**: Precisely control motor position with minimal oscillation
2. **Self-balancing Robots**: Maintain stability using multiple sensor inputs
3. **Temperature Control Systems**: Optimize heating/cooling with minimal energy usage
4. **Drone Flight Control**: Stabilize attitude with optimal control authority
5. **Servo Control**: Smooth and precise control of servo mechanisms

See the examples folder for detailed implementation examples.

## Troubleshooting

### Gain Computation Fails

- Ensure matrices A, B, Q, and R have appropriate dimensions
- Check that Q and R are symmetric and positive definite
- Verify the system is controllable
- Try adjusting Q and R values (start with diagonal matrices)

### Unstable Control Output

- Check sign conventions in control output application
- Ensure state error is properly calculated (current - desired)
- Verify system model (A and B matrices) accurately represents the physical system
- Gradually increase Q values for more aggressive control
- Gradually increase R values for smoother control action

### Memory Issues

- Reduce system size if possible
- Use pre-computed gains instead of on-board computation
- Consider static allocation for very constrained systems

## API Reference

### Constructor

```cpp
AutoLQR(int stateSize, int controlSize)
```
- `stateSize`: Number of state variables
- `controlSize`: Number of control inputs

### Core Methods

```cpp
bool setStateMatrix(const float* A)
```
Sets the system dynamics matrix A. Returns success/failure.

```cpp
bool setInputMatrix(const float* B)
```
Sets the input matrix B. Returns success/failure.

```cpp
bool setCostMatrices(const float* Q, const float* R)
```
Sets the state cost matrix Q and control cost matrix R. Returns success/failure.

```cpp
bool computeGains()
```
Computes the optimal feedback gains by solving the discrete algebraic Riccati equation. Returns success/failure.

```cpp
void updateState(const float* currentState)
```
Updates the controller with the current system state or state error.

```cpp
void calculateControl(float* controlOutput)
```
Calculates the optimal control inputs for the current state.

```cpp
void setGains(const float* K)
```
Sets pre-computed gain values directly.

### Advanced Methods

```cpp
bool isSystemControllable()
```
Checks if the system is controllable. Returns true/false.

```cpp
const float* getRicattiSolution() const
```
Gets the solution of the Riccati equation (P matrix).

```cpp
void estimateFeedforwardGain(float* ffGain, const float* desiredState)
```
Estimates feedforward gain for steady-state tracking.

```cpp
float estimateConvergenceTime(float convergenceThreshold = 0.05f)
```
Estimates time to convergence. Returns time in seconds or -1 if estimation fails.

```cpp
bool exportGains(float* exportedK)
```
Exports computed gains to an external array. Returns success/failure.

```cpp
float calculateExpectedCost()
```
Calculates expected cost from current state. Returns cost value or -1 if calculation fails.

## Contributing

Contributions to improve AutoLQR are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch
3. Implement your changes
4. Submit a pull request

Please include tests and example code when adding new features.

## License

This library is released under the MIT License. See [LICENSE](LICENSE) for details.

## Credits

Created by [1999AZZAR](https://github.com/1999AZZAR).

## Contact

For questions or support, please contact: azzar.mr.zs@gmail.com
