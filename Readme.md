# AutoLQR Library for Arduino

## Overview
The **AutoLQR** library for Arduino provides an intuitive and efficient implementation of the Linear Quadratic Regulator (LQR), a powerful control technique for dynamic systems. With AutoLQR, you can create optimized controllers for single-input, single-output (SISO) systems, or even multi-input, multi-output (MIMO) systems, directly on your Arduino-compatible hardware.

### What is LQR?
Linear Quadratic Regulator (LQR) is a method used to compute optimal feedback gains for controlling a linear dynamic system. It balances minimizing:
- **State deviations** from a desired target (e.g., position, velocity, angle).
- **Control effort** required to achieve that goal (e.g., motor torque, current).

LQR achieves this balance by solving a mathematical optimization problem, generating control gains that lead to:
1. **Stability**: The system converges to the desired state.
2. **Efficiency**: The control effort is minimized.

LQR is widely used in robotics, aerospace, and industrial automation due to its ability to handle dynamic systems robustly.

---

## Features
- **Flexible System Dimensions**: Supports arbitrary state and control dimensions.
- **Dynamic Gains Computation**: Automatically computes the optimal feedback gain matrix \( K \).
- **Customizable Costs**: Allows tuning of state and control penalties through \( Q \) and \( R \) matrices.
- **Lightweight Implementation**: Optimized for Arduino's limited memory and computational power.

---

## System Model
AutoLQR assumes your system is modeled as a discrete-time linear state-space system:

\[
\mathbf{x}[k+1] = \mathbf{A}\mathbf{x}[k] + \mathbf{B}\mathbf{u}[k] \\
\mathbf{u}[k] = -\mathbf{K}\mathbf{x}[k]
\]

Where:
- \( \mathbf{x}[k] \): System state vector at time step \( k \) (e.g., position, velocity).
- \( \mathbf{u}[k] \): Control input vector at time step \( k \) (e.g., motor command).
- \( \mathbf{A} \): State matrix defining the system's dynamics.
- \( \mathbf{B} \): Input matrix defining how control inputs affect the system.
- \( \mathbf{K} \): Optimal feedback gain matrix computed by LQR.

The LQR algorithm calculates \( \mathbf{K} \) by minimizing the cost function:

\[
J = \sum_{k=0}^{\infty} \left( \mathbf{x}^T[k]\mathbf{Q}\mathbf{x}[k] + \mathbf{u}^T[k]\mathbf{R}\mathbf{u}[k] \right)
\]

Where:
- \( \mathbf{Q} \): State cost matrix (penalizes deviation from the desired state).
- \( \mathbf{R} \): Input cost matrix (penalizes control effort).

---

## Library Components
The **AutoLQR** library consists of two main files:
1. `AutoLQR.h`
2. `AutoLQR.cpp`

Additionally, there is a `library.properties` file for Arduino IDE compatibility.

### Key Functions in AutoLQR

#### 1. Constructor: `AutoLQR(int stateSize, int controlSize)`
Initializes the LQR controller.
- `stateSize`: Number of states in the system (size of \( \mathbf{x} \)).
- `controlSize`: Number of control inputs (size of \( \mathbf{u} \)).

#### 2. `void setStateMatrix(float* A)`
Sets the state matrix \( \mathbf{A} \).
- Input: Pointer to a row-major array representing the \( \mathbf{A} \) matrix.

#### 3. `void setInputMatrix(float* B)`
Sets the input matrix \( \mathbf{B} \).
- Input: Pointer to a row-major array representing the \( \mathbf{B} \) matrix.

#### 4. `void setCostMatrices(float* Q, float* R)`
Sets the cost matrices \( \mathbf{Q} \) and \( \mathbf{R} \).
- Inputs:
  - `Q`: Pointer to a row-major array representing the \( \mathbf{Q} \) matrix.
  - `R`: Pointer to a row-major array representing the \( \mathbf{R} \) matrix.

#### 5. `bool computeGains()`
Computes the optimal feedback gain matrix \( \mathbf{K} \) by solving the discrete-time algebraic Riccati equation (DARE).
- Returns: `true` if computation succeeds; `false` otherwise.

#### 6. `void updateState(float* state)`
Updates the current state vector \( \mathbf{x} \).
- Input: Pointer to the current state vector.

#### 7. `void calculateControl(float* control)`
Calculates the control vector \( \mathbf{u} \) based on the current state and gain matrix \( \mathbf{K} \).
- Output: Pointer to the calculated control vector.

#### 8. `void printGains()`
Prints the computed \( \mathbf{K} \) matrix to the serial monitor for debugging.

---

## Cost Matrices \( \mathbf{Q} \) and \( \mathbf{R} \): Tuning Guidelines
The \( \mathbf{Q} \) and \( \mathbf{R} \) matrices allow you to adjust the balance between state accuracy and control effort:

- **\( \mathbf{Q} \): State Cost Matrix**
  - Larger values penalize deviations in specific states (e.g., position or angle).
  - Example: To prioritize accurate position tracking, increase the corresponding diagonal element of \( \mathbf{Q} \).

- **\( \mathbf{R} \): Input Cost Matrix**
  - Larger values penalize high control efforts (e.g., motor torque or speed).
  - Example: For limited power systems, increase \( \mathbf{R} \) values to reduce excessive control commands.

---

## Notes on Stability and Performance
1. **Stability**: Ensure your system is controllable and the \( \mathbf{Q} \) and \( \mathbf{R} \) matrices are positive definite.
2. **Discretization**: The library assumes a discrete-time system. If you start with continuous-time matrices, discretize them using the sampling time.
3. **Computational Limits**: Arduino's computational power is limited. For large systems, precompute the \( \mathbf{K} \) matrix offline and load it directly.

---

## Usage Workflow
1. Define your system's \( \mathbf{A} \) and \( \mathbf{B} \) matrices.
2. Define the cost matrices \( \mathbf{Q} \) and \( \mathbf{R} \) based on your priorities.
3. Use the library to compute the optimal \( \mathbf{K} \) matrix.
4. Continuously update the system state and calculate the control inputs using the library's functions.

---

## Example Use Cases
### Simple Example:
- Single-state control (e.g., position control of a motor).

### Intermediate Example:
- Two-state systems (e.g., position and velocity of a robot arm).

### Advanced Example:
- Multi-state systems like balancing an inverted pendulum.

For complete code examples, refer to the provided files.

---

## Compatibility
- Tested on Arduino Uno, Mega, and ESP32.
- Requires Arduino IDE version 1.8.13 or higher.

---

## License
This library is open-source and licensed under the MIT License. Feel free to modify and adapt it to your specific needs.

