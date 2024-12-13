// Simple Example: Single-State System Control
// This example controls a single-state system, such as a position control with one input (e.g., a motor's speed).

#include <AutoLQR.h>

// Define the system size
#define STATE_SIZE 1
#define CONTROL_SIZE 1

// Create an LQR object
AutoLQR lqr(STATE_SIZE, CONTROL_SIZE);

// Define matrices for the system
float A[STATE_SIZE * STATE_SIZE] = {1.0}; // Identity matrix for simplicity
float B[STATE_SIZE * CONTROL_SIZE] = {1.0}; // Direct mapping from input to state
float Q[STATE_SIZE * STATE_SIZE] = {1.0}; // Penalize deviation from desired state
float R[CONTROL_SIZE * CONTROL_SIZE] = {1.0}; // Penalize control effort

// Variables for state and control
float state[STATE_SIZE] = {5.0}; // Initial state (e.g., position)
float control[CONTROL_SIZE] = {0.0}; // Control output

void setup() {
  Serial.begin(9600);

  // Set system matrices and compute gains
  lqr.setStateMatrix(A);
  lqr.setInputMatrix(B);
  lqr.setCostMatrices(Q, R);
  lqr.computeGains();
}

void loop() {
  // Update the current state
  lqr.updateState(state);

  // Calculate the control signal
  lqr.calculateControl(control);

  // Apply control to system (for simulation, we'll just update state)
  state[0] -= control[0]; // Simulating system response to control

  // Print the results
  Serial.print("State: ");
  Serial.print(state[0]);
  Serial.print(", Control: ");
  Serial.println(control[0]);

  delay(1000);
}
