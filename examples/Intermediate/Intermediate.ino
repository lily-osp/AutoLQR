// Intermediate Example: Two-State System (Position + Velocity)
// This example controls a two-state system, such as a position and velocity control for a robotic arm or pendulum.

#include <AutoLQR.h>

// Define the system size
#define STATE_SIZE 2
#define CONTROL_SIZE 1

// Create an LQR object
AutoLQR lqr(STATE_SIZE, CONTROL_SIZE);

// Define matrices for the system
float A[STATE_SIZE * STATE_SIZE] = {1.0, 1.0, 0.0, 1.0}; // Position + velocity dynamics
float B[STATE_SIZE * CONTROL_SIZE] = {0.0, 1.0}; // Control affects velocity
float Q[STATE_SIZE * STATE_SIZE] = {10.0, 0.0, 0.0, 1.0}; // Penalize position more than velocity
float R[CONTROL_SIZE * CONTROL_SIZE] = {0.1}; // Penalize control effort

// Variables for state and control
float state[STATE_SIZE] = {5.0, 0.0}; // Initial state (position=5, velocity=0)
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

  // Apply control to system (for simulation, update position and velocity)
  state[0] += state[1]; // Update position
  state[1] += control[0]; // Update velocity based on control

  // Print the results
  Serial.print("Position: ");
  Serial.print(state[0]);
  Serial.print(", Velocity: ");
  Serial.print(state[1]);
  Serial.print(", Control: ");
  Serial.println(control[0]);

  delay(1000);
}
