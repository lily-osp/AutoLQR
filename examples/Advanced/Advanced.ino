// Advanced Example: Balancing an Inverted Pendulum
// This example balances an inverted pendulum system, a classic control problem with 4 states (position, velocity, angle, angular velocity).

#include <AutoLQR.h>

// Define the system size
#define STATE_SIZE 4
#define CONTROL_SIZE 1

// Create an LQR object
AutoLQR lqr(STATE_SIZE, CONTROL_SIZE);

// Define matrices for the system
float A[STATE_SIZE * STATE_SIZE] = {
  1.0, 0.01, 0.0, 0.0,
  0.0, 1.0, -0.01, 0.0,
  0.0, 0.0, 1.0, 0.01,
  0.0, 0.0, 9.8, 1.0
}; // Approximate linearized dynamics
float B[STATE_SIZE * CONTROL_SIZE] = {0.0, 0.1, 0.0, -0.1}; // Control affects velocity and angle
float Q[STATE_SIZE * STATE_SIZE] = {
  100.0, 0.0, 0.0, 0.0,
  0.0, 10.0, 0.0, 0.0,
  0.0, 0.0, 100.0, 0.0,
  0.0, 0.0, 0.0, 10.0
}; // Penalize position and angle heavily
float R[CONTROL_SIZE * CONTROL_SIZE] = {1.0}; // Penalize control effort

// Variables for state and control
float state[STATE_SIZE] = {0.5, 0.0, 0.1, 0.0}; // Initial state (small angle + position offset)
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

  // Simulate the system response (simple Euler integration)
  float newState[STATE_SIZE];
  for (int i = 0; i < STATE_SIZE; i++) {
    newState[i] = 0;
    for (int j = 0; j < STATE_SIZE; j++) {
      newState[i] += A[i * STATE_SIZE + j] * state[j];
    }
    newState[i] += B[i] * control[0];
  }

  memcpy(state, newState, sizeof(float) * STATE_SIZE);

  // Print the results
  Serial.print("Position: ");
  Serial.print(state[0]);
  Serial.print(", Velocity: ");
  Serial.print(state[1]);
  Serial.print(", Angle: ");
  Serial.print(state[2]);
  Serial.print(", Angular Velocity: ");
  Serial.print(state[3]);
  Serial.print(", Control: ");
  Serial.println(control[0]);

  delay(10); // Faster simulation
}
