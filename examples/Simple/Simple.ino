// Enhanced Simple Example: Single-State System Control with Additional Features
// This example controls a single-state system, such as a position control with one input (e.g., a motor's speed).

#include <AutoLQR.h>

// Single state system (position only)
#define STATE_SIZE 1
#define CONTROL_SIZE 1

// System matrices
float A[STATE_SIZE * STATE_SIZE] = { 0.95 }; // Natural decay (e.g., friction)
float B[STATE_SIZE * CONTROL_SIZE] = { 0.1 }; // Effect of input on position

// Cost matrices
float Q[STATE_SIZE * STATE_SIZE] = { 1.0 }; // Cost on position error
float R[CONTROL_SIZE * CONTROL_SIZE] = { 0.1 }; // Cost on control effort

// Controller and state variables
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);
float currentState[STATE_SIZE] = { 0 }; // Current position
float desiredState[STATE_SIZE] = { 10.0 }; // Target position (10 units)
float controlInput[CONTROL_SIZE] = { 0 }; // Control output
float feedforwardGain[CONTROL_SIZE] = { 0 }; // Feedforward gain for steady-state tracking

// Simulated system variables
float actualPosition = 0.0;
bool useAdaptive = false;
bool useFeedforward = false;
unsigned long startTime = 0;
int updateCount = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("Enhanced Position Control LQR Example");
    Serial.println("Commands: 'a' - toggle adaptive control, 'f' - toggle feedforward, 'n' - new setpoint");

    // Initialize controller with system dynamics
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);

    // Compute optimal gains
    if (controller.computeGains()) {
        Serial.println("LQR gains computed successfully");

        // Calculate feedforward gain for steady-state tracking
        controller.estimateFeedforwardGain(feedforwardGain, desiredState);
        Serial.print("Feedforward gain: ");
        Serial.println(feedforwardGain[0], 4);

        // Estimate convergence time
        float convTime = controller.estimateConvergenceTime(0.05);
        Serial.print("Estimated time to 95% convergence: ");
        Serial.print(convTime);
        Serial.println(" time steps");
    } else {
        Serial.println("Failed to compute LQR gains");
    }

    // Check if system is controllable
    if (controller.isSystemControllable()) {
        Serial.println("System is controllable");
    } else {
        Serial.println("Warning: System may not be controllable");
    }

    startTime = millis();
}

void loop()
{
    // Process any incoming commands
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'a') {
            useAdaptive = !useAdaptive;
            Serial.print("Adaptive control: ");
            Serial.println(useAdaptive ? "ON" : "OFF");
        } else if (cmd == 'f') {
            useFeedforward = !useFeedforward;
            Serial.print("Feedforward: ");
            Serial.println(useFeedforward ? "ON" : "OFF");
        } else if (cmd == 'n') {
            // Change setpoint randomly
            desiredState[0] = random(5, 15);
            controller.estimateFeedforwardGain(feedforwardGain, desiredState);
            Serial.print("New setpoint: ");
            Serial.println(desiredState[0]);
        }
    }

    // Calculate error (current state - desired state)
    currentState[0] = actualPosition - desiredState[0];

    // Update controller with current state error
    controller.updateState(currentState);

    // Calculate control input
    controller.calculateControl(controlInput);

    // Apply adaptive control if enabled (example: adjust gains based on error magnitude)
    if (useAdaptive && abs(currentState[0]) > 5.0) {
        // Increase response for large errors
        controlInput[0] *= 1.2;
    }

    // Add feedforward term if enabled
    if (useFeedforward) {
        controlInput[0] += feedforwardGain[0];
    }

    // Apply control to simulated system
    actualPosition = A[0] * actualPosition + B[0] * controlInput[0];

    // Calculate expected cost
    float expectedCost = controller.calculateExpectedCost();

    // Print current state and control (every 10 iterations to avoid flooding serial)
    updateCount++;
    if (updateCount % 10 == 0) {
        Serial.print("Position: ");
        Serial.print(actualPosition);
        Serial.print(", Target: ");
        Serial.print(desiredState[0]);
        Serial.print(", Control: ");
        Serial.print(controlInput[0]);

        if (expectedCost >= 0) {
            Serial.print(", Cost: ");
            Serial.print(expectedCost, 2);
        }

        // Calculate runtime stats
        float runtime = (millis() - startTime) / 1000.0;
        Serial.print(", Runtime: ");
        Serial.print(runtime, 1);
        Serial.println("s");
    }

    delay(100); // Update at 10Hz
}
