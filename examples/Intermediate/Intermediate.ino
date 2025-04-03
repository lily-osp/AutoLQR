// Enhanced Intermediate Example: Two-State System (Position + Velocity)
// This example controls a two-state system with additional features
// like disturbance rejection, gain scheduling, and performance metrics.

#include <AutoLQR.h>

// Two state system (position and velocity)
#define STATE_SIZE 2
#define CONTROL_SIZE 1

// System matrices
float A[STATE_SIZE * STATE_SIZE] = {
    1.0, 0.1, // Position += velocity * dt
    0.0, 0.98 // Velocity has slight damping
};

float B[STATE_SIZE * CONTROL_SIZE] = {
    0.005, // Input has small direct effect on position
    0.1 // Input primarily affects velocity
};

// Cost matrices for normal operation
float Q[STATE_SIZE * STATE_SIZE] = {
    10.0, 0.0, // Cost on position error
    0.0, 1.0 // Cost on velocity error
};

// Alternative cost matrix for high precision mode
float Q_precise[STATE_SIZE * STATE_SIZE] = {
    30.0, 0.0, // Higher cost on position error
    0.0, 1.0 // Same cost on velocity error
};

// Cost matrix for energy-saving mode
float Q_efficient[STATE_SIZE * STATE_SIZE] = {
    5.0, 0.0, // Lower cost on position error
    0.0, 0.5 // Lower cost on velocity error
};

float R[CONTROL_SIZE * CONTROL_SIZE] = {
    0.1 // Cost on control effort
};

// Alternative R for smoother control
float R_smooth[CONTROL_SIZE * CONTROL_SIZE] = {
    0.5 // Higher cost on control effort
};

// Controller and state variables
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);
float currentState[STATE_SIZE] = { 0, 0 }; // Current [position, velocity]
float desiredState[STATE_SIZE] = { 5.0, 0.0 }; // Target [position, velocity]
float errorState[STATE_SIZE] = { 0, 0 }; // Error state
float controlInput[CONTROL_SIZE] = { 0 }; // Control output
float feedforwardGain[CONTROL_SIZE] = { 0 }; // Feedforward gain

// Simulated system variables
float actualPosition = 0.0;
float actualVelocity = 0.0;
const float dt = 0.1; // Time step in seconds

// Performance tracking variables
float totalError = 0.0;
float maxError = 0.0;
int oscillationCount = 0;
bool lastErrorPositive = false;
int stepCount = 0;
bool disturbanceActive = false;
int disturbanceCountdown = 0;
int controlMode = 0; // 0=normal, 1=precise, 2=efficient, 3=smooth

void setup()
{
    Serial.begin(115200);
    Serial.println("Enhanced Position and Velocity Control LQR Example");
    Serial.println("Commands: 'd' - add disturbance, 'm' - change control mode, 'r' - reset");

    // Initialize controller with system dynamics
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);

    // Compute optimal gains
    if (controller.computeGains()) {
        Serial.println("LQR gains computed successfully");

        // Export the computed gains
        float exportedGains[CONTROL_SIZE * STATE_SIZE];
        controller.exportGains(exportedGains);
        Serial.print("Position gain: ");
        Serial.print(exportedGains[0], 4);
        Serial.print(", Velocity gain: ");
        Serial.println(exportedGains[1], 4);

        // Compute feedforward gain
        controller.estimateFeedforwardGain(feedforwardGain, desiredState);
        Serial.print("Feedforward gain: ");
        Serial.println(feedforwardGain[0], 4);

        // Estimate convergence time
        float convTime = controller.estimateConvergenceTime(0.05);
        if (convTime > 0) {
            Serial.print("Estimated convergence time: ");
            Serial.print(convTime);
            Serial.println(" steps");
        }
    } else {
        Serial.println("Failed to compute LQR gains");
    }
}

void loop()
{
    // Check for commands
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'd') {
            // Add a disturbance
            disturbanceActive = true;
            disturbanceCountdown = 20; // Apply disturbance for 20 steps
            Serial.println("Disturbance applied!");
        } else if (cmd == 'm') {
            // Change control mode
            controlMode = (controlMode + 1) % 4;

            switch (controlMode) {
            case 0: // Normal
                controller.setCostMatrices(Q, R);
                Serial.println("Normal control mode");
                break;
            case 1: // Precise
                controller.setCostMatrices(Q_precise, R);
                Serial.println("Precise control mode");
                break;
            case 2: // Efficient
                controller.setCostMatrices(Q_efficient, R);
                Serial.println("Efficient control mode");
                break;
            case 3: // Smooth
                controller.setCostMatrices(Q, R_smooth);
                Serial.println("Smooth control mode");
                break;
            }

            // Recompute gains for new mode
            controller.computeGains();
        } else if (cmd == 'r') {
            // Reset
            actualPosition = 0.0;
            actualVelocity = 0.0;
            totalError = 0.0;
            maxError = 0.0;
            oscillationCount = 0;
            stepCount = 0;
            Serial.println("System reset");
        }
    }

    // Calculate error state (current - desired)
    errorState[0] = actualPosition - desiredState[0];
    errorState[1] = actualVelocity - desiredState[1];

    // Update controller with current state error
    controller.updateState(errorState);

    // Calculate control input
    controller.calculateControl(controlInput);

    // Add feedforward component for better tracking
    controlInput[0] += feedforwardGain[0];

    // Apply disturbance if active
    if (disturbanceActive) {
        actualVelocity += 0.5; // Add velocity disturbance
        disturbanceCountdown--;
        if (disturbanceCountdown <= 0) {
            disturbanceActive = false;
        }
    }

    // Apply control to simulated system (simplified discrete model)
    actualVelocity = A[2] * actualVelocity + B[1] * controlInput[0];
    actualPosition += actualVelocity * dt + B[0] * controlInput[0];

    // Track performance metrics
    stepCount++;
    float absError = fabs(errorState[0]);
    totalError += absError;
    if (absError > maxError)
        maxError = absError;

    // Count oscillations (sign changes in error)
    bool currentErrorPositive = errorState[0] > 0;
    if (currentErrorPositive != lastErrorPositive && stepCount > 5) {
        oscillationCount++;
    }
    lastErrorPositive = currentErrorPositive;

    // Calculate expected cost
    float expectedCost = controller.calculateExpectedCost();

    // Print current states and control (every 5 iterations)
    if (stepCount % 5 == 0) {
        Serial.print("Position: ");
        Serial.print(actualPosition, 3);
        Serial.print(", Velocity: ");
        Serial.print(actualVelocity, 3);
        Serial.print(", Error: ");
        Serial.print(errorState[0], 3);
        Serial.print(", Control: ");
        Serial.print(controlInput[0], 3);

        // Print performance metrics every 20 steps
        if (stepCount % 20 == 0) {
            Serial.print(" | Avg Error: ");
            Serial.print(totalError / stepCount, 3);
            Serial.print(", Max Error: ");
            Serial.print(maxError, 3);
            Serial.print(", Oscillations: ");
            Serial.print(oscillationCount);

            if (expectedCost >= 0) {
                Serial.print(", Cost: ");
                Serial.print(expectedCost, 2);
            }
        }

        Serial.println();
    }

    delay(100); // Update at 10Hz
}
