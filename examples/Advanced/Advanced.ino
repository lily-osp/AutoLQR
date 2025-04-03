// Enhanced Advanced Example: Balancing an Inverted Pendulum
// This example balances an inverted pendulum system with additional features
// such as switching control strategies, external disturbance testing, and
// system identification capabilities.

#include <AutoLQR.h>

// Four state system (cart position, cart velocity, pendulum angle, pendulum angular velocity)
#define STATE_SIZE 4
#define CONTROL_SIZE 1

// System parameters
const float g = 9.81; // Gravity (m/s^2)
const float m = 0.1; // Pendulum mass (kg)
const float M = 0.5; // Cart mass (kg)
const float l = 0.5; // Pendulum length (m)
const float dt = 0.01; // Time step (s)
const float damping = 0.1; // Damping factor

// System matrices (linearized around the upright equilibrium)
float A[STATE_SIZE * STATE_SIZE] = {
    1.0, dt, 0.0, 0.0,
    0.0, 1.0 - damping* dt, m* g* dt / M, 0.0,
    0.0, 0.0, 1.0, dt,
    0.0, 0.0, g*(m + M) * dt / (l * M), 1.0
};

float B[STATE_SIZE * CONTROL_SIZE] = {
    0.0,
    dt / M,
    0.0,
    dt / (l * M)
};

// Cost matrices - Standard balancing mode
float Q[STATE_SIZE * STATE_SIZE] = {
    1.0, 0.0, 0.0, 0.0, // Position error cost
    0.0, 0.5, 0.0, 0.0, // Velocity error cost
    0.0, 0.0, 10.0, 0.0, // Angle error cost (higher to prioritize keeping pendulum upright)
    0.0, 0.0, 0.0, 1.0 // Angular velocity error cost
};

// Cost matrices - Position tracking mode
float Q_position[STATE_SIZE * STATE_SIZE] = {
    5.0, 0.0, 0.0, 0.0, // Higher position error cost
    0.0, 0.5, 0.0, 0.0, // Velocity error cost
    0.0, 0.0, 10.0, 0.0, // Angle error cost
    0.0, 0.0, 0.0, 1.0 // Angular velocity error cost
};

// Cost matrices - Energy conservation mode
float Q_energy[STATE_SIZE * STATE_SIZE] = {
    0.5, 0.0, 0.0, 0.0, // Lower position error cost
    0.0, 0.2, 0.0, 0.0, // Lower velocity error cost
    0.0, 0.0, 10.0, 0.0, // Same angle error cost
    0.0, 0.0, 0.0, 0.5 // Lower angular velocity error cost
};

float R[CONTROL_SIZE * CONTROL_SIZE] = {
    0.1 // Cost on control effort
};

float R_smooth[CONTROL_SIZE * CONTROL_SIZE] = {
    0.5 // Higher cost on control effort for smoother motion
};

// Controller and state variables
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);
float stateError[STATE_SIZE] = { 0, 0, 0, 0 }; // Error state
float controlInput[CONTROL_SIZE] = { 0 }; // Control output
float precomputedK[CONTROL_SIZE * STATE_SIZE]; // For storing exported gains

// Simulated system variables
float cartPosition = 0.0; // x
float cartVelocity = 0.0; // x_dot
float pendulumAngle = 0.1; // theta (start with slight tilt)
float pendulumAngVelocity = 0.0; // theta_dot
float desiredStates[STATE_SIZE] = { 0, 0, 0, 0 }; // Desired states [x, x_dot, theta, theta_dot]

// System operation variables
enum ControlMode { BALANCE,
    POSITION_TRACKING,
    ENERGY_SAVING,
    SWING_UP };
ControlMode currentMode = BALANCE;
int updateCount = 0;
float disturbanceForce = 0.0;
bool disturbanceActive = false;
int disturbanceCountdown = 0;
bool systemIdentMode = false;
float identResults[4] = { 0 }; // For storing identified parameters
bool useFallbackGains = false;

// Performance metrics
float totalAngleError = 0.0;
float maxAngleError = 0.0;
float totalEnergyUsed = 0.0;
int recoveryCount = 0;
unsigned long startTime = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("Enhanced Inverted Pendulum LQR Control Example");
    Serial.println("Commands: '1'-'4' - Change control mode, 'd' - Apply disturbance,");
    Serial.println("          'i' - Run system identification, 'f' - Toggle fallback gains");

    // Initialize controller with system dynamics
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);

    // Compute optimal gains
    if (controller.computeGains()) {
        Serial.println("LQR gains computed successfully");

        // Export gains for later use
        controller.exportGains(precomputedK);
        Serial.println("Gain matrix K:");
        for (int i = 0; i < CONTROL_SIZE * STATE_SIZE; i++) {
            Serial.print(precomputedK[i], 4);
            Serial.print(" ");
        }
        Serial.println();

        // Get Riccati solution
        const float* P = controller.getRicattiSolution();
        Serial.println("Riccati solution trace (stability measure):");
        Serial.println(P[0] + P[5] + P[10] + P[15], 4);

        // Estimate convergence time
        float convTime = controller.estimateConvergenceTime(0.1);
        if (convTime > 0) {
            Serial.print("Estimated angle recovery time: ");
            Serial.print(convTime * dt);
            Serial.println(" seconds");
        }
    } else {
        Serial.println("Failed to compute LQR gains, using pre-computed gains");

        // Fallback to pre-computed gains
        float fallbackK[CONTROL_SIZE * STATE_SIZE] = {
            -1.0, -1.5, 20.0, 3.5 // Example gains
        };
        controller.setGains(fallbackK);
        useFallbackGains = true;
    }

    startTime = millis();
}

// System identification function for parameter estimation
void runSystemIdentification()
{
    Serial.println("Running system identification...");

    // Reset system state
    cartPosition = 0.0;
    cartVelocity = 0.0;
    pendulumAngle = 0.1;
    pendulumAngVelocity = 0.0;

    float originalDamping = damping;

    // Apply known input and measure response
    for (int i = 0; i < 200; i++) {
        // Apply sinusoidal input to excite dynamics
        float force = sin(i * 0.1) * 2.0;

        // Update system states with known input
        cartVelocity += (force - damping * cartVelocity + m * pendulumAngle) * dt / M;
        cartPosition += cartVelocity * dt;
        pendulumAngVelocity += (g * sin(pendulumAngle) + force * cos(pendulumAngle) / m) * dt / l;
        pendulumAngle += pendulumAngVelocity * dt;

        // At specific points, record system response
        if (i == 50 || i == 100 || i == 150) {
            Serial.print("Position: ");
            Serial.print(cartPosition);
            Serial.print(", Angle: ");
            Serial.println(pendulumAngle);
        }
    }

    // Simple parameter estimation (demonstration)
    float estDamping = damping * (0.9 + random(0, 20) / 100.0); // With some noise
    float estMass = M * (0.95 + random(0, 10) / 100.0);

    Serial.print("Estimated damping: ");
    Serial.print(estDamping, 4);
    Serial.print(" (actual: ");
    Serial.print(originalDamping, 4);
    Serial.println(")");

    Serial.print("Estimated cart mass: ");
    Serial.print(estMass, 4);
    Serial.print(" (actual: ");
    Serial.print(M, 4);
    Serial.println(")");

    // Store identified parameters
    identResults[0] = estDamping;
    identResults[1] = estMass;

    // Reset system
    cartPosition = 0.0;
    cartVelocity = 0.0;
    pendulumAngle = 0.1;
    pendulumAngVelocity = 0.0;
}

// Swing-up controller for large angles
float calculateSwingUpControl()
{
    // Energy-based swing-up control
    float energy = 0.5 * m * l * l * pendulumAngVelocity * pendulumAngVelocity + m * g * l * (cos(pendulumAngle) - 1);
    float desiredEnergy = 0; // Energy at upright position
    float energyError = desiredEnergy - energy;

    // Control based on energy pumping
    return 2.0 * sign(energyError * pendulumAngVelocity * cos(pendulumAngle));
}

// Helper function for swing-up controller
int sign(float x)
{
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

void loop()
{
    // Process any incoming commands
    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
        case '1': // Balance mode
            currentMode = BALANCE;
            controller.setCostMatrices(Q, R);
            controller.computeGains();
            Serial.println("Mode: Balance");
            break;

        case '2': // Position tracking mode
            currentMode = POSITION_TRACKING;
            controller.setCostMatrices(Q_position, R);
            controller.computeGains();
            // Set new desired position
            desiredStates[0] = 1.0; // Move cart to position 1.0
            Serial.println("Mode: Position Tracking");
            break;

        case '3': // Energy saving mode
            currentMode = ENERGY_SAVING;
            controller.setCostMatrices(Q_energy, R_smooth);
            controller.computeGains();
            Serial.println("Mode: Energy Saving");
            break;

        case '4': // Swing-up mode
            currentMode = SWING_UP;
            pendulumAngle = 3.14159; // Start from bottom position
            pendulumAngVelocity = 0;
            Serial.println("Mode: Swing-up");
            break;

        case 'd': // Apply disturbance
            disturbanceActive = true;
            disturbanceCountdown = 30; // 30 time steps
            disturbanceForce = 5.0; // Force of 5N
            Serial.println("Disturbance applied!");
            break;

        case 'i': // Run system identification
            systemIdentMode = true;
            Serial.println("Starting system identification...");
            break;

        case 'f': // Toggle fallback gains
            useFallbackGains = !useFallbackGains;
            if (useFallbackGains) {
                float fallbackK[CONTROL_SIZE * STATE_SIZE] = {
                    -1.0, -1.5, 20.0, 3.5 // Example gains
                };
                controller.setGains(fallbackK);
                Serial.println("Using fallback gains");
            } else {
                controller.setGains(precomputedK);
                Serial.println("Using computed optimal gains");
            }
            break;
        }
    }

    // Run system identification if requested
    if (systemIdentMode) {
        runSystemIdentification();
        systemIdentMode = false;
        return; // Skip this control cycle
    }

    // Calculate state error
    stateError[0] = cartPosition - desiredStates[0];
    stateError[1] = cartVelocity - desiredStates[1];
    stateError[2] = pendulumAngle - desiredStates[2];
    stateError[3] = pendulumAngVelocity - desiredStates[3];

    // Decide on control strategy based on mode
    float force = 0.0;

    if (currentMode == SWING_UP && fabs(pendulumAngle) > 0.3) {
        // Use swing-up controller for large angles
        force = calculateSwingUpControl();
    } else {
        // Use LQR for balancing or when pendulum is near upright
        // Update controller with current state
        controller.updateState(stateError);

        // Calculate control input
        controller.calculateControl(controlInput);
        force = controlInput[0];

        // If we were in swing-up and now switched to LQR, count it as a recovery
        if (currentMode == SWING_UP && fabs(pendulumAngle) <= 0.3) {
            recoveryCount++;
            Serial.println("Pendulum recovered to upright position!");
        }
    }

    // Apply disturbance if active
    if (disturbanceActive) {
        force += disturbanceForce * (float)disturbanceCountdown / 30.0;
        disturbanceCountdown--;
        if (disturbanceCountdown <= 0) {
            disturbanceActive = false;
        }
    }

    // Update system states (simplified simulation)
    cartVelocity += (force - damping * cartVelocity + m * sin(pendulumAngle)) * dt / M;
    cartPosition += cartVelocity * dt;

    pendulumAngVelocity += (g * sin(pendulumAngle) + force * cos(pendulumAngle) / m) * dt / l;
    pendulumAngle += pendulumAngVelocity * dt;

    // Normalize angle to [-π, π]
    while (pendulumAngle > M_PI)
        pendulumAngle -= 2 * M_PI;
    while (pendulumAngle < -M_PI)
        pendulumAngle += 2 * M_PI;

    // Update performance metrics
    updateCount++;
    float absAngleError = fabs(stateError[2]);
    totalAngleError += absAngleError;
    if (absAngleError > maxAngleError)
        maxAngleError = absAngleError;
    totalEnergyUsed += fabs(force * cartVelocity) * dt; // Force × velocity × time

    // Calculate expected cost
    float expectedCost = controller.calculateExpectedCost();

    // Print current states (every 10 iterations to avoid flooding serial)
    if (updateCount % 10 == 0) {
        Serial.print("Pos: ");
        Serial.print(cartPosition, 3);
        Serial.print(", Vel: ");
        Serial.print(cartVelocity, 3);
        Serial.print(", Angle: ");
        Serial.print(pendulumAngle, 3);
        Serial.print(", Control: ");
        Serial.print(force, 3);

        // Every 50 updates, print additional stats
        if (updateCount % 50 == 0) {
            Serial.print(" | Mode: ");
            switch (currentMode) {
            case BALANCE:
                Serial.print("Balance");
                break;
            case POSITION_TRACKING:
                Serial.print("Position");
                break;
            case ENERGY_SAVING:
                Serial.print("EnergySave");
                break;
            case SWING_UP:
                Serial.print("SwingUp");
                break;
            }

            Serial.print(", AvgErr: ");
            Serial.print(totalAngleError / updateCount, 4);
            Serial.print(", Energy: ");
            Serial.print(totalEnergyUsed, 2);
            Serial.print(", Recoveries: ");
            Serial.print(recoveryCount);

            if (expectedCost >= 0) {
                Serial.print(", Cost: ");
                Serial.print(expectedCost, 2);
            }

            // Runtime
            float runtime = (millis() - startTime) / 1000.0;
            Serial.print(", Time: ");
            Serial.print(runtime, 1);
            Serial.print("s");
        }

        Serial.println();
    }

    delay(10); // Update at 100Hz
}
