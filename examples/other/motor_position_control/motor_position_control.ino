#include <AutoLQR.h>

// Pin definitions
#define MOTOR_PWM_PIN 9 // PWM output to motor driver
#define MOTOR_DIR_PIN 8 // Direction control pin
#define ENCODER_A_PIN 2 // Encoder A channel (interrupt)
#define ENCODER_B_PIN 3 // Encoder B channel (interrupt)
#define TARGET_POT_PIN A0 // Potentiometer for target position

// System parameters
#define STATE_SIZE 2 // Position and velocity
#define CONTROL_SIZE 1 // Single PWM control input
#define CONTROL_FREQUENCY 50 // Control loop frequency in Hz
#define MAX_PWM 255 // Maximum PWM value
#define COUNTS_PER_REV 1200 // Encoder counts per revolution
#define GEAR_RATIO 19.2 // Motor gear ratio (if applicable)

// Interrupt-based encoder counter
volatile long encoderCount = 0;
long lastEncoderCount = 0;
unsigned long lastTime = 0;

// LQR controller instance
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);

// Current state variables
float currentPosition = 0.0; // Position in revolutions
float currentVelocity = 0.0; // Velocity in revolutions per second
float targetPosition = 0.0; // Target position in revolutions

// System matrices
float A[STATE_SIZE * STATE_SIZE]; // State matrix
float B[STATE_SIZE * CONTROL_SIZE]; // Input matrix
float Q[STATE_SIZE * STATE_SIZE]; // State cost matrix
float R[CONTROL_SIZE * CONTROL_SIZE]; // Control cost matrix

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("AutoLQR Motor Position Control Example");

    // Set up motor control pins
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(TARGET_POT_PIN, INPUT);

    // Set up encoder pins with interrupts
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, CHANGE);

    // Initialize system matrices
    setupSystem();

    // Configure controller
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);

    // Compute optimal control gains
    if (controller.computeGains()) {
        Serial.println("LQR gain computation successful");

        // Print calculated gains
        float K[CONTROL_SIZE * STATE_SIZE];
        controller.exportGains(K);
        Serial.print("K = [");
        Serial.print(K[0], 4);
        Serial.print(", ");
        Serial.print(K[1], 4);
        Serial.println("]");

        // Estimate convergence time
        float convergenceTime = controller.estimateConvergenceTime(0.05);
        Serial.print("Estimated convergence time: ");
        Serial.print(convergenceTime, 2);
        Serial.println(" seconds");
    } else {
        Serial.println("Error: Failed to compute LQR gains");
    }

    // Initialize timing
    lastTime = millis();
}

void loop()
{
    // Control loop timing at specified frequency
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= (1000 / CONTROL_FREQUENCY)) {
        // Read target position from potentiometer
        int potValue = analogRead(TARGET_POT_PIN);
        targetPosition = map(potValue, 0, 1023, 0, 5) * 1.0; // Map to 0-5 revolutions

        // Update current state based on encoder feedback
        updateState();

        // Calculate control output
        calculateControl();

        // Log data
        logData();

        // Update timing
        lastTime = currentTime;
    }
}

void setupSystem()
{
    // Sample time for discretization
    float dt = 1.0f / CONTROL_FREQUENCY;

    // Discrete-time state matrix (position-velocity system)
    // x(k+1) = A * x(k) + B * u(k)
    A[0] = 1.0f; // Position += velocity * dt
    A[1] = dt; // Time step
    A[2] = 0.0f; // Velocity is not directly affected by position
    A[3] = 0.98f; // Velocity damping factor (experimentally determined)

    // Input matrix - effect of control input on state
    B[0] = 0.5f * dt * dt; // Control effect on position (acceleration * dt²/2)
    B[1] = dt; // Control effect on velocity (acceleration * dt)

    // State cost matrix (higher values = faster response but more oscillation)
    Q[0] = 100.0f; // Position error cost
    Q[1] = 0.0f; // No cross-coupling
    Q[2] = 0.0f; // No cross-coupling
    Q[3] = 10.0f; // Velocity error cost

    // Control cost matrix (higher values = smoother control but slower response)
    R[0] = 1.0f;
}

void updateState()
{
    // Calculate position in revolutions
    currentPosition = (float)encoderCount / (COUNTS_PER_REV * GEAR_RATIO);

    // Calculate velocity (revolutions per second)
    long encoderDelta = encoderCount - lastEncoderCount;
    float dt = (millis() - lastTime) / 1000.0f;
    currentVelocity = (float)encoderDelta / (COUNTS_PER_REV * GEAR_RATIO) / dt;

    // Store encoder count for next update
    lastEncoderCount = encoderCount;

    // Create state error vector (current - target)
    float stateError[STATE_SIZE] = {
        currentPosition - targetPosition,
        currentVelocity - 0.0f // Target velocity is zero (steady state)
    };

    // Update controller with current state error
    controller.updateState(stateError);
}

void calculateControl()
{
    // Calculate optimal control input
    float control[CONTROL_SIZE];
    controller.calculateControl(control);

    // Scale and apply control output to motor
    int pwmOutput = constrain(abs(control[0] * MAX_PWM), 0, MAX_PWM);

    // Set motor direction
    if (control[0] > 0) {
        digitalWrite(MOTOR_DIR_PIN, HIGH); // Forward
    } else {
        digitalWrite(MOTOR_DIR_PIN, LOW); // Reverse
    }

    // Apply PWM value
    analogWrite(MOTOR_PWM_PIN, pwmOutput);
}

void logData()
{
    // Print current state and control information
    Serial.print("Target: ");
    Serial.print(targetPosition, 2);
    Serial.print(" rev, Current: ");
    Serial.print(currentPosition, 2);
    Serial.print(" rev, Velocity: ");
    Serial.print(currentVelocity, 2);
    Serial.print(" rev/s, Error: ");
    Serial.print(currentPosition - targetPosition, 2);
    Serial.print(" rev, Cost: ");
    Serial.println(controller.calculateExpectedCost(), 2);
}

// Encoder interrupt service routine
void encoderISR()
{
    // Read both encoder pins
    bool A = digitalRead(ENCODER_A_PIN);
    bool B = digitalRead(ENCODER_B_PIN);

    // Increment or decrement based on rotation direction
    if (A == B) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}
