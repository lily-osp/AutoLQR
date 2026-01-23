/**
 * Delta Robot 3-Motor Control Example using AutoLQR
 * 
 * This example demonstrates how to control 3 independent DC motors
 * for a Delta Robot using separate AutoLQR instances for each motor.
 * 
 * Architecture:
 *   [Inverse Kinematics] --> [Motor 1 LQR] --> PWM1
 *   (X,Y,Z target)       --> [Motor 2 LQR] --> PWM2
 *                        --> [Motor 3 LQR] --> PWM3
 * 
 * Hardware Requirements:
 *   - 3x DC geared motors with encoders
 *   - 3x Motor drivers (H-bridge or similar)
 *   - Arduino Mega or similar (for sufficient pins and memory)
 */

#include <AutoLQR.h>

// System configuration
#define STATE_SIZE 2      // Position and velocity per motor
#define CONTROL_SIZE 1    // Single control input per motor
#define NUM_MOTORS 3      // Number of motors in Delta Robot
#define CONTROL_FREQ 50   // Control loop frequency (Hz)
#define MAX_PWM 255       // Maximum PWM value

// Motor 1 pins
#define M1_PWM_PIN 9
#define M1_DIR_PIN 8
#define M1_ENC_A 2
#define M1_ENC_B 22

// Motor 2 pins
#define M2_PWM_PIN 10
#define M2_DIR_PIN 11
#define M2_ENC_A 3
#define M2_ENC_B 23

// Motor 3 pins
#define M3_PWM_PIN 12
#define M3_DIR_PIN 13
#define M3_ENC_A 18
#define M3_ENC_B 24

// Encoder and motor parameters
#define COUNTS_PER_REV 1200   // Encoder counts per revolution
#define GEAR_RATIO 19.2       // Motor gear ratio

// Delta robot geometry (mm)
#define BASE_RADIUS 100.0     // Distance from center to motor axis
#define EFFECTOR_RADIUS 30.0  // Distance from center to effector joint
#define ARM_LENGTH 150.0      // Upper arm length
#define FOREARM_LENGTH 250.0  // Lower arm (rod) length

// Create 3 independent LQR controllers
AutoLQR motor1Controller(STATE_SIZE, CONTROL_SIZE);
AutoLQR motor2Controller(STATE_SIZE, CONTROL_SIZE);
AutoLQR motor3Controller(STATE_SIZE, CONTROL_SIZE);

// Encoder counters (volatile for ISR access)
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
volatile long encoder3Count = 0;

// Previous encoder counts for velocity calculation
long lastEncoder1Count = 0;
long lastEncoder2Count = 0;
long lastEncoder3Count = 0;

// Current motor states [position (rad), velocity (rad/s)]
float motor1State[STATE_SIZE] = {0, 0};
float motor2State[STATE_SIZE] = {0, 0};
float motor3State[STATE_SIZE] = {0, 0};

// Target joint angles (radians)
float targetAngle1 = 0.0;
float targetAngle2 = 0.0;
float targetAngle3 = 0.0;

// System matrices (shared by all motors since they are identical)
float A[STATE_SIZE * STATE_SIZE];
float B[STATE_SIZE * CONTROL_SIZE];
float Q[STATE_SIZE * STATE_SIZE];
float R[CONTROL_SIZE * CONTROL_SIZE];

// Timing
unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Delta Robot 3-Motor LQR Control");
    Serial.println("==============================");
    
    // Initialize motor pins
    setupMotorPins();
    
    // Setup encoder interrupts
    setupEncoders();
    
    // Initialize system matrices
    setupSystemMatrices();
    
    // Configure all three controllers with identical matrices
    configureControllers();
    
    // Compute optimal gains
    if (computeAllGains()) {
        Serial.println("All LQR gains computed successfully");
        printGains();
    } else {
        Serial.println("Error: Failed to compute LQR gains");
        while (1); // Halt on error
    }
    
    lastControlTime = millis();
    lastPrintTime = millis();
    
    Serial.println("\nSend coordinates as: X,Y,Z (in mm)");
    Serial.println("Example: 0,0,-200");
}

void loop() {
    unsigned long currentTime = millis();
    
    // Control loop at specified frequency
    if (currentTime - lastControlTime >= (1000 / CONTROL_FREQ)) {
        // Update motor states from encoders
        updateMotorStates();
        
        // Calculate and apply control for each motor
        controlMotor1();
        controlMotor2();
        controlMotor3();
        
        lastControlTime = currentTime;
    }
    
    // Print status every 500ms
    if (currentTime - lastPrintTime >= 500) {
        printStatus();
        lastPrintTime = currentTime;
    }
    
    // Check for serial commands
    processSerialInput();
}

void setupMotorPins() {
    // Motor 1
    pinMode(M1_PWM_PIN, OUTPUT);
    pinMode(M1_DIR_PIN, OUTPUT);
    
    // Motor 2
    pinMode(M2_PWM_PIN, OUTPUT);
    pinMode(M2_DIR_PIN, OUTPUT);
    
    // Motor 3
    pinMode(M3_PWM_PIN, OUTPUT);
    pinMode(M3_DIR_PIN, OUTPUT);
    
    // Stop all motors initially
    analogWrite(M1_PWM_PIN, 0);
    analogWrite(M2_PWM_PIN, 0);
    analogWrite(M3_PWM_PIN, 0);
}

void setupEncoders() {
    // Encoder pins
    pinMode(M1_ENC_A, INPUT_PULLUP);
    pinMode(M1_ENC_B, INPUT_PULLUP);
    pinMode(M2_ENC_A, INPUT_PULLUP);
    pinMode(M2_ENC_B, INPUT_PULLUP);
    pinMode(M3_ENC_A, INPUT_PULLUP);
    pinMode(M3_ENC_B, INPUT_PULLUP);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), encoder2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M3_ENC_A), encoder3ISR, CHANGE);
}

void setupSystemMatrices() {
    float dt = 1.0f / CONTROL_FREQ;
    
    // Discrete-time state matrix A
    // x(k+1) = A * x(k) + B * u(k)
    // State: [position, velocity]
    A[0] = 1.0f;    // position += velocity * dt
    A[1] = dt;      // time step
    A[2] = 0.0f;    // velocity not directly affected by position
    A[3] = 0.95f;   // velocity damping (friction/back-EMF)
    
    // Input matrix B
    // Effect of control input (voltage/PWM) on state
    B[0] = 0.5f * dt * dt;  // effect on position
    B[1] = dt;              // effect on velocity
    
    // State cost matrix Q (diagonal)
    // Higher values = more aggressive tracking
    Q[0] = 150.0f;  // position error cost
    Q[1] = 0.0f;    // no cross-coupling
    Q[2] = 0.0f;    // no cross-coupling
    Q[3] = 15.0f;   // velocity error cost
    
    // Control cost matrix R
    // Higher values = smoother control, slower response
    R[0] = 1.0f;
}

void configureControllers() {
    // Motor 1
    motor1Controller.setStateMatrix(A);
    motor1Controller.setInputMatrix(B);
    motor1Controller.setCostMatrices(Q, R);
    
    // Motor 2
    motor2Controller.setStateMatrix(A);
    motor2Controller.setInputMatrix(B);
    motor2Controller.setCostMatrices(Q, R);
    
    // Motor 3
    motor3Controller.setStateMatrix(A);
    motor3Controller.setInputMatrix(B);
    motor3Controller.setCostMatrices(Q, R);
}

bool computeAllGains() {
    bool success = true;
    
    success &= motor1Controller.computeGains();
    success &= motor2Controller.computeGains();
    success &= motor3Controller.computeGains();
    
    return success;
}

void printGains() {
    float K[CONTROL_SIZE * STATE_SIZE];
    
    motor1Controller.exportGains(K);
    Serial.print("Motor 1 K = [");
    Serial.print(K[0], 4);
    Serial.print(", ");
    Serial.print(K[1], 4);
    Serial.println("]");
    
    // Motors 2 and 3 have identical gains since matrices are the same
    Serial.println("Motors 2 & 3 have identical gains (same system)");
}

void updateMotorStates() {
    float dt = 1.0f / CONTROL_FREQ;
    
    // Motor 1 state
    float pos1 = encoderToRadians(encoder1Count);
    float vel1 = (encoder1Count - lastEncoder1Count) / (COUNTS_PER_REV * GEAR_RATIO) * 2.0f * PI / dt;
    motor1State[0] = pos1 - targetAngle1;  // position error
    motor1State[1] = vel1;                  // velocity (target velocity = 0)
    lastEncoder1Count = encoder1Count;
    
    // Motor 2 state
    float pos2 = encoderToRadians(encoder2Count);
    float vel2 = (encoder2Count - lastEncoder2Count) / (COUNTS_PER_REV * GEAR_RATIO) * 2.0f * PI / dt;
    motor2State[0] = pos2 - targetAngle2;
    motor2State[1] = vel2;
    lastEncoder2Count = encoder2Count;
    
    // Motor 3 state
    float pos3 = encoderToRadians(encoder3Count);
    float vel3 = (encoder3Count - lastEncoder3Count) / (COUNTS_PER_REV * GEAR_RATIO) * 2.0f * PI / dt;
    motor3State[0] = pos3 - targetAngle3;
    motor3State[1] = vel3;
    lastEncoder3Count = encoder3Count;
}

float encoderToRadians(long counts) {
    return (float)counts / (COUNTS_PER_REV * GEAR_RATIO) * 2.0f * PI;
}

void controlMotor1() {
    motor1Controller.updateState(motor1State);
    
    float control[CONTROL_SIZE];
    motor1Controller.calculateControl(control);
    
    applyMotorControl(M1_PWM_PIN, M1_DIR_PIN, control[0]);
}

void controlMotor2() {
    motor2Controller.updateState(motor2State);
    
    float control[CONTROL_SIZE];
    motor2Controller.calculateControl(control);
    
    applyMotorControl(M2_PWM_PIN, M2_DIR_PIN, control[0]);
}

void controlMotor3() {
    motor3Controller.updateState(motor3State);
    
    float control[CONTROL_SIZE];
    motor3Controller.calculateControl(control);
    
    applyMotorControl(M3_PWM_PIN, M3_DIR_PIN, control[0]);
}

void applyMotorControl(int pwmPin, int dirPin, float control) {
    int pwmValue = constrain(abs(control * MAX_PWM), 0, MAX_PWM);
    
    if (control > 0) {
        digitalWrite(dirPin, HIGH);
    } else {
        digitalWrite(dirPin, LOW);
    }
    
    analogWrite(pwmPin, pwmValue);
}

// Delta Robot Inverse Kinematics
// Converts Cartesian coordinates (X, Y, Z) to joint angles (theta1, theta2, theta3)
bool inverseKinematics(float x, float y, float z, float* theta1, float* theta2, float* theta3) {
    // Delta robot inverse kinematics
    // Reference frame: Z pointing up, origin at center of base
    
    float e = EFFECTOR_RADIUS;
    float f = BASE_RADIUS;
    float re = FOREARM_LENGTH;
    float rf = ARM_LENGTH;
    
    float sqrt3 = sqrt(3.0f);
    float pi = PI;
    float sin120 = sqrt3 / 2.0f;
    float cos120 = -0.5f;
    float tan60 = sqrt3;
    float sin30 = 0.5f;
    float tan30 = 1.0f / sqrt3;
    
    // Theta 1 (motor at 0 degrees)
    float y1 = -0.5f * tan30 * f;
    float y1_prime = y - 0.5f * tan30 * e;
    
    float a1 = (x*x + y1_prime*y1_prime + z*z + rf*rf - re*re - y1*y1) / (2.0f * z);
    float b1 = (y1 - y1_prime) / z;
    
    float disc1 = -(a1 + b1*y1) * (a1 + b1*y1) + rf * (b1*b1*rf + rf);
    if (disc1 < 0) return false;
    
    float yj1 = (y1 - a1*b1 - sqrt(disc1)) / (b1*b1 + 1);
    float zj1 = a1 + b1*yj1;
    *theta1 = atan2(-zj1, (y1 - yj1));
    
    // Theta 2 (motor at 120 degrees)
    float x2 = x * cos120 + y * sin120;
    float y2 = y * cos120 - x * sin120;
    float y2_prime = y2 - 0.5f * tan30 * e;
    
    float a2 = (x2*x2 + y2_prime*y2_prime + z*z + rf*rf - re*re - y1*y1) / (2.0f * z);
    float b2 = (y1 - y2_prime) / z;
    
    float disc2 = -(a2 + b2*y1) * (a2 + b2*y1) + rf * (b2*b2*rf + rf);
    if (disc2 < 0) return false;
    
    float yj2 = (y1 - a2*b2 - sqrt(disc2)) / (b2*b2 + 1);
    float zj2 = a2 + b2*yj2;
    *theta2 = atan2(-zj2, (y1 - yj2));
    
    // Theta 3 (motor at 240 degrees)
    float x3 = x * cos120 - y * sin120;
    float y3 = y * cos120 + x * sin120;
    float y3_prime = y3 - 0.5f * tan30 * e;
    
    float a3 = (x3*x3 + y3_prime*y3_prime + z*z + rf*rf - re*re - y1*y1) / (2.0f * z);
    float b3 = (y1 - y3_prime) / z;
    
    float disc3 = -(a3 + b3*y1) * (a3 + b3*y1) + rf * (b3*b3*rf + rf);
    if (disc3 < 0) return false;
    
    float yj3 = (y1 - a3*b3 - sqrt(disc3)) / (b3*b3 + 1);
    float zj3 = a3 + b3*yj3;
    *theta3 = atan2(-zj3, (y1 - yj3));
    
    return true;
}

void processSerialInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        // Parse X,Y,Z coordinates
        int comma1 = input.indexOf(',');
        int comma2 = input.indexOf(',', comma1 + 1);
        
        if (comma1 > 0 && comma2 > comma1) {
            float x = input.substring(0, comma1).toFloat();
            float y = input.substring(comma1 + 1, comma2).toFloat();
            float z = input.substring(comma2 + 1).toFloat();
            
            Serial.print("Target position: X=");
            Serial.print(x);
            Serial.print(", Y=");
            Serial.print(y);
            Serial.print(", Z=");
            Serial.println(z);
            
            // Calculate inverse kinematics
            float t1, t2, t3;
            if (inverseKinematics(x, y, z, &t1, &t2, &t3)) {
                targetAngle1 = t1;
                targetAngle2 = t2;
                targetAngle3 = t3;
                
                Serial.print("Joint angles (deg): ");
                Serial.print(t1 * 180.0f / PI);
                Serial.print(", ");
                Serial.print(t2 * 180.0f / PI);
                Serial.print(", ");
                Serial.println(t3 * 180.0f / PI);
            } else {
                Serial.println("Error: Position out of workspace");
            }
        }
    }
}

void printStatus() {
    Serial.print("Pos(deg): ");
    Serial.print(encoderToRadians(encoder1Count) * 180.0f / PI, 1);
    Serial.print(", ");
    Serial.print(encoderToRadians(encoder2Count) * 180.0f / PI, 1);
    Serial.print(", ");
    Serial.print(encoderToRadians(encoder3Count) * 180.0f / PI, 1);
    
    Serial.print(" | Target: ");
    Serial.print(targetAngle1 * 180.0f / PI, 1);
    Serial.print(", ");
    Serial.print(targetAngle2 * 180.0f / PI, 1);
    Serial.print(", ");
    Serial.print(targetAngle3 * 180.0f / PI, 1);
    
    Serial.print(" | Err: ");
    Serial.print(motor1State[0] * 180.0f / PI, 2);
    Serial.print(", ");
    Serial.print(motor2State[0] * 180.0f / PI, 2);
    Serial.print(", ");
    Serial.println(motor3State[0] * 180.0f / PI, 2);
}

// Encoder interrupt service routines
void encoder1ISR() {
    bool A = digitalRead(M1_ENC_A);
    bool B = digitalRead(M1_ENC_B);
    encoder1Count += (A == B) ? 1 : -1;
}

void encoder2ISR() {
    bool A = digitalRead(M2_ENC_A);
    bool B = digitalRead(M2_ENC_B);
    encoder2Count += (A == B) ? 1 : -1;
}

void encoder3ISR() {
    bool A = digitalRead(M3_ENC_A);
    bool B = digitalRead(M3_ENC_B);
    encoder3Count += (A == B) ? 1 : -1;
}
