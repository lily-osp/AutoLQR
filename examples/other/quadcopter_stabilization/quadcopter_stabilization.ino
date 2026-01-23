/**
 * Quadcopter Single-Axis Stabilization Example using AutoLQR
 * 
 * This example demonstrates LQR-based attitude control for a single axis
 * (roll or pitch) of a quadcopter or similar multirotor vehicle.
 * 
 * System Model:
 *   State: [angle, angular_velocity]
 *   Control: motor differential thrust
 * 
 * Hardware Requirements:
 *   - IMU sensor (MPU6050, BNO055, etc.)
 *   - ESC and brushless motors
 *   - Arduino with sufficient processing power
 * 
 * Note: This is a simplified single-axis example for educational purposes.
 * Real flight controllers require full 3-axis control with sensor fusion.
 */

#include <AutoLQR.h>
#include <Wire.h>

// System configuration
#define STATE_SIZE 2       // Angle and angular velocity
#define CONTROL_SIZE 1     // Single control output (differential thrust)
#define CONTROL_FREQ 250   // Control loop frequency (Hz)

// Pin definitions (adjust for your hardware)
#define MOTOR_LEFT_PIN 9    // Left motor ESC PWM
#define MOTOR_RIGHT_PIN 10  // Right motor ESC PWM
#define LED_PIN 13          // Status LED

// Motor PWM range (standard ESC signal)
#define PWM_MIN 1000        // Minimum pulse width (us)
#define PWM_MAX 2000        // Maximum pulse width (us)
#define PWM_ARM 1100        // Arming pulse width
#define THROTTLE_IDLE 1200  // Idle throttle

// IMU I2C address (MPU6050)
#define IMU_ADDR 0x68

// Physical parameters
const float maxAngle = 45.0f;       // Maximum allowed angle (degrees)
const float maxAngularRate = 300.0f; // Maximum angular rate (deg/s)
const float momentOfInertia = 0.01f; // kg*m^2 (estimate)
const float armLength = 0.15f;       // Motor arm length (m)
const float thrustCoeff = 0.00001f;  // Thrust per PWM unit

// LQR controller
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);

// System matrices
float A[STATE_SIZE * STATE_SIZE];
float B[STATE_SIZE * CONTROL_SIZE];
float Q[STATE_SIZE * STATE_SIZE];
float R[CONTROL_SIZE * CONTROL_SIZE];

// State variables
float currentAngle = 0.0f;       // Current roll/pitch angle (degrees)
float currentRate = 0.0f;        // Current angular rate (deg/s)
float targetAngle = 0.0f;        // Target angle (degrees)
float gyroOffset = 0.0f;         // Gyro calibration offset

// Control variables
float baseThrottle = THROTTLE_IDLE;
float controlOutput = 0.0f;
int leftMotorPWM = PWM_MIN;
int rightMotorPWM = PWM_MIN;

// System state
bool armed = false;
bool imuCalibrated = false;
unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;
int cycleCount = 0;

// Complementary filter parameters
const float filterAlpha = 0.98f;  // Gyro weight in complementary filter
float accelAngle = 0.0f;

void setup() {
    Serial.begin(115200);
    Serial.println("Quadcopter Single-Axis Stabilization - AutoLQR");
    Serial.println("=============================================");
    
    // Initialize pins
    pinMode(MOTOR_LEFT_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    
    // Start with motors off
    setMotorPWM(MOTOR_LEFT_PIN, PWM_MIN);
    setMotorPWM(MOTOR_RIGHT_PIN, PWM_MIN);
    
    // Initialize I2C and IMU
    Wire.begin();
    initIMU();
    
    // Calibrate gyro
    calibrateGyro();
    
    // Initialize system matrices
    setupSystemMatrices();
    
    // Configure controller
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);
    
    // Compute optimal gains
    if (controller.computeGains()) {
        Serial.println("LQR gains computed successfully");
        
        float K[CONTROL_SIZE * STATE_SIZE];
        controller.exportGains(K);
        Serial.print("K = [");
        Serial.print(K[0], 4);
        Serial.print(", ");
        Serial.print(K[1], 4);
        Serial.println("]");
        
        // Check controllability
        if (controller.isSystemControllable()) {
            Serial.println("System is controllable");
        }
        
        float convTime = controller.estimateConvergenceTime(0.05);
        if (convTime > 0) {
            Serial.print("Estimated response time: ");
            Serial.print(convTime * 1000.0f / CONTROL_FREQ);
            Serial.println(" ms");
        }
    } else {
        Serial.println("Error: Failed to compute LQR gains");
    }
    
    Serial.println("\nCommands:");
    Serial.println("  A - Arm motors");
    Serial.println("  D - Disarm motors");
    Serial.println("  T[value] - Set throttle (1200-1800)");
    Serial.println("  R[angle] - Set target angle (-30 to 30)");
    Serial.println("  C - Recalibrate gyro");
    Serial.println("  S - Print status");
    
    lastControlTime = micros();
    lastPrintTime = millis();
}

void loop() {
    unsigned long currentMicros = micros();
    
    // High-frequency control loop
    if (currentMicros - lastControlTime >= (1000000 / CONTROL_FREQ)) {
        float dt = (currentMicros - lastControlTime) / 1000000.0f;
        
        // Read IMU data
        readIMU(dt);
        
        // Safety check
        if (armed && (abs(currentAngle) > maxAngle || abs(currentRate) > maxAngularRate)) {
            disarm();
            Serial.println("SAFETY: Angle/rate limit exceeded - disarmed");
        }
        
        // Calculate and apply control
        if (armed) {
            calculateControl();
            applyControl();
        }
        
        cycleCount++;
        lastControlTime = currentMicros;
    }
    
    // Low-frequency status print
    if (millis() - lastPrintTime >= 100) {
        printStatus();
        lastPrintTime = millis();
    }
    
    // Process serial commands
    processSerialInput();
}

void setupSystemMatrices() {
    float dt = 1.0f / CONTROL_FREQ;
    
    // Discrete-time angular dynamics model
    // State: [angle (deg), angular_velocity (deg/s)]
    // 
    // angle(k+1) = angle(k) + angular_velocity(k) * dt
    // angular_velocity(k+1) = angular_velocity(k) * damping + control * gain
    
    float dampingFactor = 0.98f;  // Angular rate damping (drag)
    
    A[0] = 1.0f;           // Angle persists
    A[1] = dt;             // Rate affects angle
    A[2] = 0.0f;           // Angle doesn't directly affect rate
    A[3] = dampingFactor;  // Rate damping
    
    // Control input effect
    // Control signal produces torque -> angular acceleration
    float torqueGain = (armLength * thrustCoeff) / momentOfInertia;
    torqueGain *= (180.0f / PI);  // Convert to degrees
    
    B[0] = 0.5f * torqueGain * dt * dt;  // Effect on angle
    B[1] = torqueGain * dt;               // Effect on rate
    
    // State cost matrix
    // Prioritize angle stabilization
    Q[0] = 100.0f;   // Angle error cost (high priority)
    Q[1] = 0.0f;     // No cross-coupling
    Q[2] = 0.0f;     // No cross-coupling
    Q[3] = 10.0f;    // Rate error cost
    
    // Control cost - prevent aggressive motor commands
    R[0] = 1.0f;
}

void initIMU() {
    // Wake up MPU6050
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0x00);  // Wake up
    Wire.endTransmission(true);
    
    // Configure gyro range (250 deg/s)
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x1B);  // GYRO_CONFIG
    Wire.write(0x00);  // FS_SEL = 0 (250 deg/s)
    Wire.endTransmission(true);
    
    // Configure accelerometer range (2g)
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x1C);  // ACCEL_CONFIG
    Wire.write(0x00);  // AFS_SEL = 0 (2g)
    Wire.endTransmission(true);
    
    // Configure low-pass filter
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x1A);  // CONFIG
    Wire.write(0x03);  // DLPF_CFG = 3 (44Hz bandwidth)
    Wire.endTransmission(true);
    
    delay(100);
    Serial.println("IMU initialized");
}

void calibrateGyro() {
    Serial.println("Calibrating gyro - keep vehicle still...");
    
    float sum = 0;
    int samples = 500;
    
    for (int i = 0; i < samples; i++) {
        Wire.beginTransmission(IMU_ADDR);
        Wire.write(0x43);  // GYRO_XOUT_H (use X for roll)
        Wire.endTransmission(false);
        Wire.requestFrom(IMU_ADDR, 2, true);
        
        int16_t raw = (Wire.read() << 8) | Wire.read();
        sum += raw / 131.0f;  // Convert to deg/s (250 deg/s range)
        
        delay(2);
    }
    
    gyroOffset = sum / samples;
    imuCalibrated = true;
    
    Serial.print("Gyro offset: ");
    Serial.print(gyroOffset, 3);
    Serial.println(" deg/s");
    Serial.println("Calibration complete");
}

void readIMU(float dt) {
    // Read gyroscope data
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x43);  // GYRO_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_ADDR, 2, true);
    
    int16_t gyroRaw = (Wire.read() << 8) | Wire.read();
    float gyroRate = (gyroRaw / 131.0f) - gyroOffset;  // deg/s
    
    // Read accelerometer data for complementary filter
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(0x3B);  // ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_ADDR, 6, true);
    
    int16_t accelX = (Wire.read() << 8) | Wire.read();
    int16_t accelY = (Wire.read() << 8) | Wire.read();
    int16_t accelZ = (Wire.read() << 8) | Wire.read();
    
    // Calculate angle from accelerometer
    accelAngle = atan2(accelY, accelZ) * (180.0f / PI);
    
    // Complementary filter for angle estimation
    currentAngle = filterAlpha * (currentAngle + gyroRate * dt) + (1.0f - filterAlpha) * accelAngle;
    currentRate = gyroRate;
}

void calculateControl() {
    // Calculate state error
    float stateError[STATE_SIZE];
    stateError[0] = currentAngle - targetAngle;  // Angle error
    stateError[1] = currentRate - 0.0f;          // Rate error (target rate = 0)
    
    // Update controller state
    controller.updateState(stateError);
    
    // Calculate control output
    float control[CONTROL_SIZE];
    controller.calculateControl(control);
    
    controlOutput = control[0];
    
    // Limit control output
    controlOutput = constrain(controlOutput, -200.0f, 200.0f);
}

void applyControl() {
    // Apply differential thrust
    // Positive control -> increase left motor, decrease right motor
    leftMotorPWM = (int)(baseThrottle - controlOutput);
    rightMotorPWM = (int)(baseThrottle + controlOutput);
    
    // Clamp motor values
    leftMotorPWM = constrain(leftMotorPWM, PWM_MIN, PWM_MAX);
    rightMotorPWM = constrain(rightMotorPWM, PWM_MIN, PWM_MAX);
    
    // Write to motors
    setMotorPWM(MOTOR_LEFT_PIN, leftMotorPWM);
    setMotorPWM(MOTOR_RIGHT_PIN, rightMotorPWM);
}

void setMotorPWM(int pin, int pulseWidth) {
    // Use analogWrite with mapping for standard PWM
    // For actual ESC control, use Servo library or hardware PWM
    int pwmValue = map(pulseWidth, PWM_MIN, PWM_MAX, 0, 255);
    analogWrite(pin, pwmValue);
}

void arm() {
    if (!imuCalibrated) {
        Serial.println("Error: IMU not calibrated");
        return;
    }
    
    Serial.println("Arming motors...");
    
    // Send arming signal
    setMotorPWM(MOTOR_LEFT_PIN, PWM_ARM);
    setMotorPWM(MOTOR_RIGHT_PIN, PWM_ARM);
    delay(1000);
    
    armed = true;
    baseThrottle = THROTTLE_IDLE;
    digitalWrite(LED_PIN, HIGH);
    
    Serial.println("Armed");
}

void disarm() {
    armed = false;
    
    setMotorPWM(MOTOR_LEFT_PIN, PWM_MIN);
    setMotorPWM(MOTOR_RIGHT_PIN, PWM_MIN);
    
    digitalWrite(LED_PIN, LOW);
    Serial.println("Disarmed");
}

void printStatus() {
    Serial.print("Angle=");
    Serial.print(currentAngle, 1);
    Serial.print(" deg, Rate=");
    Serial.print(currentRate, 1);
    Serial.print(" deg/s, Target=");
    Serial.print(targetAngle, 1);
    
    if (armed) {
        Serial.print(", L=");
        Serial.print(leftMotorPWM);
        Serial.print(", R=");
        Serial.print(rightMotorPWM);
        Serial.print(", Ctrl=");
        Serial.print(controlOutput, 1);
        
        // Print cost occasionally
        if (cycleCount % 250 == 0) {
            float cost = controller.calculateExpectedCost();
            if (cost >= 0) {
                Serial.print(", Cost=");
                Serial.print(cost, 2);
            }
        }
    }
    
    Serial.print(armed ? " [ARMED]" : " [DISARMED]");
    Serial.println();
}

void processSerialInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toUpperCase();
        
        if (input == "A") {
            arm();
        }
        else if (input == "D") {
            disarm();
        }
        else if (input.startsWith("T")) {
            int throttle = input.substring(1).toInt();
            if (throttle >= THROTTLE_IDLE && throttle <= 1800) {
                baseThrottle = throttle;
                Serial.print("Throttle set to ");
                Serial.println(throttle);
            } else {
                Serial.println("Error: Throttle must be 1200-1800");
            }
        }
        else if (input.startsWith("R")) {
            float angle = input.substring(1).toFloat();
            if (abs(angle) <= 30.0f) {
                targetAngle = angle;
                Serial.print("Target angle set to ");
                Serial.print(targetAngle, 1);
                Serial.println(" deg");
            } else {
                Serial.println("Error: Angle must be -30 to 30 degrees");
            }
        }
        else if (input == "C") {
            disarm();
            calibrateGyro();
        }
        else if (input == "S") {
            Serial.println("\n--- System Status ---");
            Serial.print("Armed: ");
            Serial.println(armed ? "Yes" : "No");
            Serial.print("Current Angle: ");
            Serial.print(currentAngle, 2);
            Serial.println(" deg");
            Serial.print("Angular Rate: ");
            Serial.print(currentRate, 2);
            Serial.println(" deg/s");
            Serial.print("Target Angle: ");
            Serial.print(targetAngle, 2);
            Serial.println(" deg");
            Serial.print("Base Throttle: ");
            Serial.println(baseThrottle);
            Serial.print("Gyro Offset: ");
            Serial.print(gyroOffset, 3);
            Serial.println(" deg/s");
            Serial.print("Control Cycles: ");
            Serial.println(cycleCount);
            Serial.print("Loop Frequency: ");
            Serial.print(CONTROL_FREQ);
            Serial.println(" Hz");
            
            // Export gains
            float K[CONTROL_SIZE * STATE_SIZE];
            controller.exportGains(K);
            Serial.print("LQR Gains: K = [");
            Serial.print(K[0], 4);
            Serial.print(", ");
            Serial.print(K[1], 4);
            Serial.println("]");
            
            Serial.println("--------------------\n");
        }
    }
}
