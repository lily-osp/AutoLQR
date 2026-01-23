/**
 * Temperature Control System Example using AutoLQR
 * 
 * This example demonstrates LQR-based temperature control for thermal systems
 * such as 3D printer hotends, soldering stations, or industrial heaters.
 * 
 * System Model:
 *   State: [temperature, temperature_rate]
 *   Control: heater PWM duty cycle
 * 
 * Hardware Requirements:
 *   - Thermistor or thermocouple for temperature sensing
 *   - MOSFET or SSR for heater control
 *   - Heating element (cartridge heater, heating coil, etc.)
 */

#include <AutoLQR.h>

// System configuration
#define STATE_SIZE 2       // Temperature and rate of change
#define CONTROL_SIZE 1     // Single heater PWM output
#define CONTROL_FREQ 10    // Control frequency (Hz)
#define MAX_PWM 255        // Maximum PWM value

// Pin definitions
#define HEATER_PIN 9           // PWM output to MOSFET/SSR
#define THERMISTOR_PIN A0      // Thermistor analog input
#define STATUS_LED_PIN 13      // Status indicator

// Thermistor parameters (NTC 100K, beta=3950)
#define THERMISTOR_NOMINAL 100000  // Resistance at 25C
#define TEMPERATURE_NOMINAL 25     // Temperature for nominal resistance
#define B_COEFFICIENT 3950         // Beta coefficient
#define SERIES_RESISTOR 10000      // Series resistor value

// Temperature limits
#define MIN_TEMP 20.0f        // Minimum operating temperature (C)
#define MAX_TEMP 280.0f       // Maximum safe temperature (C)
#define AMBIENT_TEMP 25.0f    // Assumed ambient temperature (C)

// Thermal system parameters (experimentally determined)
const float thermalTimeConstant = 30.0f;  // Seconds to reach 63% of target
const float heaterGain = 0.5f;            // Degrees per second at full power
const float coolingRate = 0.02f;          // Natural cooling coefficient

// LQR controller
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);

// System matrices
float A[STATE_SIZE * STATE_SIZE];
float B[STATE_SIZE * CONTROL_SIZE];
float Q[STATE_SIZE * STATE_SIZE];
float R[CONTROL_SIZE * CONTROL_SIZE];

// State variables
float currentTemp = AMBIENT_TEMP;
float lastTemp = AMBIENT_TEMP;
float tempRate = 0.0f;
float targetTemp = 60.0f;  // Default target temperature

// Control variables
float controlOutput = 0.0f;
int pwmOutput = 0;

// Performance tracking
float integralError = 0.0f;
float maxOvershoot = 0.0f;
unsigned long settlingStartTime = 0;
bool settled = false;
int cycleCount = 0;

// Temperature profiles
enum TempProfile { CONSTANT, RAMP, STEP };
TempProfile currentProfile = CONSTANT;
float rampRate = 1.0f;  // Degrees per second for ramp profile

// Timing
unsigned long lastControlTime = 0;
unsigned long lastPrintTime = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Temperature Control System - AutoLQR");
    Serial.println("====================================");
    
    // Initialize pins
    pinMode(HEATER_PIN, OUTPUT);
    pinMode(STATUS_LED_PIN, OUTPUT);
    analogWrite(HEATER_PIN, 0);  // Start with heater off
    
    // Read initial temperature
    currentTemp = readTemperature();
    lastTemp = currentTemp;
    
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
        Serial.print(K[0], 6);
        Serial.print(", ");
        Serial.print(K[1], 6);
        Serial.println("]");
        
        float convTime = controller.estimateConvergenceTime(0.05);
        if (convTime > 0) {
            Serial.print("Estimated settling time: ");
            Serial.print(convTime / CONTROL_FREQ);
            Serial.println(" seconds");
        }
    } else {
        Serial.println("Error: Failed to compute LQR gains");
    }
    
    Serial.println("\nCommands:");
    Serial.println("  Txx  - Set target temperature (e.g., T200 for 200C)");
    Serial.println("  P    - Toggle temperature profile (constant/ramp/step)");
    Serial.println("  S    - Print system status");
    Serial.println("  R    - Reset controller");
    
    lastControlTime = millis();
    lastPrintTime = millis();
    settlingStartTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    
    // Control loop
    if (currentTime - lastControlTime >= (1000 / CONTROL_FREQ)) {
        float dt = (currentTime - lastControlTime) / 1000.0f;
        
        // Read current temperature
        currentTemp = readTemperature();
        
        // Calculate temperature rate of change
        tempRate = (currentTemp - lastTemp) / dt;
        lastTemp = currentTemp;
        
        // Update target based on profile
        updateTargetProfile(dt);
        
        // Safety check
        if (currentTemp > MAX_TEMP) {
            analogWrite(HEATER_PIN, 0);
            Serial.println("SAFETY: Max temperature exceeded - heater disabled");
            digitalWrite(STATUS_LED_PIN, HIGH);
            return;
        }
        
        // Calculate control output
        calculateControl();
        
        // Apply control
        applyControl();
        
        // Track performance
        trackPerformance();
        
        cycleCount++;
        lastControlTime = currentTime;
    }
    
    // Print status periodically
    if (currentTime - lastPrintTime >= 500) {
        printStatus();
        lastPrintTime = currentTime;
    }
    
    // Process serial commands
    processSerialInput();
}

void setupSystemMatrices() {
    float dt = 1.0f / CONTROL_FREQ;
    
    // Discrete-time thermal system model
    // State: [temperature, temperature_rate]
    // 
    // Temperature dynamics: T(k+1) = T(k) + T_rate(k) * dt
    // Rate dynamics: T_rate(k+1) = T_rate(k) * decay + heater_effect * u(k)
    
    float decayFactor = exp(-dt / thermalTimeConstant);
    
    A[0] = 1.0f;           // Temperature persists
    A[1] = dt;             // Rate affects temperature
    A[2] = 0.0f;           // Temperature doesn't directly affect rate
    A[3] = decayFactor;    // Rate decays due to thermal inertia
    
    // Input effect on state
    B[0] = 0.5f * heaterGain * dt * dt;  // Heater effect on temperature
    B[1] = heaterGain * dt;               // Heater effect on rate
    
    // State cost matrix
    // Higher weight on temperature error for tight control
    Q[0] = 10.0f;    // Temperature error cost
    Q[1] = 0.0f;     // No cross-coupling
    Q[2] = 0.0f;     // No cross-coupling
    Q[3] = 1.0f;     // Rate error cost (penalize rapid changes)
    
    // Control cost - balance between response speed and energy
    R[0] = 0.1f;
}

float readTemperature() {
    // Read thermistor and convert to temperature
    int adcValue = analogRead(THERMISTOR_PIN);
    
    // Prevent division by zero
    if (adcValue <= 0) adcValue = 1;
    if (adcValue >= 1023) adcValue = 1022;
    
    // Calculate thermistor resistance
    float resistance = SERIES_RESISTOR / (1023.0f / adcValue - 1.0f);
    
    // Steinhart-Hart equation (simplified B-parameter)
    float steinhart;
    steinhart = resistance / THERMISTOR_NOMINAL;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0f / (TEMPERATURE_NOMINAL + 273.15f);
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f;
    
    // Apply simple low-pass filter
    static float filteredTemp = AMBIENT_TEMP;
    filteredTemp = 0.8f * filteredTemp + 0.2f * steinhart;
    
    return filteredTemp;
}

void calculateControl() {
    // Calculate state error
    float stateError[STATE_SIZE];
    stateError[0] = currentTemp - targetTemp;   // Temperature error
    stateError[1] = tempRate - 0.0f;            // Rate error (target rate = 0 at setpoint)
    
    // For ramp profile, adjust target rate
    if (currentProfile == RAMP && currentTemp < targetTemp) {
        stateError[1] = tempRate - rampRate;
    }
    
    // Update controller state
    controller.updateState(stateError);
    
    // Calculate control output
    float control[CONTROL_SIZE];
    controller.calculateControl(control);
    
    controlOutput = control[0];
    
    // Add feedforward for heating (compensate for natural cooling)
    if (targetTemp > currentTemp) {
        float coolingCompensation = coolingRate * (currentTemp - AMBIENT_TEMP);
        controlOutput += coolingCompensation * 0.1f;
    }
}

void applyControl() {
    // Convert control signal to PWM (0-255)
    // Control output is typically negative (error reduction)
    float normalizedControl = -controlOutput;
    
    // Clamp and scale
    normalizedControl = constrain(normalizedControl, 0.0f, 1.0f);
    pwmOutput = (int)(normalizedControl * MAX_PWM);
    
    // Apply anti-windup: reduce output near target
    float tempError = targetTemp - currentTemp;
    if (abs(tempError) < 2.0f) {
        // Fine control near setpoint
        pwmOutput = constrain(pwmOutput, 0, MAX_PWM / 4);
    }
    
    // Safety: no heating if above target
    if (currentTemp > targetTemp + 1.0f) {
        pwmOutput = 0;
    }
    
    analogWrite(HEATER_PIN, pwmOutput);
    
    // Status LED indicates heating
    digitalWrite(STATUS_LED_PIN, pwmOutput > 0 ? HIGH : LOW);
}

void updateTargetProfile(float dt) {
    static float profileTarget = targetTemp;
    
    switch (currentProfile) {
        case CONSTANT:
            // Target stays constant
            break;
            
        case RAMP:
            // Gradually approach target
            if (profileTarget < targetTemp) {
                profileTarget += rampRate * dt;
                if (profileTarget > targetTemp) {
                    profileTarget = targetTemp;
                }
            }
            break;
            
        case STEP:
            // Immediate step (already set)
            profileTarget = targetTemp;
            break;
    }
}

void trackPerformance() {
    float error = currentTemp - targetTemp;
    
    // Track overshoot
    if (error > maxOvershoot) {
        maxOvershoot = error;
    }
    
    // Track settling
    if (!settled && abs(error) < 2.0f && abs(tempRate) < 0.5f) {
        settled = true;
        unsigned long settleTime = millis() - settlingStartTime;
        Serial.print("Settled in ");
        Serial.print(settleTime / 1000.0f);
        Serial.println(" seconds");
    }
    
    // Accumulate integral error for analysis
    integralError += abs(error) / CONTROL_FREQ;
}

void printStatus() {
    Serial.print("T=");
    Serial.print(currentTemp, 1);
    Serial.print("C, Target=");
    Serial.print(targetTemp, 1);
    Serial.print("C, Rate=");
    Serial.print(tempRate, 2);
    Serial.print("C/s, PWM=");
    Serial.print(pwmOutput);
    Serial.print(" (");
    Serial.print((pwmOutput * 100) / MAX_PWM);
    Serial.print("%)");
    
    // Print cost every 10th update
    if (cycleCount % 10 == 0) {
        float cost = controller.calculateExpectedCost();
        if (cost >= 0) {
            Serial.print(", Cost=");
            Serial.print(cost, 2);
        }
    }
    
    Serial.println();
}

void processSerialInput() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toUpperCase();
        
        if (input.startsWith("T")) {
            // Set target temperature
            float newTarget = input.substring(1).toFloat();
            if (newTarget >= MIN_TEMP && newTarget <= MAX_TEMP) {
                targetTemp = newTarget;
                settled = false;
                settlingStartTime = millis();
                maxOvershoot = 0;
                integralError = 0;
                Serial.print("Target set to ");
                Serial.print(targetTemp, 1);
                Serial.println("C");
            } else {
                Serial.print("Error: Temperature must be between ");
                Serial.print(MIN_TEMP);
                Serial.print(" and ");
                Serial.println(MAX_TEMP);
            }
        }
        else if (input == "P") {
            // Toggle profile
            currentProfile = (TempProfile)((currentProfile + 1) % 3);
            Serial.print("Profile: ");
            switch (currentProfile) {
                case CONSTANT: Serial.println("Constant"); break;
                case RAMP: Serial.println("Ramp"); break;
                case STEP: Serial.println("Step"); break;
            }
        }
        else if (input == "S") {
            // Print detailed status
            Serial.println("\n--- System Status ---");
            Serial.print("Current Temperature: ");
            Serial.print(currentTemp, 2);
            Serial.println(" C");
            Serial.print("Target Temperature: ");
            Serial.print(targetTemp, 2);
            Serial.println(" C");
            Serial.print("Temperature Rate: ");
            Serial.print(tempRate, 3);
            Serial.println(" C/s");
            Serial.print("Max Overshoot: ");
            Serial.print(maxOvershoot, 2);
            Serial.println(" C");
            Serial.print("Integral Error: ");
            Serial.println(integralError, 2);
            Serial.print("Settled: ");
            Serial.println(settled ? "Yes" : "No");
            Serial.print("Control Cycles: ");
            Serial.println(cycleCount);
            Serial.println("--------------------\n");
        }
        else if (input == "R") {
            // Reset controller
            integralError = 0;
            maxOvershoot = 0;
            settled = false;
            settlingStartTime = millis();
            cycleCount = 0;
            Serial.println("Controller reset");
        }
    }
}
