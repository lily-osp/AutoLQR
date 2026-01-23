/**
 * Static Allocation Example for AutoLQR
 * 
 * This example demonstrates the use of AutoLQRStatic<N,M> which uses
 * compile-time fixed arrays instead of dynamic memory allocation.
 * 
 * Benefits:
 *   - No heap fragmentation
 *   - Predictable memory usage
 *   - Configurable solver parameters
 *   - Suitable for long-running embedded applications
 * 
 * Target: Arduino Mega or equivalent (2x Uno memory/processing)
 */

#include <AutoLQRStatic.h>

// System dimensions as compile-time constants
constexpr int STATE_SIZE = 2;
constexpr int CONTROL_SIZE = 1;

// Create static controller - no heap allocation
// Memory is allocated at compile time in .bss or .data section
AutoLQRStatic<STATE_SIZE, CONTROL_SIZE> controller;

// System matrices (position-velocity system)
const float A[STATE_SIZE * STATE_SIZE] = {
    1.0f, 0.1f,   // Position += velocity * dt
    0.0f, 0.95f   // Velocity with damping
};

const float B[STATE_SIZE * CONTROL_SIZE] = {
    0.005f,  // Control effect on position
    0.1f     // Control effect on velocity
};

// Cost matrices
const float Q[STATE_SIZE * STATE_SIZE] = {
    10.0f, 0.0f,  // Position error cost
    0.0f,  1.0f   // Velocity error cost
};

const float R[CONTROL_SIZE * CONTROL_SIZE] = {
    0.1f  // Control effort cost
};

// State variables
float currentPosition = 0.0f;
float currentVelocity = 0.0f;
float targetPosition = 5.0f;

// Timing
unsigned long lastControlTime = 0;
const unsigned long controlPeriod = 100;  // 10 Hz
int cycleCount = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // Wait for serial (with timeout)
    
    Serial.println("AutoLQRStatic Example");
    Serial.println("=====================");
    
    // Print compile-time sizes
    Serial.print("State size: ");
    Serial.println(controller.getStateSize());
    Serial.print("Control size: ");
    Serial.println(controller.getControlSize());
    
    // Calculate approximate memory usage
    // Arrays: A(N*N) + B(N*M) + Q(N*N) + R(M*M) + K(M*N) + state(N) + P(N*N)
    size_t memUsage = (2 * STATE_SIZE * STATE_SIZE + 
                       2 * STATE_SIZE * CONTROL_SIZE + 
                       CONTROL_SIZE * CONTROL_SIZE + 
                       STATE_SIZE + 
                       CONTROL_SIZE * STATE_SIZE) * sizeof(float);
    Serial.print("Estimated controller memory: ");
    Serial.print(memUsage);
    Serial.println(" bytes (static allocation)");
    
    // Configure controller
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);
    
    // Check controllability
    if (controller.isSystemControllable()) {
        Serial.println("System is controllable");
    } else {
        Serial.println("Warning: System may not be controllable");
    }
    
    // Compute gains with custom solver parameters
    // More iterations and tighter tolerance for better accuracy
    Serial.println("Computing optimal gains...");
    unsigned long startTime = micros();
    
    bool success = controller.computeGains(150, 1e-7f);  // 150 iterations, 1e-7 tolerance
    
    unsigned long computeTime = micros() - startTime;
    
    if (success) {
        Serial.println("Gain computation successful");
        Serial.print("Computation time: ");
        Serial.print(computeTime);
        Serial.println(" us");
        
        // Export and print gains
        float K[CONTROL_SIZE * STATE_SIZE];
        controller.exportGains(K);
        Serial.print("K = [");
        for (int i = 0; i < CONTROL_SIZE * STATE_SIZE; i++) {
            Serial.print(K[i], 6);
            if (i < CONTROL_SIZE * STATE_SIZE - 1) Serial.print(", ");
        }
        Serial.println("]");
        
        // Estimate convergence time
        float convTime = controller.estimateConvergenceTime(0.05f);
        if (convTime > 0) {
            Serial.print("Estimated convergence: ");
            Serial.print(convTime * controlPeriod / 1000.0f, 2);
            Serial.println(" seconds");
        }
    } else {
        Serial.println("Error: Gain computation failed");
    }
    
    Serial.println("\nCommands: 'T[pos]' set target, 'R' reset, 'S' status");
    Serial.println("Starting control loop...\n");
    
    lastControlTime = millis();
}

void loop() {
    unsigned long now = millis();
    
    // Control loop at fixed rate
    if (now - lastControlTime >= controlPeriod) {
        float dt = (now - lastControlTime) / 1000.0f;
        lastControlTime = now;
        
        // Skip if gains not computed
        if (!controller.hasValidGains()) {
            return;
        }
        
        // Calculate state error
        float stateError[STATE_SIZE] = {
            currentPosition - targetPosition,
            currentVelocity - 0.0f  // Target velocity = 0
        };
        
        // Update controller and calculate control
        controller.updateState(stateError);
        
        float control[CONTROL_SIZE];
        controller.calculateControl(control);
        
        // Simulate system response
        currentVelocity = A[2] * currentPosition + A[3] * currentVelocity + B[1] * control[0];
        currentPosition = A[0] * currentPosition + A[1] * currentVelocity + B[0] * control[0];
        
        cycleCount++;
        
        // Print status every 500ms
        if (cycleCount % 5 == 0) {
            Serial.print("Pos=");
            Serial.print(currentPosition, 3);
            Serial.print(", Vel=");
            Serial.print(currentVelocity, 3);
            Serial.print(", Target=");
            Serial.print(targetPosition, 1);
            Serial.print(", Ctrl=");
            Serial.print(control[0], 4);
            
            // Print cost every 2 seconds
            if (cycleCount % 20 == 0) {
                float cost = controller.calculateExpectedCost();
                if (cost >= 0) {
                    Serial.print(", Cost=");
                    Serial.print(cost, 3);
                }
            }
            
            Serial.println();
        }
    }
    
    // Process serial input
    processSerial();
}

void processSerial() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toUpperCase();
        
        if (input.startsWith("T")) {
            float newTarget = input.substring(1).toFloat();
            targetPosition = newTarget;
            Serial.print("Target set to ");
            Serial.println(targetPosition);
        }
        else if (input == "R") {
            currentPosition = 0.0f;
            currentVelocity = 0.0f;
            cycleCount = 0;
            Serial.println("System reset");
        }
        else if (input == "S") {
            Serial.println("\n--- Status ---");
            Serial.print("Position: ");
            Serial.println(currentPosition, 4);
            Serial.print("Velocity: ");
            Serial.println(currentVelocity, 4);
            Serial.print("Target: ");
            Serial.println(targetPosition, 4);
            Serial.print("Gains valid: ");
            Serial.println(controller.hasValidGains() ? "Yes" : "No");
            Serial.print("Cycles: ");
            Serial.println(cycleCount);
            
            // Print Riccati solution
            const float* P = controller.getRiccatiSolution();
            Serial.print("P matrix trace: ");
            float trace = 0;
            for (int i = 0; i < STATE_SIZE; i++) {
                trace += P[i * STATE_SIZE + i];
            }
            Serial.println(trace, 4);
            Serial.println("--------------\n");
        }
    }
}
