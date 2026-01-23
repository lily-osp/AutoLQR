/**
 * Solver Configuration Example for AutoLQR
 * 
 * This example demonstrates how to configure the DARE solver parameters
 * for different use cases:
 *   - Fast computation with fewer iterations
 *   - High precision with tighter tolerance
 *   - Comparing solver performance
 * 
 * The DARE (Discrete Algebraic Riccati Equation) solver iteratively
 * computes the optimal gain matrix K. Adjusting parameters allows
 * trading off computation time vs accuracy.
 */

#include <AutoLQR.h>

// System configuration
#define STATE_SIZE 2
#define CONTROL_SIZE 1

// Create controller
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);

// System matrices (position-velocity with damping)
float A[STATE_SIZE * STATE_SIZE] = {
    1.0f, 0.1f,
    0.0f, 0.95f
};

float B[STATE_SIZE * CONTROL_SIZE] = {
    0.005f,
    0.1f
};

// Cost matrices
float Q[STATE_SIZE * STATE_SIZE] = {
    10.0f, 0.0f,
    0.0f, 1.0f
};

float R[CONTROL_SIZE * CONTROL_SIZE] = {
    0.1f
};

// Store gains for comparison
float K_default[CONTROL_SIZE * STATE_SIZE];
float K_fast[CONTROL_SIZE * STATE_SIZE];
float K_precise[CONTROL_SIZE * STATE_SIZE];

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println("AutoLQR Solver Configuration Example");
    Serial.println("=====================================\n");
    
    // Initialize system matrices
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);
    
    // Print default solver parameters
    Serial.println("Default solver parameters:");
    Serial.print("  Max iterations: ");
    Serial.println(controller.getMaxIterations());
    Serial.print("  Tolerance: ");
    Serial.println(controller.getTolerance(), 8);
    Serial.println();
    
    // Test 1: Default parameters
    Serial.println("--- Test 1: Default Parameters ---");
    testSolverDefault();
    
    // Test 2: Fast computation (fewer iterations)
    Serial.println("\n--- Test 2: Fast Computation ---");
    testSolverFast();
    
    // Test 3: High precision (more iterations, tighter tolerance)
    Serial.println("\n--- Test 3: High Precision ---");
    testSolverPrecise();
    
    // Test 4: Using setSolverParameters for persistent config
    Serial.println("\n--- Test 4: Persistent Configuration ---");
    testPersistentConfig();
    
    // Compare results
    Serial.println("\n--- Comparison of Results ---");
    compareResults();
    
    Serial.println("\n--- Interactive Mode ---");
    Serial.println("Commands:");
    Serial.println("  D - Compute with default params");
    Serial.println("  F - Compute with fast params (50 iter, 1e-4)");
    Serial.println("  P - Compute with precise params (200 iter, 1e-8)");
    Serial.println("  S[iter],[tol] - Set custom params (e.g., S150,0.0000001)");
    Serial.println("  I - Show current settings");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toUpperCase();
        
        if (input == "D") {
            Serial.println("\nComputing with default parameters...");
            controller.setSolverParameters(100, 1e-6f);
            runAndPrint();
        }
        else if (input == "F") {
            Serial.println("\nComputing with fast parameters...");
            unsigned long start = micros();
            controller.computeGains(50, 1e-4f);
            unsigned long elapsed = micros() - start;
            printGainsAndTime(elapsed);
        }
        else if (input == "P") {
            Serial.println("\nComputing with precise parameters...");
            unsigned long start = micros();
            controller.computeGains(200, 1e-8f);
            unsigned long elapsed = micros() - start;
            printGainsAndTime(elapsed);
        }
        else if (input.startsWith("S")) {
            // Parse custom parameters: S[iter],[tol]
            int commaIdx = input.indexOf(',');
            if (commaIdx > 1) {
                int iter = input.substring(1, commaIdx).toInt();
                float tol = input.substring(commaIdx + 1).toFloat();
                
                if (iter > 0 && tol > 0) {
                    Serial.print("\nSetting parameters: iterations=");
                    Serial.print(iter);
                    Serial.print(", tolerance=");
                    Serial.println(tol, 10);
                    
                    controller.setSolverParameters(iter, tol);
                    runAndPrint();
                } else {
                    Serial.println("Invalid parameters");
                }
            } else {
                Serial.println("Format: S[iterations],[tolerance]");
            }
        }
        else if (input == "I") {
            Serial.println("\nCurrent solver settings:");
            Serial.print("  Max iterations: ");
            Serial.println(controller.getMaxIterations());
            Serial.print("  Tolerance: ");
            Serial.println(controller.getTolerance(), 10);
        }
    }
}

void testSolverDefault() {
    unsigned long startTime = micros();
    bool success = controller.computeGains();
    unsigned long elapsed = micros() - startTime;
    
    if (success) {
        controller.exportGains(K_default);
        Serial.print("Time: ");
        Serial.print(elapsed);
        Serial.println(" us");
        Serial.print("K = [");
        Serial.print(K_default[0], 6);
        Serial.print(", ");
        Serial.print(K_default[1], 6);
        Serial.println("]");
    } else {
        Serial.println("Computation failed");
    }
}

void testSolverFast() {
    // Use one-time parameters (doesn't change persistent settings)
    unsigned long startTime = micros();
    bool success = controller.computeGains(50, 1e-4f);  // Fewer iterations, looser tolerance
    unsigned long elapsed = micros() - startTime;
    
    if (success) {
        controller.exportGains(K_fast);
        Serial.print("Parameters: 50 iterations, 1e-4 tolerance\n");
        Serial.print("Time: ");
        Serial.print(elapsed);
        Serial.println(" us");
        Serial.print("K = [");
        Serial.print(K_fast[0], 6);
        Serial.print(", ");
        Serial.print(K_fast[1], 6);
        Serial.println("]");
    } else {
        Serial.println("Computation failed");
    }
}

void testSolverPrecise() {
    // Use one-time parameters for high precision
    unsigned long startTime = micros();
    bool success = controller.computeGains(200, 1e-8f);  // More iterations, tighter tolerance
    unsigned long elapsed = micros() - startTime;
    
    if (success) {
        controller.exportGains(K_precise);
        Serial.print("Parameters: 200 iterations, 1e-8 tolerance\n");
        Serial.print("Time: ");
        Serial.print(elapsed);
        Serial.println(" us");
        Serial.print("K = [");
        Serial.print(K_precise[0], 6);
        Serial.print(", ");
        Serial.print(K_precise[1], 6);
        Serial.println("]");
    } else {
        Serial.println("Computation failed");
    }
}

void testPersistentConfig() {
    // Demonstrate setSolverParameters for persistent configuration
    Serial.println("Setting persistent parameters: 150 iter, 1e-7 tol");
    controller.setSolverParameters(150, 1e-7f);
    
    // Verify settings
    Serial.print("Stored max iterations: ");
    Serial.println(controller.getMaxIterations());
    Serial.print("Stored tolerance: ");
    Serial.println(controller.getTolerance(), 8);
    
    // Compute with stored parameters
    unsigned long startTime = micros();
    bool success = controller.computeGains();  // Uses stored parameters
    unsigned long elapsed = micros() - startTime;
    
    if (success) {
        float K[CONTROL_SIZE * STATE_SIZE];
        controller.exportGains(K);
        Serial.print("Time: ");
        Serial.print(elapsed);
        Serial.println(" us");
        Serial.print("K = [");
        Serial.print(K[0], 6);
        Serial.print(", ");
        Serial.print(K[1], 6);
        Serial.println("]");
    }
    
    // Reset to default for further tests
    controller.setSolverParameters(100, 1e-6f);
}

void compareResults() {
    Serial.println("Gain comparison (K[0], K[1]):");
    Serial.println();
    
    Serial.print("  Default (100, 1e-6):  [");
    Serial.print(K_default[0], 8);
    Serial.print(", ");
    Serial.print(K_default[1], 8);
    Serial.println("]");
    
    Serial.print("  Fast (50, 1e-4):      [");
    Serial.print(K_fast[0], 8);
    Serial.print(", ");
    Serial.print(K_fast[1], 8);
    Serial.println("]");
    
    Serial.print("  Precise (200, 1e-8):  [");
    Serial.print(K_precise[0], 8);
    Serial.print(", ");
    Serial.print(K_precise[1], 8);
    Serial.println("]");
    
    // Calculate differences
    Serial.println();
    Serial.println("Difference from precise:");
    
    float diff_default_0 = fabs(K_default[0] - K_precise[0]);
    float diff_default_1 = fabs(K_default[1] - K_precise[1]);
    Serial.print("  Default: [");
    Serial.print(diff_default_0, 10);
    Serial.print(", ");
    Serial.print(diff_default_1, 10);
    Serial.println("]");
    
    float diff_fast_0 = fabs(K_fast[0] - K_precise[0]);
    float diff_fast_1 = fabs(K_fast[1] - K_precise[1]);
    Serial.print("  Fast:    [");
    Serial.print(diff_fast_0, 10);
    Serial.print(", ");
    Serial.print(diff_fast_1, 10);
    Serial.println("]");
}

void runAndPrint() {
    unsigned long start = micros();
    bool success = controller.computeGains();
    unsigned long elapsed = micros() - start;
    
    if (success) {
        printGainsAndTime(elapsed);
    } else {
        Serial.println("Computation failed");
    }
}

void printGainsAndTime(unsigned long elapsed) {
    float K[CONTROL_SIZE * STATE_SIZE];
    controller.exportGains(K);
    
    Serial.print("Time: ");
    Serial.print(elapsed);
    Serial.println(" us");
    Serial.print("K = [");
    Serial.print(K[0], 8);
    Serial.print(", ");
    Serial.print(K[1], 8);
    Serial.println("]");
}
