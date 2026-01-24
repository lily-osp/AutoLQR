/**
 * Large Matrix Test Example - Testing Gauss-Jordan Matrix Inversion
 * 
 * This example tests the AutoLQR library with MIMO systems that have
 * more than 3 control inputs, which requires Gauss-Jordan matrix inversion.
 * 
 * The example simulates a simplified quadcopter-like system with:
 *   - 4 states: [roll, pitch, roll_rate, pitch_rate]
 *   - 4 control inputs: [motor1, motor2, motor3, motor4]
 * 
 * This configuration requires inverting a 4x4 matrix during DARE solving,
 * which uses the Gauss-Jordan algorithm instead of closed-form solutions.
 */

#include <AutoLQR.h>

// System configuration: 4 states, 4 control inputs
#define STATE_SIZE 4
#define CONTROL_SIZE 4

// Create controller - this will use Gauss-Jordan for matrix inversion
AutoLQR controller(STATE_SIZE, CONTROL_SIZE);

// System matrices
float A[STATE_SIZE * STATE_SIZE];
float B[STATE_SIZE * CONTROL_SIZE];
float Q[STATE_SIZE * STATE_SIZE];
float R[CONTROL_SIZE * CONTROL_SIZE];

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    
    Serial.println("===========================================");
    Serial.println("AutoLQR Large Matrix Test (Gauss-Jordan)");
    Serial.println("===========================================");
    Serial.println();
    
    Serial.print("State size: ");
    Serial.println(STATE_SIZE);
    Serial.print("Control size: ");
    Serial.println(CONTROL_SIZE);
    Serial.println();
    
    // Initialize and run tests
    setupSystemMatrices();
    
    Serial.println("--- Test 1: Basic MIMO System ---");
    if (runBasicTest()) {
        Serial.println("PASSED: Basic MIMO test\n");
    } else {
        Serial.println("FAILED: Basic MIMO test\n");
    }
    
    Serial.println("--- Test 2: Controllability Check ---");
    if (runControllabilityTest()) {
        Serial.println("PASSED: Controllability test\n");
    } else {
        Serial.println("FAILED: Controllability test\n");
    }
    
    Serial.println("--- Test 3: Control Calculation ---");
    runControlTest();
    
    Serial.println("--- Test 4: Different Q/R Weights ---");
    runWeightTest();
    
    Serial.println("===========================================");
    Serial.println("All tests completed");
    Serial.println("===========================================");
}

void loop() {
    // Nothing to do in loop
    delay(1000);
}

void setupSystemMatrices() {
    // Simplified quadcopter-like discrete-time model
    // State: [roll, pitch, roll_rate, pitch_rate]
    // Control: [motor1, motor2, motor3, motor4]
    
    float dt = 0.02f;  // 50 Hz control loop
    
    // State matrix A (simplified dynamics)
    // Position states integrate velocity
    // Velocity states have some damping
    memset(A, 0, sizeof(A));
    A[0*STATE_SIZE + 0] = 1.0f;      // roll
    A[0*STATE_SIZE + 2] = dt;        // roll += roll_rate * dt
    A[1*STATE_SIZE + 1] = 1.0f;      // pitch
    A[1*STATE_SIZE + 3] = dt;        // pitch += pitch_rate * dt
    A[2*STATE_SIZE + 2] = 0.95f;     // roll_rate with damping
    A[3*STATE_SIZE + 3] = 0.95f;     // pitch_rate with damping
    
    // Input matrix B
    // Each motor affects roll and pitch rates differently
    // Motor layout: 1=front-left, 2=front-right, 3=rear-left, 4=rear-right
    memset(B, 0, sizeof(B));
    float gain = 0.1f;
    
    // Roll rate affected by left-right motor difference
    B[2*CONTROL_SIZE + 0] = +gain;   // motor1 (front-left) -> +roll
    B[2*CONTROL_SIZE + 1] = -gain;   // motor2 (front-right) -> -roll
    B[2*CONTROL_SIZE + 2] = +gain;   // motor3 (rear-left) -> +roll
    B[2*CONTROL_SIZE + 3] = -gain;   // motor4 (rear-right) -> -roll
    
    // Pitch rate affected by front-rear motor difference
    B[3*CONTROL_SIZE + 0] = +gain;   // motor1 (front-left) -> +pitch
    B[3*CONTROL_SIZE + 1] = +gain;   // motor2 (front-right) -> +pitch
    B[3*CONTROL_SIZE + 2] = -gain;   // motor3 (rear-left) -> -pitch
    B[3*CONTROL_SIZE + 3] = -gain;   // motor4 (rear-right) -> -pitch
}

bool runBasicTest() {
    Serial.println("Setting up 4x4 MIMO system...");
    
    // Set system matrices
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    
    // State cost matrix Q (diagonal)
    memset(Q, 0, sizeof(Q));
    Q[0*STATE_SIZE + 0] = 100.0f;    // roll error cost
    Q[1*STATE_SIZE + 1] = 100.0f;    // pitch error cost
    Q[2*STATE_SIZE + 2] = 10.0f;     // roll_rate error cost
    Q[3*STATE_SIZE + 3] = 10.0f;     // pitch_rate error cost
    
    // Control cost matrix R (4x4 diagonal)
    // This is the matrix that requires Gauss-Jordan inversion
    memset(R, 0, sizeof(R));
    R[0*CONTROL_SIZE + 0] = 1.0f;    // motor1 effort cost
    R[1*CONTROL_SIZE + 1] = 1.0f;    // motor2 effort cost
    R[2*CONTROL_SIZE + 2] = 1.0f;    // motor3 effort cost
    R[3*CONTROL_SIZE + 3] = 1.0f;    // motor4 effort cost
    
    controller.setCostMatrices(Q, R);
    
    Serial.println("Computing LQR gains (requires 4x4 matrix inversion)...");
    
    unsigned long startTime = micros();
    bool success = controller.computeGains();
    unsigned long endTime = micros();
    
    Serial.print("Computation time: ");
    Serial.print(endTime - startTime);
    Serial.println(" microseconds");
    
    if (success) {
        Serial.println("Gain computation successful!");
        
        // Export and print gains
        float K[CONTROL_SIZE * STATE_SIZE];
        controller.exportGains(K);
        
        Serial.println("\nComputed gain matrix K:");
        printMatrix(K, CONTROL_SIZE, STATE_SIZE, "K");
        
        return true;
    } else {
        Serial.println("Gain computation failed!");
        return false;
    }
}

bool runControllabilityTest() {
    Serial.println("Checking system controllability...");
    
    int rank = controller.getControllabilityRank();
    bool controllable = controller.isSystemControllable();
    
    Serial.print("Controllability rank: ");
    Serial.print(rank);
    Serial.print(" / ");
    Serial.println(STATE_SIZE);
    
    Serial.print("System controllable: ");
    Serial.println(controllable ? "Yes" : "No");
    
    return controllable;
}

void runControlTest() {
    Serial.println("Testing control output calculation...");
    
    // Simulate a state error
    float state_error[STATE_SIZE] = {
        0.1f,   // 0.1 rad roll error
        -0.05f, // -0.05 rad pitch error
        0.0f,   // no roll rate error
        0.0f    // no pitch rate error
    };
    
    controller.updateState(state_error);
    
    float control[CONTROL_SIZE];
    controller.calculateControl(control);
    
    Serial.println("\nState error:");
    Serial.print("  Roll: "); Serial.print(state_error[0], 4); Serial.println(" rad");
    Serial.print("  Pitch: "); Serial.print(state_error[1], 4); Serial.println(" rad");
    Serial.print("  Roll rate: "); Serial.print(state_error[2], 4); Serial.println(" rad/s");
    Serial.print("  Pitch rate: "); Serial.print(state_error[3], 4); Serial.println(" rad/s");
    
    Serial.println("\nControl output:");
    Serial.print("  Motor 1: "); Serial.println(control[0], 4);
    Serial.print("  Motor 2: "); Serial.println(control[1], 4);
    Serial.print("  Motor 3: "); Serial.println(control[2], 4);
    Serial.print("  Motor 4: "); Serial.println(control[3], 4);
    
    // Verify control makes sense
    // For positive roll error, we expect motors 2,4 to increase (right side)
    // For negative pitch error, we expect motors 3,4 to increase (rear)
    float controlSum = control[0] + control[1] + control[2] + control[3];
    Serial.print("\nControl sum (should be ~0 for pure attitude): ");
    Serial.println(controlSum, 4);
    Serial.println();
}

void runWeightTest() {
    Serial.println("Testing different Q/R weight configurations...");
    
    // Test 1: Aggressive (high Q, low R)
    memset(Q, 0, sizeof(Q));
    Q[0*STATE_SIZE + 0] = 500.0f;
    Q[1*STATE_SIZE + 1] = 500.0f;
    Q[2*STATE_SIZE + 2] = 50.0f;
    Q[3*STATE_SIZE + 3] = 50.0f;
    
    memset(R, 0, sizeof(R));
    for (int i = 0; i < CONTROL_SIZE; i++) {
        R[i*CONTROL_SIZE + i] = 0.1f;
    }
    
    controller.setCostMatrices(Q, R);
    
    if (controller.computeGains()) {
        float K[CONTROL_SIZE * STATE_SIZE];
        controller.exportGains(K);
        
        Serial.println("\nAggressive tuning (high Q, low R):");
        Serial.print("  K[0][0] (roll->motor1): ");
        Serial.println(K[0], 4);
    }
    
    // Test 2: Conservative (low Q, high R)
    memset(Q, 0, sizeof(Q));
    Q[0*STATE_SIZE + 0] = 50.0f;
    Q[1*STATE_SIZE + 1] = 50.0f;
    Q[2*STATE_SIZE + 2] = 5.0f;
    Q[3*STATE_SIZE + 3] = 5.0f;
    
    memset(R, 0, sizeof(R));
    for (int i = 0; i < CONTROL_SIZE; i++) {
        R[i*CONTROL_SIZE + i] = 10.0f;
    }
    
    controller.setCostMatrices(Q, R);
    
    if (controller.computeGains()) {
        float K[CONTROL_SIZE * STATE_SIZE];
        controller.exportGains(K);
        
        Serial.println("\nConservative tuning (low Q, high R):");
        Serial.print("  K[0][0] (roll->motor1): ");
        Serial.println(K[0], 4);
    }
    
    Serial.println("\nNote: Aggressive tuning gives higher gains (faster response)");
    Serial.println("      Conservative tuning gives lower gains (smoother control)");
    Serial.println();
}

void printMatrix(float* M, int rows, int cols, const char* name) {
    Serial.print(name);
    Serial.println(" =");
    
    for (int i = 0; i < rows; i++) {
        Serial.print("  [");
        for (int j = 0; j < cols; j++) {
            if (M[i * cols + j] >= 0) Serial.print(" ");
            Serial.print(M[i * cols + j], 4);
            if (j < cols - 1) Serial.print(", ");
        }
        Serial.println("]");
    }
}
