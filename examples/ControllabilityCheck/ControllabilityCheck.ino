/**
 * Controllability Check Example for AutoLQR
 * 
 * This example demonstrates the full controllability matrix computation
 * and rank checking to verify system controllability before computing gains.
 * 
 * The controllability matrix is defined as:
 *   C = [B, AB, A²B, ..., A^(n-1)B]
 * 
 * A system is controllable if and only if the controllability matrix
 * has full row rank (rank = n, where n is the number of states).
 * 
 * This example shows:
 *   - Computing controllability rank
 *   - Exporting the full controllability matrix
 *   - Detecting uncontrollable systems
 *   - Compile-time memory size (static version)
 */

#include <AutoLQR.h>
#include <AutoLQRStatic.h>

// Example 1: Controllable 2-state system
void testControllableSystem() {
    Serial.println("=== Test 1: Controllable 2-State System ===");
    
    AutoLQR controller(2, 1);
    
    // Position-velocity system (controllable)
    float A[] = {
        1.0f, 0.1f,
        0.0f, 0.95f
    };
    
    float B[] = {
        0.005f,
        0.1f
    };
    
    float Q[] = {10.0f, 0.0f, 0.0f, 1.0f};
    float R[] = {0.1f};
    
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);
    
    // Check controllability
    Serial.print("State size: ");
    Serial.println(controller.getStateSize());
    Serial.print("Control size: ");
    Serial.println(controller.getControlSize());
    
    int rank = controller.getControllabilityRank();
    Serial.print("Controllability rank: ");
    Serial.print(rank);
    Serial.print(" / ");
    Serial.println(controller.getStateSize());
    
    if (controller.isSystemControllable()) {
        Serial.println("System is CONTROLLABLE");
        
        // Export and print controllability matrix
        float C[2 * 2];  // 2 x (2*1) = 2 x 2
        controller.getControllabilityMatrix(C);
        
        Serial.println("Controllability matrix C = [B, AB]:");
        Serial.print("  [");
        Serial.print(C[0], 4);
        Serial.print(", ");
        Serial.print(C[1], 4);
        Serial.println("]");
        Serial.print("  [");
        Serial.print(C[2], 4);
        Serial.print(", ");
        Serial.print(C[3], 4);
        Serial.println("]");
        
        // Compute gains
        if (controller.computeGains()) {
            float K[2];
            controller.exportGains(K);
            Serial.print("LQR gains: K = [");
            Serial.print(K[0], 4);
            Serial.print(", ");
            Serial.print(K[1], 4);
            Serial.println("]");
        }
    } else {
        Serial.println("System is NOT controllable");
    }
    
    Serial.println();
}

// Example 2: Uncontrollable system
void testUncontrollableSystem() {
    Serial.println("=== Test 2: Uncontrollable System ===");
    
    AutoLQR controller(2, 1);
    
    // System where B is in null space of A - makes it uncontrollable
    // A has eigenvalues but B doesn't excite all modes
    float A[] = {
        1.0f, 0.0f,
        0.0f, 1.0f  // Identity - each state evolves independently
    };
    
    float B[] = {
        1.0f,
        0.0f   // Only affects first state - second state uncontrollable
    };
    
    float Q[] = {1.0f, 0.0f, 0.0f, 1.0f};
    float R[] = {0.1f};
    
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);
    
    int rank = controller.getControllabilityRank();
    Serial.print("Controllability rank: ");
    Serial.print(rank);
    Serial.print(" / ");
    Serial.println(controller.getStateSize());
    
    if (controller.isSystemControllable()) {
        Serial.println("System is CONTROLLABLE");
    } else {
        Serial.println("System is NOT controllable");
        Serial.println("Cannot compute optimal gains for uncontrollable system");
        
        // Show why it's uncontrollable
        float C[2 * 2];
        controller.getControllabilityMatrix(C);
        
        Serial.println("Controllability matrix C = [B, AB]:");
        Serial.print("  [");
        Serial.print(C[0], 4);
        Serial.print(", ");
        Serial.print(C[1], 4);
        Serial.println("]");
        Serial.print("  [");
        Serial.print(C[2], 4);
        Serial.print(", ");
        Serial.print(C[3], 4);
        Serial.println("]");
        Serial.println("Note: Second row is all zeros - rank deficient");
    }
    
    Serial.println();
}

// Example 3: Static version with compile-time memory calculation
void testStaticVersion() {
    Serial.println("=== Test 3: Static Version with Memory Calculation ===");
    
    // Create static controller
    AutoLQRStatic<3, 1> controller;
    
    // Print compile-time constants
    Serial.print("State size (compile-time): ");
    Serial.println(AutoLQRStatic<3, 1>::StateSize);
    Serial.print("Control size (compile-time): ");
    Serial.println(AutoLQRStatic<3, 1>::ControlSize);
    Serial.print("Memory usage (compile-time): ");
    Serial.print(AutoLQRStatic<3, 1>::MemorySize);
    Serial.println(" bytes");
    
    // 3-state system (position, velocity, acceleration)
    float A[] = {
        1.0f, 0.1f, 0.005f,
        0.0f, 1.0f, 0.1f,
        0.0f, 0.0f, 0.95f
    };
    
    float B[] = {
        0.0001f,
        0.005f,
        0.1f
    };
    
    float Q[] = {
        100.0f, 0.0f, 0.0f,
        0.0f, 10.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };
    float R[] = {0.1f};
    
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);
    
    int rank = controller.getControllabilityRank();
    Serial.print("Controllability rank: ");
    Serial.print(rank);
    Serial.print(" / ");
    Serial.println(controller.getStateSize());
    
    if (controller.isSystemControllable()) {
        Serial.println("System is CONTROLLABLE");
        
        // Export controllability matrix (3 x 3 for single input)
        float C[3 * 3];
        controller.getControllabilityMatrix(C);
        
        Serial.println("Controllability matrix C = [B, AB, A²B]:");
        for (int i = 0; i < 3; i++) {
            Serial.print("  [");
            for (int j = 0; j < 3; j++) {
                Serial.print(C[i * 3 + j], 5);
                if (j < 2) Serial.print(", ");
            }
            Serial.println("]");
        }
        
        if (controller.computeGains()) {
            float K[3];
            controller.exportGains(K);
            Serial.print("LQR gains: K = [");
            Serial.print(K[0], 4);
            Serial.print(", ");
            Serial.print(K[1], 4);
            Serial.print(", ");
            Serial.print(K[2], 4);
            Serial.println("]");
        }
    }
    
    Serial.println();
}

// Example 4: Multi-input system
void testMultiInputSystem() {
    Serial.println("=== Test 4: Multi-Input System (2 states, 2 controls) ===");
    
    AutoLQRStatic<2, 2> controller;
    
    Serial.print("Memory usage: ");
    Serial.print(controller.getMemorySize());
    Serial.println(" bytes");
    
    // 2-state, 2-input system
    float A[] = {
        0.9f, 0.1f,
        0.0f, 0.9f
    };
    
    float B[] = {
        1.0f, 0.0f,  // Control 1 affects state 1
        0.0f, 1.0f   // Control 2 affects state 2
    };
    
    float Q[] = {10.0f, 0.0f, 0.0f, 10.0f};
    float R[] = {0.1f, 0.0f, 0.0f, 0.1f};
    
    controller.setStateMatrix(A);
    controller.setInputMatrix(B);
    controller.setCostMatrices(Q, R);
    
    int rank = controller.getControllabilityRank();
    Serial.print("Controllability rank: ");
    Serial.print(rank);
    Serial.print(" / ");
    Serial.println(controller.getStateSize());
    
    // Controllability matrix is 2 x 4 for (2 states, 2 inputs)
    float C[2 * 4];
    controller.getControllabilityMatrix(C);
    
    Serial.println("Controllability matrix C = [B, AB]:");
    for (int i = 0; i < 2; i++) {
        Serial.print("  [");
        for (int j = 0; j < 4; j++) {
            Serial.print(C[i * 4 + j], 3);
            if (j < 3) Serial.print(", ");
        }
        Serial.println("]");
    }
    
    if (controller.isSystemControllable()) {
        Serial.println("System is CONTROLLABLE");
        
        if (controller.computeGains()) {
            float K[2 * 2];  // 2 controls x 2 states
            controller.exportGains(K);
            Serial.println("LQR gain matrix K:");
            Serial.print("  [");
            Serial.print(K[0], 4);
            Serial.print(", ");
            Serial.print(K[1], 4);
            Serial.println("]");
            Serial.print("  [");
            Serial.print(K[2], 4);
            Serial.print(", ");
            Serial.print(K[3], 4);
            Serial.println("]");
        }
    }
    
    Serial.println();
}

// Example 5: Memory comparison between configurations
void testMemoryComparison() {
    Serial.println("=== Test 5: Memory Size Comparison ===");
    Serial.println();
    Serial.println("Configuration       | Memory (bytes)");
    Serial.println("--------------------|---------------");
    
    Serial.print("2 states, 1 control | ");
    Serial.println(AutoLQRStatic<2, 1>::MemorySize);
    
    Serial.print("3 states, 1 control | ");
    Serial.println(AutoLQRStatic<3, 1>::MemorySize);
    
    Serial.print("4 states, 1 control | ");
    Serial.println(AutoLQRStatic<4, 1>::MemorySize);
    
    Serial.print("2 states, 2 control | ");
    Serial.println(AutoLQRStatic<2, 2>::MemorySize);
    
    Serial.print("3 states, 2 control | ");
    Serial.println(AutoLQRStatic<3, 2>::MemorySize);
    
    Serial.print("4 states, 2 control | ");
    Serial.println(AutoLQRStatic<4, 2>::MemorySize);
    
    Serial.print("4 states, 3 control | ");
    Serial.println(AutoLQRStatic<4, 3>::MemorySize);
    
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);
    
    Serial.println("AutoLQR Controllability Check Example");
    Serial.println("=====================================");
    Serial.println();
    
    testControllableSystem();
    testUncontrollableSystem();
    testStaticVersion();
    testMultiInputSystem();
    testMemoryComparison();
    
    Serial.println("All tests complete.");
}

void loop() {
    // Nothing to do in loop
}
