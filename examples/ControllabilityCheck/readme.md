# Controllability Check Example

This example demonstrates how to verify system controllability before computing LQR gains.

## Overview

A system is controllable if and only if the controllability matrix has full row rank. This example shows:
- Computing controllability rank
- Exporting the controllability matrix
- Detecting uncontrollable systems
- Memory size comparison for different configurations

## Controllability Theory

The controllability matrix is defined as:
```
C = [B, AB, A²B, ..., A^(n-1)B]
```

Where:
- n = number of states
- A = state matrix
- B = input matrix

A system with n states is controllable if rank(C) = n.

## Test Cases

### Test 1: Controllable System
Standard position-velocity system where both states can be controlled through the input.

### Test 2: Uncontrollable System
A system where the input only affects one state:
```cpp
A = [1, 0]    B = [1]
    [0, 1]        [0]  // Second state unaffected
```
Result: rank = 1, but need rank = 2 for controllability.

### Test 3: Three-State System
Position-velocity-acceleration system demonstrating the static version with compile-time memory calculation.

### Test 4: Multi-Input System
Two states with two control inputs, showing controllability matrix for MIMO systems.

### Test 5: Memory Comparison
Displays memory usage for various system configurations.

## API Usage

### Check Controllability
```cpp
if (controller.isSystemControllable()) {
    // Safe to compute gains
    controller.computeGains();
} else {
    // Cannot compute optimal gains
}
```

### Get Controllability Rank
```cpp
int rank = controller.getControllabilityRank();
int required = controller.getStateSize();

if (rank < required) {
    Serial.print("System is rank deficient: ");
    Serial.print(rank);
    Serial.print(" < ");
    Serial.println(required);
}
```

### Export Controllability Matrix
```cpp
// For 2 states, 1 input: C is 2x2
float C[STATE_SIZE * STATE_SIZE * CONTROL_SIZE];
controller.getControllabilityMatrix(C);
```

## Memory Size Table

| Configuration | Memory (bytes) |
|--------------|----------------|
| 2 states, 1 control | 73 |
| 3 states, 1 control | 117 |
| 4 states, 1 control | 169 |
| 2 states, 2 control | 89 |
| 3 states, 2 control | 141 |
| 4 states, 2 control | 201 |
| 4 states, 3 control | 217 |

## Common Controllability Issues

1. **Decoupled states**: Some states don't interact with inputs
2. **Redundant inputs**: Multiple inputs affecting states identically
3. **System at equilibrium**: Certain operating points may lose controllability

## Usage

1. Upload to Arduino
2. Open Serial Monitor at 115200 baud
3. Observe test results for each system configuration
4. Note which systems are controllable and why others fail
