# Large Matrix Test Example

This example tests the Gauss-Jordan matrix inversion for MIMO systems with more than 3 control inputs.

## Overview

The AutoLQR library uses different matrix inversion methods:
- **1x1, 2x2, 3x3**: Closed-form solutions (fast, no extra memory)
- **4x4 and larger**: Gauss-Jordan elimination with partial pivoting

This example verifies the Gauss-Jordan implementation works correctly with a 4-state, 4-control system.

## System Model

Simplified quadcopter-like attitude control:
- **States**: roll, pitch, roll_rate, pitch_rate (4 states)
- **Controls**: motor1, motor2, motor3, motor4 (4 inputs)

Motor layout (X configuration):
```
    1       2
     \     /
      \   /
       \ /
       / \
      /   \
     /     \
    3       4

Motor 1: front-left  (+roll, +pitch)
Motor 2: front-right (-roll, +pitch)
Motor 3: rear-left   (+roll, -pitch)
Motor 4: rear-right  (-roll, -pitch)
```

## Tests Performed

### Test 1: Basic MIMO System
- Sets up 4x4 system matrices
- Computes LQR gains (requires 4x4 matrix inversion)
- Reports computation time
- Prints gain matrix

### Test 2: Controllability Check
- Verifies the 4-state system is controllable
- Reports controllability rank

### Test 3: Control Calculation
- Applies a test state error
- Computes control outputs
- Verifies motor commands make physical sense

### Test 4: Q/R Weight Comparison
- Tests aggressive tuning (high Q, low R)
- Tests conservative tuning (low Q, high R)
- Shows effect on gain magnitude

## Why Gauss-Jordan?

The DARE solver needs to invert (R + B'PB), which is controlSize x controlSize. For this example:
- R is 4x4
- B'PB is 4x4
- Sum is 4x4

Previous library versions used closed-form inversion limited to 3x3, restricting controlSize to maximum 3. The Gauss-Jordan implementation removes this limitation.

## Expected Output

```
State size: 4
Control size: 4

--- Test 1: Basic MIMO System ---
Setting up 4x4 MIMO system...
Computing LQR gains (requires 4x4 matrix inversion)...
Computation time: XXXX microseconds
Gain computation successful!
```

## Native Test

A companion test file `test_gauss_jordan.cpp` can be compiled and run on a PC:

```bash
cd AutoLQR
g++ -std=c++11 -o test_gauss_jordan test_gauss_jordan.cpp -lm
./test_gauss_jordan
```

This verifies matrix inversion for 2x2 through 6x6 matrices.

## Usage

1. Upload to Arduino (Mega recommended for memory)
2. Open Serial Monitor at 115200 baud
3. Observe all tests run automatically
4. Verify all tests pass

## Requirements

- Arduino with sufficient RAM (Mega or equivalent)
- The dynamic AutoLQR class (not AutoLQRStatic, which is limited to M<=3)
