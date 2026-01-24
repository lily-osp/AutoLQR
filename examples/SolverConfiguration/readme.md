# Solver Configuration Example

This example demonstrates how to configure the DARE (Discrete Algebraic Riccati Equation) solver parameters for different use cases.

## Overview

The LQR gain computation involves iteratively solving the DARE until convergence. This example shows how to:
- Adjust iteration count and tolerance
- Trade off computation time vs accuracy
- Use persistent vs one-time parameters
- Compare results across configurations

## Solver Parameters

| Parameter | Description | Default | Effect |
|-----------|-------------|---------|--------|
| maxIterations | Maximum DARE iterations | 100 | More = better convergence |
| tolerance | Convergence threshold | 1e-6 | Lower = more precise |

## Configuration Methods

### One-Time Parameters
```cpp
// Parameters apply only to this call
controller.computeGains(50, 1e-4f);  // Fast
controller.computeGains(200, 1e-8f); // Precise
```

### Persistent Parameters
```cpp
// Set parameters that persist across calls
controller.setSolverParameters(150, 1e-7f);
controller.computeGains();  // Uses stored parameters

// Query current settings
int iter = controller.getMaxIterations();
float tol = controller.getTolerance();
```

## Pre-Configured Modes

| Mode | Iterations | Tolerance | Use Case |
|------|------------|-----------|----------|
| Fast | 50 | 1e-4 | Quick computation, acceptable accuracy |
| Default | 100 | 1e-6 | Balanced performance |
| Precise | 200 | 1e-8 | Maximum accuracy |

## Features Demonstrated

1. **Performance Comparison**: Times each configuration
2. **Accuracy Comparison**: Shows gain differences between modes
3. **Interactive Testing**: Try different parameters in real-time

## Serial Commands

- `D` - Compute with default parameters (100 iter, 1e-6)
- `F` - Compute with fast parameters (50 iter, 1e-4)
- `P` - Compute with precise parameters (200 iter, 1e-8)
- `S[iter],[tol]` - Set custom parameters (e.g., `S150,0.0000001`)
- `I` - Show current settings

## Results Interpretation

The example shows that for well-conditioned systems:
- Fast mode may have slightly different gains (typically <0.1% difference)
- Default mode is usually sufficient
- Precise mode provides reference values

For ill-conditioned systems or when maximum accuracy is needed, use more iterations and tighter tolerance.

## When to Adjust Parameters

**Increase iterations/tighten tolerance when:**
- System is near the edge of controllability
- Very precise control is required
- Computation time is not critical

**Decrease iterations/loosen tolerance when:**
- Running on slow microcontrollers
- Gains need to be recomputed frequently
- System is well-conditioned
