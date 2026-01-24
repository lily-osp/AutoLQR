# Static Allocation Example

This example demonstrates using `AutoLQRStatic<N,M>` for compile-time fixed memory allocation, suitable for production embedded systems.

## Overview

The static allocation variant provides:
- No heap fragmentation
- Predictable memory usage at compile time
- Configurable solver parameters
- Same functionality as dynamic version

## Key Differences from Dynamic Version

| Feature | AutoLQR | AutoLQRStatic<N,M> |
|---------|---------|-------------------|
| Memory | Heap (dynamic) | Stack/BSS (static) |
| Sizing | Runtime | Compile-time |
| Fragmentation | Possible | None |
| Max control size | Unlimited | 3 |

## System Model

Standard position-velocity system:
- **States**: Position and velocity (2 states)
- **Control**: Single input
- **Rate**: 10 Hz control loop

## Features

1. **Compile-Time Sizing**: Template parameters define system size
2. **Memory Calculation**: Reports exact memory usage
3. **Configurable Solver**: Custom iterations and tolerance
4. **Gain Validation**: `hasValidGains()` checks if gains are ready
5. **Riccati Solution Access**: Direct access to P matrix

## Usage

### Declaration
```cpp
#include <AutoLQRStatic.h>

// Compile-time fixed sizes: 2 states, 1 control
AutoLQRStatic<2, 1> controller;
```

### Memory Calculation
```cpp
// Access at compile time
constexpr size_t mem = AutoLQRStatic<2, 1>::MemorySize;

// Or at runtime
size_t bytes = controller.getMemorySize();
```

### Custom Solver Parameters
```cpp
// 150 iterations, 1e-7 tolerance
bool success = controller.computeGains(150, 1e-7f);
```

## Serial Commands

- `T[pos]` - Set target position (e.g., `T10` for position 10)
- `R` - Reset system state
- `S` - Show detailed status

## When to Use Static Allocation

Use `AutoLQRStatic` when:
- Running on memory-constrained devices
- Application runs for extended periods
- Predictable memory usage is required
- Control size is 3 or fewer inputs

Use `AutoLQR` (dynamic) when:
- Prototyping or testing
- System size may change at runtime
- More than 3 control inputs needed

## Memory Usage Examples

| Configuration | Memory (bytes) |
|--------------|----------------|
| 2 states, 1 control | ~73 |
| 3 states, 1 control | ~117 |
| 4 states, 2 control | ~193 |
