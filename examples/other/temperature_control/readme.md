# Temperature Control System Example

LQR-based temperature control for thermal systems such as 3D printer hotends, soldering stations, or industrial heaters.

## System Model

The thermal system is modeled as a second-order system:

- **State Vector**: `[temperature, temperature_rate]`
- **Control Input**: Heater PWM duty cycle (0-255)

### State-Space Representation

```
x(k+1) = A * x(k) + B * u(k)
```

Where:
- `x = [T, dT/dt]` (temperature and its rate of change)
- `u = PWM` (heater control signal)

The system accounts for:
- Thermal inertia (time constant)
- Natural cooling to ambient
- Heater power gain

## Hardware Requirements

| Component | Description | Pin |
|-----------|-------------|-----|
| Thermistor | NTC 100K (3950 beta) | A0 |
| MOSFET/SSR | Heater switching | D9 |
| Heater | Cartridge heater or coil | - |
| Status LED | Heating indicator | D13 |

### Circuit Notes

1. Use a logic-level MOSFET (IRLZ44N or similar) for PWM control
2. Add flyback diode if using inductive loads
3. Thermistor requires series resistor (10K default)
4. Ensure adequate current handling for heater power

## Configuration

Adjust these parameters in the code for your specific thermal system:

```cpp
const float thermalTimeConstant = 30.0f;  // Time to reach 63% of target
const float heaterGain = 0.5f;            // Degrees/second at full power
const float coolingRate = 0.02f;          // Natural cooling coefficient
```

### Thermistor Parameters

```cpp
#define THERMISTOR_NOMINAL 100000  // Resistance at 25C
#define B_COEFFICIENT 3950         // Beta coefficient
#define SERIES_RESISTOR 10000      // Series resistor value
```

## Serial Commands

| Command | Description | Example |
|---------|-------------|---------|
| `Txx` | Set target temperature | `T200` for 200C |
| `P` | Toggle temperature profile | Constant/Ramp/Step |
| `S` | Print detailed status | - |
| `R` | Reset controller | - |

## Temperature Profiles

1. **Constant**: Maintains fixed setpoint
2. **Ramp**: Gradually approaches target at configurable rate
3. **Step**: Immediate step change to target

## Tuning

### Cost Matrices

```cpp
Q[0] = 10.0f;   // Temperature error weight
Q[3] = 1.0f;    // Rate error weight
R[0] = 0.1f;    // Control effort weight
```

- Increase `Q[0]` for tighter temperature tracking
- Increase `Q[3]` to reduce oscillation
- Increase `R[0]` for smoother, slower response

### System Identification

To determine thermal parameters for your system:

1. Apply 50% PWM and record temperature rise
2. Measure time to reach 63% of final temperature (time constant)
3. Measure final temperature rise per PWM unit (heater gain)
4. Turn off heater and measure cooling rate

## Safety Features

- Maximum temperature limit (280C default)
- Automatic heater shutoff on over-temperature
- PWM reduction near setpoint
- Status LED indicates heating state

## Performance Metrics

The example tracks:
- Maximum overshoot
- Settling time (within 2C of target)
- Integral absolute error
- LQR cost function value
