# LD2410C Engineering Mode Example

This example demonstrates advanced usage with engineering mode enabled.

## Features

- Enable engineering mode for detailed diagnostics
- View per-gate energy values (9 gates)
- Configure gate-specific sensitivity
- Calibrate detection thresholds

## What is Engineering Mode?

Engineering mode provides detailed energy readings for each of the 9 detection gates. Each gate represents a distance range:

| Gate | Distance | Range       |
|------|----------|-------------|
| 0    | 0.75m    | 0-0.75m     |
| 1    | 1.5m     | 0.75-1.5m   |
| 2    | 2.25m    | 1.5-2.25m   |
| 3    | 3.0m     | 2.25-3.0m   |
| 4    | 3.75m    | 3.0-3.75m   |
| 5    | 4.5m     | 3.75-4.5m   |
| 6    | 5.25m    | 4.5-5.25m   |
| 7    | 6.0m     | 5.25-6.0m   |
| 8    | 6.75m    | 6.0-6.75m   |

## Hardware Required

Same as basic example.

## How to Use

1. Connect hardware
2. Build and flash:
   ```bash
   idf.py build
   idf.py flash monitor
   ```
3. Observe gate energy values
4. Adjust sensitivities as needed

## Expected Output

```
========== Gate Energy Report ==========
Presence: DETECTED
Moving: 180cm (energy:55) | Still: 0cm (energy:0)

Gate | Distance | Move Energy | Still Energy
-----|----------|-------------|-------------
  0  |  0.75m   |      5      |      3
  1  |  1.50m   |     12      |      8
  2  |  2.25m   |     45 *    |     15
  3  |  3.00m   |     38      |     10
  4  |  3.75m   |     18      |      5
  5  |  4.50m   |      8      |      2
  6  |  5.25m   |      3      |      1
  7  |  6.00m   |      1      |      0
  8  |  6.75m   |      0      |      0
========================================
* = Energy > 40 (high activity)
```

## Calibration Tips

1. **Reduce false positives**: Lower sensitivity on gates with frequent noise
2. **Increase detection range**: Raise sensitivity on distant gates
3. **Fine-tune zones**: Adjust per-gate to create detection zones
4. **Test environment**: Walk at different distances and observe gate energies

## Example: Reduce Gate 2 Sensitivity

```c
// If gate 2 shows noise, reduce sensitivity
ld2410c_set_gate_sensitivity(2, 30, 30);  // Lower from default
```

## Notes

- Engineering mode increases UART traffic
- Energy values range from 0-100
- Higher values indicate stronger detection
- Use this mode for initial setup and calibration
