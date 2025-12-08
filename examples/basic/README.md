# LD2410C Basic Example

This example demonstrates basic usage of the LD2410C driver.

## Features

- Initialize sensor with default settings
- Read presence detection data
- Monitor connection status
- Display moving and stationary target information

## Hardware Required

- ESP32 development board
- HLK-LD2410C sensor module
- Connecting wires

## Wiring

| ESP32 Pin | LD2410C Pin |
|-----------|-------------|
| GPIO 17   | RX          |
| GPIO 18   | TX          |
| 5V        | VCC         |
| GND       | GND         |

**Note:** Adjust pin numbers in `main.c` if using different GPIOs.

## How to Use

1. Connect the hardware as shown above
2. Configure your ESP-IDF environment
3. Build and flash:
   ```bash
   idf.py build
   idf.py flash monitor
   ```

## Expected Output

```
I (123) ld2410c_example: LD2410C Basic Example
I (145) ld2410c: Initializing LD2410C driver
I (156) ld2410c: UART config: TX=17, RX=18, Baud=256000
I (234) ld2410c_example: LD2410C initialized successfully
I (245) ld2410c_example: Waiting for sensor data...
I (2345) ld2410c_example: === PRESENCE DETECTED ===
I (2345) ld2410c_example:   Moving target: 120 cm (energy: 45)
I (2345) ld2410c_example:   Detection distance: 120 cm
```

## Troubleshooting

### No data received

1. Check wiring connections
2. Verify sensor is powered (5V)
3. Ensure sensor is not paired via Bluetooth
4. Power cycle the sensor

### Sensor disconnected

- Check UART baud rate (default: 256000)
- Verify TX/RX pins are not swapped
- Ensure no other device is using the UART port
