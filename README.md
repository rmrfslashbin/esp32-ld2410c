# HLK-LD2410C ESP-IDF Driver

Complete C implementation of the HLK-LD2410C human presence sensor driver for ESP-IDF.

**Author:** Robert Sigler (code@sigler.io)
**License:** MIT License

## Features

- Full protocol implementation based on official specification V1.00
- Non-blocking UART communication
- Thread-safe data access
- Basic and engineering modes
- Configuration support (sensitivity, gates, timeout)
- Automatic frame parsing and validation

## Hardware

- **Sensor**: HLK-LD2410C 24GHz mmWave radar presence sensor
- **Interface**: UART (TTL level, default 256000 baud, 8N1)
- **Power**: 5V (module supports 5-12V)

## Protocol

Based on:
- Official LD2410C Serial Communication Protocol V1.00
- ESPHome ld2410 component

### Frame Types

**Command Frames** (Host → Sensor):
```
[Header 0xFDFCFBFA] [Length 2B] [Command 2B] [Value NB] [Footer 0x04030201]
```

**Data Frames** (Sensor → Host):
```
[Header 0xF4F3F2F1] [Length 2B] [Type 1B] [Head 0xAA] [Data] [Tail 0x55] [Check 0x00] [Footer 0xF8F7F6F5]
```

### Initialization Sequence

The sensor boots in configuration/Bluetooth mode. To start streaming data:

1. Send `ENABLE_CONF` (0xFF) command
2. Optionally send configuration commands
3. Send `DISABLE_CONF` (0xFE) command → sensor starts streaming data frames

## Usage

### Basic Example

```c
#include "ld2410c.h"

void app_main(void) {
    // Configure sensor
    ld2410c_config_t config = {
        .uart_num = UART_NUM_1,
        .tx_pin = 43,
        .rx_pin = 44,
        .baud_rate = 256000,
        .max_move_gate = 8,
        .max_still_gate = 8,
        .timeout_seconds = 5,
        .engineering_mode = false
    };

    // Initialize
    ESP_ERROR_CHECK(ld2410c_init(&config));

    // Read sensor data
    while (1) {
        ld2410c_data_t data;
        if (ld2410c_get_data(&data) == ESP_OK) {
            if (data.presence_detected) {
                printf("Presence detected at %d cm\n", data.detection_distance);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### Check Connection Status

```c
if (ld2410c_is_connected(5)) {
    printf("Sensor is connected\n");
} else {
    printf("Sensor disconnected - no data received in last 5 seconds\n");
}
```

### Configure Sensitivity

```c
// Set gate 3 sensitivity: moving=40, stationary=40
ld2410c_set_gate_sensitivity(3, 40, 40);
```

### Enable Engineering Mode

```c
// Get detailed per-gate energy values
ld2410c_set_engineering_mode(true);

ld2410c_data_t data;
ld2410c_get_data(&data);

for (int i = 0; i < LD2410C_MAX_GATES; i++) {
    printf("Gate %d: move=%d, still=%d\n",
           i, data.gate_move_energy[i], data.gate_still_energy[i]);
}
```

## API Reference

### Initialization

- `ld2410c_init(config)` - Initialize driver and sensor
- `ld2410c_deinit()` - Cleanup and deinitialize

### Data Access

- `ld2410c_get_data(data)` - Get current sensor readings
- `ld2410c_is_connected(timeout)` - Check if sensor is connected

### Configuration

- `ld2410c_set_engineering_mode(enable)` - Enable/disable engineering mode
- `ld2410c_set_max_distances_timeout(move, still, timeout)` - Set detection gates and timeout
- `ld2410c_set_gate_sensitivity(gate, move, still)` - Set per-gate sensitivity

### System Commands

- `ld2410c_restart()` - Restart sensor module
- `ld2410c_factory_reset()` - Reset to factory defaults

## Data Structure

```c
typedef struct {
    // Target state
    bool presence_detected;
    bool moving_target;
    bool stationary_target;

    // Distances (cm)
    uint16_t moving_distance;
    uint16_t stationary_distance;
    uint16_t detection_distance;

    // Energy (0-100)
    uint8_t moving_energy;
    uint8_t stationary_energy;

    // Engineering mode: per-gate energy
    uint8_t gate_move_energy[9];
    uint8_t gate_still_energy[9];

    // Timestamp
    int64_t last_update_time;
} ld2410c_data_t;
```

## Configuration

### Kconfig Options

Configure the driver via `idf.py menuconfig` under "Component config" → "LD2410C Configuration":

- `LD2410C_UART_BUF_SIZE` - UART buffer size (default: 1024 bytes)
- `LD2410C_MAX_FRAME_SIZE` - Maximum frame size (default: 64 bytes)
- `LD2410C_ACK_TIMEOUT_MS` - Command acknowledgment timeout (default: 500 ms)
- `LD2410C_DEFAULT_BAUD_RATE` - Default baud rate (default: 256000)
- `LD2410C_UART_TASK_STACK_SIZE` - UART task stack size (default: 4096 bytes)
- `LD2410C_UART_TASK_PRIORITY` - UART task priority (default: 5)
- `LD2410C_ENABLE_DEBUG_LOGS` - Enable verbose debug logging (default: disabled)

### Default Configuration

- Max detection gate: 8 (both moving and stationary)
- Timeout: 5 seconds
- Baud rate: 256000
- Distance resolution: 0.75m per gate

## Examples

### Basic Example
See `examples/basic/` for a complete working example showing:
- Sensor initialization
- Reading presence data
- Connection monitoring

### Engineering Mode Example
See `examples/engineering_mode/` for advanced usage:
- Per-gate energy readings
- Sensitivity calibration
- Detection zone configuration

## Testing

### Unit Tests
```bash
cd test
idf.py build flash monitor
```

Run the test suite to verify:
- Input validation
- Error handling
- API compliance

See `test/README.md` for details.

## Compliance

This component follows ESP-IDF standards:
- ✅ ESP-IDF coding style guide
- ✅ Component structure requirements
- ✅ Kconfig integration
- ✅ Security best practices
- ✅ Thread safety
- ✅ Input validation

See `CONTRIBUTING.md` for development guidelines.

## Troubleshooting

### Sensor not sending data (0 bytes received)

**Cause**: Bluetooth/UART exclusivity - sensor can only use ONE interface at a time.

**Solution**:
1. Disconnect/unpair sensor from any Bluetooth devices
2. Turn off Bluetooth on phone
3. Power cycle the sensor
4. The initialization sequence sends the correct commands to exit config mode

### Initialization fails with ESP_ERR_INVALID_ARG

Check that:
- UART port is valid (UART_NUM_0, UART_NUM_1, or UART_NUM_2)
- GPIO pins are valid for your ESP32 variant
- Baud rate is supported (9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800)
- Detection gates are in range 2-8

### Compiler warnings

The component is configured with strict warnings. All warnings must be fixed:
```bash
idf.py build
# Should complete with no warnings from ld2410c component
```

### Memory issues

Default configuration uses:
- 1024 bytes UART RX buffer
- 64 bytes frame buffer
- 4096 bytes UART task stack
- ~200 bytes for driver state

Adjust via Kconfig if needed.

## Credits and Acknowledgments

This driver is based on and inspired by:

### 1. Official LD2410C Protocol Specification
- **Source**: Shenzhen Hi-Link Electronic Co., Ltd
- **Document**: HLK-LD2410C Serial Communication Protocol V1.00 (2022-11-7)
- **URL**: https://www.hlktech.net/index.php?id=988

### 2. ESPHome ld2410 Component
- **Primary Reference**: Implementation patterns and protocol handling
- **Authors**: ESPHome Contributors
- **Repository**: https://github.com/esphome/esphome/tree/dev/esphome/components/ld2410
- **License**: MIT License
- This driver follows ESPHome's proven approach to frame parsing and state management

### 3. HLK_LD2410C_Lib (Reference Only)
- **Author**: BorseAndy
- **Repository**: https://github.com/BorseAndy/HLK_LD2410C_Lib
- **Usage**: Validation of initialization sequence
- **Note**: This C++ library was incomplete; our implementation is independent

## Implementation

- **Protocol**: Strictly follows official Hi-Link specification
- **Architecture**: Heavily inspired by ESPHome's battle-tested component design
- **Language**: Complete rewrite in C for ESP-IDF (from ESPHome's C++)
- **Enhancements**: Thread-safety, connection detection, comprehensive error handling

## License

MIT License - Compatible with all referenced sources

## Datasheets

Official documentation is available in the `datasheets/` directory. See `datasheets/README.md` for:
- Serial Communication Protocol V1.00
- Product datasheet and specifications
- Pin definitions and electrical characteristics

## References

- [Official Protocol Specification](https://www.hlktech.net/index.php?id=988)
- [ESPHome ld2410 Component](https://github.com/esphome/esphome/tree/dev/esphome/components/ld2410)
- [Datasheet](https://drive.google.com/file/d/1CYgZTUEJRasIE8QZA22C3LO2NItiWSYs/view)
