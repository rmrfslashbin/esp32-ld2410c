# Changelog

All notable changes to the LD2410C driver will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-01-XX

### Added
- Initial release of LD2410C driver for ESP-IDF
- Full implementation of LD2410C serial communication protocol V1.00
- Support for basic and engineering modes
- Thread-safe data access with FreeRTOS mutexes
- Configuration via Kconfig
- Component manifest (idf_component.yml) for ESP Component Registry
- Comprehensive input validation
- Security hardening (buffer overflow protection, bounds checking)
- Unit test infrastructure
- Example projects:
  - Basic presence detection
  - Engineering mode with gate energy visualization
- API functions:
  - `ld2410c_init()` - Initialize driver
  - `ld2410c_deinit()` - Cleanup driver
  - `ld2410c_get_data()` - Read sensor data
  - `ld2410c_is_connected()` - Check sensor connection
  - `ld2410c_set_engineering_mode()` - Enable/disable engineering mode
  - `ld2410c_set_max_distances_timeout()` - Configure detection gates
  - `ld2410c_set_gate_sensitivity()` - Set per-gate sensitivity
  - `ld2410c_restart()` - Restart sensor
  - `ld2410c_factory_reset()` - Reset to factory defaults

### Security
- Buffer overflow protection in `send_command()`
- Integer overflow prevention
- Race condition fixes with proper mutex usage
- Input validation for all public APIs
- GPIO pin range validation
- UART port validation
- Baud rate validation
- Gate number and sensitivity validation

### Documentation
- Comprehensive README with usage examples
- API reference with Doxygen comments
- CONTRIBUTING.md with ESP-IDF coding standards
- Example projects with detailed READMEs
- Troubleshooting guide
- Protocol specification references

### Build System
- ESP-IDF component structure compliance
- Kconfig integration for runtime configuration
- CMakeLists.txt with security hardening flags
- CI/CD with GitHub Actions
- Support for ESP-IDF v5.0+
- Support for ESP32, ESP32-S3, ESP32-C3

### Code Quality
- Follows ESP-IDF coding style guide
- Static variable naming with `s_` prefix
- Proper include file ordering
- Named constants instead of magic numbers
- Comprehensive error handling
- No compiler warnings with -Wall -Wextra
- Static analysis with cppcheck and flawfinder

## [Unreleased]

### TODO
- Implement `ld2410c_get_version()` - Read firmware version
- Implement baud rate configuration
- Implement Bluetooth control commands
- Implement distance resolution configuration
- Add MAC address query function
- Add Bluetooth password configuration
- Add response/ACK parsing for configuration commands
- Add hardware-in-loop tests
- Add mock UART for unit testing
- Add code coverage reporting
- Add example for custom sensitivity profiles

### Known Limitations
- `ld2410c_get_version()` not yet implemented (returns ESP_ERR_NOT_SUPPORTED)
- No ACK verification for configuration commands
- No retry mechanism for failed commands
- UART RX task runs indefinitely (normal operation)
- Engineering mode increases UART bandwidth

### Breaking Changes
None (initial release)

---

## Version History

- **1.0.0** - Initial stable release with full protocol support
