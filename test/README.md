# LD2410C Unit Tests

This directory contains unit tests for the LD2410C driver.

## Test Categories

### Software Tests (No Hardware Required)
- Input validation
- Parameter checking
- State management
- Error handling

### Hardware Tests (Requires Sensor)
- UART communication
- Frame parsing
- Data acquisition
- Configuration commands

## Running Tests

### All Tests
```bash
cd <your-esp-idf-project>
idf.py build
idf.py flash monitor
```

### Specific Test
```bash
# Run only software tests (no hardware)
idf.py build
idf.py flash monitor -p /dev/ttyUSB0
# In the test menu, select tests without [hardware] tag
```

## Test Structure

Tests use the Unity test framework included with ESP-IDF.

### Test Naming Convention
- `test_<function>_<scenario>_<expected_result>()`
- Example: `test_init_with_null_config_returns_error()`

### Test Tags
- `[ld2410c]` - All driver tests
- `[ld2410c][hardware]` - Requires physical sensor

## Adding New Tests

1. Create test function:
```c
TEST_CASE("description", "[ld2410c]")
{
    // Arrange
    ld2410c_config_t config = {...};

    // Act
    esp_err_t ret = ld2410c_init(&config);

    // Assert
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}
```

2. Rebuild and run:
```bash
idf.py build flash monitor
```

## Current Test Coverage

- [x] Init parameter validation
- [x] NULL pointer checks
- [x] Invalid UART configuration
- [x] Invalid GPIO pins
- [x] Invalid gate ranges
- [x] Invalid baud rates
- [x] Double initialization
- [ ] Frame parsing (TODO)
- [ ] UART communication (TODO)
- [ ] Configuration commands (TODO)

## Mock Support (Future)

For hardware-independent testing, consider using:
- CMock for UART driver mocking
- Custom test fixtures for frame parsing
- Simulated sensor responses

## Continuous Integration

Tests are automatically run in CI via GitHub Actions:
- Software tests run on every commit
- Hardware tests require physical setup
