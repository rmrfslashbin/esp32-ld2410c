# Contributing to LD2410C Driver

Thank you for considering contributing to the LD2410C ESP-IDF driver!

## Development Environment

### Requirements
- ESP-IDF v5.0 or later
- Python 3.8+
- Git

### Setup
```bash
# Clone ESP-IDF
git clone -b v5.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh

# Clone this component
cd ~/your-project/components
git clone <repository-url> ld2410c
```

## ESP-IDF Coding Standards

This project strictly follows the [ESP-IDF Style Guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/contribute/style-guide.html).

### Key Requirements

#### Naming Conventions
- **Functions**: `component_function_name()` (e.g., `ld2410c_init()`)
- **Static variables**: `s_variable_name` prefix
- **Types**: `snake_case_t` suffix (e.g., `ld2410c_data_t`)
- **Enums**: `MODULE_NAME_VALUE` (e.g., `LD2410C_TARGET_MOVING`)
- **Defines**: `MODULE_NAME_CONSTANT` (e.g., `LD2410C_MAX_GATES`)

#### Formatting
- **Indentation**: 4 spaces (NO tabs)
- **Line length**: 120 characters maximum
- **Braces**: K&R style for functions, same-line for conditionals
  ```c
  void function(void)  // Opening brace on new line
  {
      if (condition) {  // Opening brace on same line
          // code
      }
  }
  ```

#### Header Files
- Use `#pragma once` (preferred) or traditional guards
- Include `extern "C"` guards for C++ compatibility
- Include order:
  1. C standard library (`<stdio.h>`)
  2. POSIX headers (`<sys/types.h>`)
  3. ESP-IDF common headers (`esp_log.h`, `esp_err.h`)
  4. FreeRTOS headers (`freertos/FreeRTOS.h`)
  5. Component public headers (`ld2410c.h`)
  6. Private headers

#### Code Style
- **Comments**: Use `//` for single-line, `/* */` for multi-line
- **Error checking**: Always check return values
- **Logging**: Use ESP_LOG macros (ESP_LOGI, ESP_LOGW, ESP_LOGE)
- **No dead code**: Remove instead of commenting out
- **Thread safety**: Use mutexes for shared data access

## Component Structure

```
ld2410c/
├── include/
│   └── ld2410c.h           # Public API header
├── ld2410c.c               # Implementation
├── CMakeLists.txt          # Build configuration
├── Kconfig                 # Configuration options
├── idf_component.yml       # Component manifest
├── README.md               # User documentation
├── CONTRIBUTING.md         # This file
├── LICENSE                 # MIT license
├── examples/
│   ├── basic/              # Basic usage example
│   └── engineering_mode/   # Advanced example
└── test/
    ├── test_ld2410c.c      # Unit tests
    └── CMakeLists.txt      # Test build config
```

## Making Changes

### 1. Code Changes

#### Before Submitting
1. Follow ESP-IDF coding style
2. Add/update documentation
3. Write unit tests
4. Test on real hardware if possible
5. Run static analysis

#### Code Quality Checklist
- [ ] Functions have Doxygen comments
- [ ] Input parameters validated
- [ ] Error handling implemented
- [ ] No compiler warnings
- [ ] No memory leaks
- [ ] Thread-safe where needed
- [ ] Follows naming conventions
- [ ] Magic numbers replaced with constants

### 2. Security Requirements

All code must pass security review:

#### Buffer Safety
- Validate array bounds before access
- Check buffer sizes before memcpy/strcpy
- Prevent integer overflows
- Use safe string functions

#### Input Validation
- Validate all public API parameters
- Check pointer arguments for NULL
- Validate ranges (GPIO pins, UART ports, etc.)
- Sanitize user-provided data

#### Thread Safety
- Protect shared data with mutexes
- Avoid race conditions
- Use atomic operations where appropriate

### 3. Testing

#### Unit Tests
```bash
cd test
idf.py build flash monitor
```

#### Hardware Tests
- Test with actual LD2410C sensor
- Verify all API functions
- Test error conditions
- Check memory usage

#### Static Analysis
```bash
# cppcheck
cppcheck --enable=all --inconclusive --std=c11 \
    -I include/ ld2410c.c

# flawfinder
flawfinder --minlevel=1 ld2410c.c include/
```

### 4. Documentation

Update documentation for:
- New functions (Doxygen comments)
- Configuration options (Kconfig)
- Examples (if API changes)
- README.md (if user-facing changes)

#### Doxygen Format
```c
/**
 * @brief Brief description
 *
 * Detailed description
 *
 * @param param1 Description of param1
 * @param param2 Description of param2
 * @return ESP_OK on success
 *         ESP_ERR_INVALID_ARG if parameters invalid
 *         ESP_FAIL on communication error
 *
 * @note Any important notes
 * @warning Any warnings
 */
esp_err_t function(int param1, bool param2);
```

## Pull Request Process

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to the branch (`git push origin feature/amazing-feature`)
5. **Open** a Pull Request

### PR Requirements
- [ ] Descriptive title and description
- [ ] Reference related issues
- [ ] All tests passing
- [ ] No compiler warnings
- [ ] Documentation updated
- [ ] Follows ESP-IDF style guide
- [ ] Security review passed

### PR Template
```markdown
## Description
Brief description of changes

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Testing
- [ ] Unit tests added/updated
- [ ] Tested on hardware
- [ ] No regressions

## Checklist
- [ ] Follows ESP-IDF coding style
- [ ] Documentation updated
- [ ] Tests passing
- [ ] No compiler warnings
```

## Reporting Issues

### Bug Reports
Include:
- ESP-IDF version
- Hardware platform (ESP32, ESP32-S3, etc.)
- Sensor model (LD2410C)
- Steps to reproduce
- Expected vs actual behavior
- Logs (with debug enabled if possible)

### Feature Requests
Include:
- Use case description
- Proposed API
- Alternatives considered
- Willingness to contribute

## Code Review

All submissions require review. We review for:
- Code correctness
- ESP-IDF compliance
- Security vulnerabilities
- Performance impact
- Documentation quality
- Test coverage

## License

By contributing, you agree that your contributions will be licensed under the MIT License.

## Questions?

- Open an issue for clarification
- Reference ESP-IDF style guide
- Check existing code for examples

## Resources

### ESP-IDF Documentation
- [ESP-IDF Style Guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/contribute/style-guide.html)
- [ESP-IDF Build System](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html)
- [ESP-IDF Component Manager](https://docs.espressif.com/projects/idf-component-manager/)

### LD2410C Documentation
- **In this repository:** `datasheets/LD2410C_Protocol.md` (Markdown, AI-optimized)
- **In this repository:** `datasheets/HLK-LD2410C_Reference.md` (Markdown, AI-optimized)
- [Official Protocol Specification PDF](https://www.hlktech.net/index.php?id=988)

The Markdown files in `datasheets/` are AI-friendly conversions of the official PDFs, optimized for quick reference during development.
