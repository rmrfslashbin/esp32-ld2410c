# Documentation

This directory contains additional documentation for the LD2410C driver.

## Quick Links

### For Users
- **[Main README](../README.md)** - Getting started, API reference, examples
- **[Hardware Datasheets](../datasheets/)** - Protocol specs and hardware documentation
- **[Examples](../examples/)** - Working code examples
  - [Basic Usage](../examples/basic/) - Simple presence detection
  - [Engineering Mode](../examples/engineering_mode/) - Advanced diagnostics

### For Developers
- **[CONTRIBUTING.md](../CONTRIBUTING.md)** - Development guidelines and ESP-IDF standards
- **[CHANGELOG.md](../CHANGELOG.md)** - Version history and changes
- **[Unit Tests](../test/)** - Test suite and testing guidelines

## Documentation Structure

```
ld2410c/
├── README.md              # Main user documentation
├── CONTRIBUTING.md        # Developer guidelines
├── CHANGELOG.md           # Version history
├── LICENSE                # MIT License
├── docs/                  # This directory
│   ├── README.md          # This file
│   ├── architecture.md    # Driver architecture (coming soon)
│   └── troubleshooting.md # Detailed troubleshooting (coming soon)
├── datasheets/            # Hardware documentation
│   ├── LD2410C_Protocol.md        # Protocol reference (Markdown)
│   ├── HLK-LD2410C_Reference.md   # Hardware specs (Markdown)
│   └── *.pdf              # Original manufacturer PDFs
├── examples/              # Usage examples
└── test/                  # Unit tests
```

## Additional Resources

### ESP-IDF Documentation
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
- [ESP-IDF API Reference](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/index.html)
- [UART Driver Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/uart.html)

### Protocol Documentation
- **In this repo:** [datasheets/LD2410C_Protocol.md](../datasheets/LD2410C_Protocol.md)
- **Manufacturer:** https://www.hlktech.net/index.php?id=988

### Community Resources
- [ESPHome ld2410 Component](https://github.com/esphome/esphome/tree/dev/esphome/components/ld2410)
- [ESP32 Forum](https://www.esp32.com/)

## Documentation TODO

Future documentation to add:
- [ ] Architecture diagram and design decisions
- [ ] Detailed troubleshooting guide
- [ ] Performance tuning guide
- [ ] Integration examples with other sensors
- [ ] Bluetooth configuration guide
- [ ] Power optimization guide

## Contributing to Documentation

Documentation improvements are always welcome! Please see [CONTRIBUTING.md](../CONTRIBUTING.md) for guidelines.

### Documentation Standards
- Use Markdown format
- Include code examples where relevant
- Keep language clear and concise
- Add diagrams for complex concepts
- Cross-reference related documentation
