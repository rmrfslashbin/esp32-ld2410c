# HLK-LD2410C Human Presence Motion Module - Implementation Reference

## Overview

The HLK-LD2410C is a 24GHz FMCW (Frequency-Modulated Continuous Wave) millimeter-wave radar module for human presence detection. It can detect both moving and stationary (micro-moving) humans within a configurable range.

**Key Capabilities:**
- Detects moving and stationary humans (sitting, lying, standing)
- Maximum detection range: 5 meters (configurable)
- Detection angle: ±60°
- Distance resolution: 0.75m per "distance gate"
- Outputs via GPIO (high/low) and UART (detailed data)
- Configurable via serial commands or Bluetooth

---

## Hardware Specifications

### Module Dimensions
- Size: 16mm × 22mm
- Pin hole diameter: 0.9mm
- Pin spacing: 2.54mm

### Pinout

| Pin | Symbol   | Function              | Description                                      |
|-----|----------|-----------------------|--------------------------------------------------|
| 1   | UART_TX  | Serial Transmit       | UART TX pin (3.3V logic)                         |
| 2   | UART_RX  | Serial Receive        | UART RX pin (3.3V logic)                         |
| 3   | OUT      | Target Status Output  | HIGH = presence detected, LOW = no presence      |
| 4   | GND      | Ground                | Power ground                                     |
| 5   | VCC      | Power Input           | 5V–12V DC (5V recommended)                       |

### Electrical Characteristics

| Parameter              | Value                          |
|------------------------|--------------------------------|
| Operating Frequency    | 24.0 – 24.25 GHz               |
| Operating Voltage      | 5V DC (range: 5–12V)           |
| Power Supply Capacity  | > 200mA required               |
| Average Current        | 79 mA                          |
| IO Output Level        | 3.3V                           |
| Operating Temperature  | -40°C to +85°C                 |

---

## Serial Communication

### UART Configuration

| Parameter    | Value    |
|--------------|----------|
| Baud Rate    | 256000   |
| Data Bits    | 8        |
| Stop Bits    | 1        |
| Parity       | None     |
| Flow Control | None     |

### Important Notes
- The module outputs detection data continuously via UART
- Configuration commands can only be sent when data streaming is stopped
- All configuration is persistent (saved to flash)

---

## Configuration Parameters

### Distance Gates

The detection range is divided into **distance gates**, each representing **0.75 meters**:

| Gate | Distance Range    |
|------|-------------------|
| 0    | 0 – 0.75m         |
| 1    | 0.75 – 1.5m       |
| 2    | 1.5 – 2.25m       |
| 3    | 2.25 – 3.0m       |
| 4    | 3.0 – 3.75m       |
| 5    | 3.75 – 4.5m       |
| 6    | 4.5 – 5.25m       |
| 7    | 5.25 – 6.0m       |
| 8    | 6.0 – 6.75m       |

**Gate configuration range:** 1–8

### Configurable Parameters

#### 1. Maximum Detection Distance
- **Motion detection gate:** Sets farthest gate for detecting moving targets
- **Static detection gate:** Sets farthest gate for detecting stationary targets
- Both configurable from gate 1 to gate 8

#### 2. Sensitivity (per gate)
- Range: 0–100
- Lower value = more sensitive
- Higher value = less sensitive
- **Setting 100 = disable detection at that gate**
- Each gate has independent motion and static sensitivity values

**Default Sensitivity Values (typical):**

| Gate | Motion Sensitivity | Static Sensitivity |
|------|--------------------|--------------------|
| 0    | 50                 | 0                  |
| 1    | 50                 | 0                  |
| 2    | 40                 | 40                 |
| 3    | 30                 | 40                 |
| 4    | 20                 | 30                 |
| 5    | 15                 | 30                 |
| 6    | 15                 | 20                 |
| 7    | 15                 | 20                 |
| 8    | 15                 | 20                 |

#### 3. No-One Duration (Timeout)
- Time in seconds before reporting "no presence" after target leaves
- Range: 0–65535 seconds
- Acts as a delay/debounce for the presence output

---

## Output Modes

### GPIO Output (OUT Pin)
- **HIGH (3.3V):** Human presence detected
- **LOW (0V):** No human presence
- Simple binary output for basic automation

### UART Output
The module outputs detailed data including:
- Target state (moving, stationary, no target)
- Detection distance
- Energy values for each distance gate
- Motion energy and static energy values

**Output Modes:**
1. **Basic Mode:** Target state and distance only
2. **Engineering Mode:** Full gate-by-gate energy data

---

## Target States

| State Code | Description        |
|------------|--------------------|
| 0x00       | No target          |
| 0x01       | Moving target      |
| 0x02       | Stationary target  |
| 0x03       | Both detected      |

---

## Bluetooth Interface

### Connection Details
- Broadcast name: `HLK-LD2410B_xxxx` (xxxx = last 4 digits of MAC address)
- Default password: `HiLink` (case-sensitive, 6 characters)
- Effective range: ~4 meters

### BLE UUIDs

| UUID                                 | Permission             | Direction           |
|--------------------------------------|------------------------|---------------------|
| `0000fff1-0000-1000-8000-00805f9b34fb` | Read/Notify            | Module → App        |
| `0000fff2-0000-1000-8000-00805f9b34fb` | Write Without Response | App → Module        |

### Bluetooth Notes
- Uses same protocol as UART (transparent passthrough)
- Only one master connection allowed
- Password required before data transmission begins
- Bluetooth enabled by default
- Can be re-enabled by power cycling 5+ times within 2–3 seconds

---

## Installation Guidelines

### Mounting Positions

**Ceiling Mount:**
- Height: 2.6–3.0m above floor
- Antenna facing downward

**Wall Mount:**
- Height: 1.5–2.0m above floor
- Antenna facing detection area
- Maximum horizontal range: 5m

### Environmental Considerations

**Avoid:**
- Moving non-human objects (curtains, plants near air vents, pets)
- Large reflective surfaces
- Air conditioners/fans in detection zone
- Vibration or movement behind the sensor

**Best Practices:**
- Use metal shield behind sensor to block back-lobe detection
- Ensure firm, stable mounting
- Keep antenna area unobstructed
- If using enclosure, ensure 24GHz wave transparency (no metal)

---

## Radome Design (Enclosure Guidelines)

### Spacing from Antenna to Enclosure
- Recommended: 1× or 1.5× wavelength
- For 24.125 GHz: **12.4mm or 18.6mm** (±1.2mm tolerance)

### Material Thickness
- Ideal: Half wavelength in the material
- Alternative: 1/8 wavelength or thinner for low-εr materials

### Common Material Properties @ 24.125 GHz

| Material        | εr (Dielectric Constant) | Half Wavelength (mm) |
|-----------------|--------------------------|----------------------|
| Air             | 1.00                     | 6.20                 |
| ABS (type 1)    | 1.50                     | 5.06                 |
| ABS (type 2)    | 2.50                     | 3.92                 |
| PC (Polycarbonate) | 3.00                  | 3.58                 |
| PMMA Acrylic    | 2.00                     | 4.38                 |
| High Density PE | 2.40                     | 4.00                 |
| Low Density PE  | 2.30                     | 4.09                 |

---

## Implementation Checklist

### Basic Integration (GPIO Only)
- [ ] Connect VCC to 5V (>200mA capable supply)
- [ ] Connect GND
- [ ] Connect OUT pin to MCU input (3.3V compatible)
- [ ] Handle HIGH/LOW state changes

### Full Integration (UART)
- [ ] Configure UART: 256000 baud, 8N1
- [ ] Implement serial protocol parser
- [ ] Handle command mode vs streaming mode
- [ ] Implement configuration commands
- [ ] Parse detection data frames

### Bluetooth Integration
- [ ] Scan for `HLK-LD2410B_*` devices
- [ ] Connect and authenticate with password
- [ ] Subscribe to notification UUID
- [ ] Send commands via write UUID
- [ ] Use same protocol as UART

---

## Quick Reference

```
Power:          5V DC, >200mA
UART:           256000 baud, 8N1
GPIO Output:    3.3V logic, HIGH=presence
Detection:      0.75m to 6m range
Resolution:     0.75m per gate
Gates:          0-8 (configurable)
Sensitivity:    0-100 per gate (lower=more sensitive)
Timeout:        0-65535 seconds
BLE Password:   "HiLink" (default)
```

---

## External Resources

- Manufacturer: Shenzhen Hi-Link Electronic Co., Ltd
- Website: www.hlktech.net
- Mobile App: "HLKRadarTools" (Android/iOS)
- Additional documentation: LD2410C Serial Port Communication Protocol.pdf (separate document with full protocol specification)

---

*Document generated from HLK-LD2410C User Manual V1.00 (2022-11-7)*
