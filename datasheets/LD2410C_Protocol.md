# HLK-LD2410C Human Presence Sensing Module - Serial Protocol Reference

> **Version:** 1.00 | **Date:** 2022-11-7 | **Manufacturer:** Shenzhen Hi-Link Electronic Co., Ltd

This document provides a complete implementation reference for the LD2410C mmWave radar presence detection module serial communication protocol.

---

## Table of Contents

1. [Hardware Interface](#1-hardware-interface)
2. [Serial Configuration](#2-serial-configuration)
3. [Protocol Fundamentals](#3-protocol-fundamentals)
4. [Command Reference](#4-command-reference)
5. [Data Output Protocol](#5-data-output-protocol)
6. [Implementation Examples](#6-implementation-examples)
7. [Configuration Defaults](#7-configuration-defaults)

---

## 1. Hardware Interface

### Pin Definition

| Pin | Symbol   | Name               | Function                                              |
|-----|----------|--------------------|-------------------------------------------------------|
| 1   | UART_Tx  | UART Transmit      | Module TX → Host RX                                   |
| 2   | UART_Rx  | UART Receive       | Host TX → Module RX                                   |
| 3   | OUT      | Target State Output| HIGH = presence detected, LOW = no presence           |
| 4   | GND      | Power Ground       | Ground reference                                      |
| 5   | VCC      | Power Input        | 5V-12V DC (5V recommended), ≥200mA supply capacity   |

### Electrical Characteristics

- **IO Output Level:** 3.3V
- **Power Supply:** 5V-12V DC (5V recommended)
- **Minimum Current Capacity:** 200mA

---

## 2. Serial Configuration

| Parameter    | Value                |
|--------------|----------------------|
| Baud Rate    | 256000 (default)     |
| Data Bits    | 8                    |
| Stop Bits    | 1                    |
| Parity       | None                 |
| Byte Order   | Little-endian        |

**Available Baud Rates:**

| Index  | Baud Rate |
|--------|-----------|
| 0x0001 | 9600      |
| 0x0002 | 19200     |
| 0x0003 | 38400     |
| 0x0004 | 57600     |
| 0x0005 | 115200    |
| 0x0006 | 230400    |
| 0x0007 | 256000    |
| 0x0008 | 460800    |

---

## 3. Protocol Fundamentals

### 3.1 Data Format

- All multi-byte values use **little-endian** byte order
- All values in this document are in **hexadecimal**

### 3.2 Command Frame Format (Host → Module)

```
┌──────────────┬─────────────────────┬────────────────────────────────┬──────────────┐
│ Frame Header │ Intra-frame Length  │         Intra-frame Data       │ End of Frame │
├──────────────┼─────────────────────┼────────────────────────────────┼──────────────┤
│ FD FC FB FA  │    2 bytes (LE)     │ Command Word (2B) + Value (NB) │ 04 03 02 01  │
└──────────────┴─────────────────────┴────────────────────────────────┴──────────────┘
```

**Constants:**
```
COMMAND_FRAME_HEADER = [0xFD, 0xFC, 0xFB, 0xFA]
COMMAND_FRAME_END    = [0x04, 0x03, 0x02, 0x01]
```

### 3.3 ACK Frame Format (Module → Host)

```
┌──────────────┬─────────────────────┬─────────────────────────────────────────┬──────────────┐
│ Frame Header │ Intra-frame Length  │              Intra-frame Data           │ End of Frame │
├──────────────┼─────────────────────┼─────────────────────────────────────────┼──────────────┤
│ FD FC FB FA  │    2 bytes (LE)     │ (CMD_WORD | 0x0100) (2B) + Return (NB)  │ 04 03 02 01  │
└──────────────┴─────────────────────┴─────────────────────────────────────────┴──────────────┘
```

**ACK Command Word:** Original command word OR'd with `0x0100`

**ACK Status Values:**
- `0x0000` = Success
- `0x0001` = Failure

### 3.4 Command Workflow

Commands must follow this sequence:

1. Send **Enable Configuration** command (`0x00FF`)
2. Wait for successful ACK
3. Send desired command(s)
4. Wait for successful ACK(s)
5. Send **End Configuration** command (`0x00FE`)
6. Wait for successful ACK

---

## 4. Command Reference

### 4.1 Enable Configuration (Required First)

**Must be sent before any other configuration commands.**

| Field         | Value                                                              |
|---------------|--------------------------------------------------------------------|
| Command Word  | `0x00FF`                                                           |
| Command Value | `0x0001`                                                           |
| Return Value  | 2B status + 2B protocol version (`0x0001`) + 2B buffer size (`0x0040`) |

**Send:**
```
FD FC FB FA 04 00 FF 00 01 00 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 08 00 FF 01 00 00 01 00 40 00 04 03 02 01
```

---

### 4.2 End Configuration

**Ends configuration mode and resumes normal operation.**

| Field         | Value          |
|---------------|----------------|
| Command Word  | `0x00FE`       |
| Command Value | None           |
| Return Value  | 2B status      |

**Send:**
```
FD FC FB FA 02 00 FE 00 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 04 00 FE 01 00 00 04 03 02 01
```

---

### 4.3 Set Maximum Distance Gate and Timeout

**Configures detection range and no-presence timeout. Persists across power cycles.**

| Field         | Value                                                    |
|---------------|----------------------------------------------------------|
| Command Word  | `0x0060`                                                 |
| Command Value | See parameter structure below                            |
| Return Value  | 2B status                                                |

**Parameter Words:**

| Parameter                   | Word     |
|-----------------------------|----------|
| Maximum motion distance gate| `0x0000` |
| Maximum static distance gate| `0x0001` |
| No-presence timeout         | `0x0002` |

**Command Value Structure (18 bytes):**
```
[2B param word 0x0000] [4B max motion gate (2-8)]
[2B param word 0x0001] [4B max static gate (2-8)]
[2B param word 0x0002] [4B timeout in seconds (0-65535)]
```

**Example:** Max gate 8 (both), timeout 5 seconds:
```
FD FC FB FA 14 00 60 00 00 00 08 00 00 00 01 00 08 00 00 00 02 00 05 00 00 00 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 04 00 60 01 00 00 04 03 02 01
```

---

### 4.4 Read Parameters

**Reads current configuration parameters.**

| Field         | Value          |
|---------------|----------------|
| Command Word  | `0x0061`       |
| Command Value | None           |

**Return Value Structure:**
```
[2B status]
[1B header 0xAA]
[1B max distance gate N (0x08)]
[1B configured max motion gate]
[1B configured max static gate]
[N+1 bytes: motion sensitivity for gates 0-N]
[N+1 bytes: static sensitivity for gates 0-N]
[2B no-presence timeout]
```

**Send:**
```
FD FC FB FA 02 00 61 00 04 03 02 01
```

**ACK Example (max gate 8, motion gate 8, static gate 8, sensitivities 20, timeout 5s):**
```
FD FC FB FA 1C 00 61 01 00 00 AA 08 08 08
14 14 14 14 14 14 14 14 14    <- Motion sensitivities (gates 0-8, hex 0x14 = 20)
19 19 19 19 19 19 19 19 19    <- Static sensitivities (gates 0-8, hex 0x19 = 25)
05 00                          <- Timeout (5 seconds)
04 03 02 01
```

---

### 4.5 Enable Engineering Mode

**Enables detailed per-gate energy reporting. Lost on power cycle.**

| Field         | Value          |
|---------------|----------------|
| Command Word  | `0x0062`       |
| Command Value | None           |
| Return Value  | 2B status      |

**Send:**
```
FD FC FB FA 02 00 62 00 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 04 00 62 01 00 00 04 03 02 01
```

---

### 4.6 Disable Engineering Mode

| Field         | Value          |
|---------------|----------------|
| Command Word  | `0x0063`       |
| Command Value | None           |
| Return Value  | 2B status      |

**Send:**
```
FD FC FB FA 02 00 63 00 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 04 00 63 01 00 00 04 03 02 01
```

---

### 4.7 Set Gate Sensitivity

**Configures sensitivity for a specific gate or all gates. Persists across power cycles.**

| Field         | Value                                     |
|---------------|-------------------------------------------|
| Command Word  | `0x0064`                                  |
| Command Value | See parameter structure below             |
| Return Value  | 2B status                                 |

**Parameter Words:**

| Parameter           | Word     |
|---------------------|----------|
| Distance gate       | `0x0000` |
| Motion sensitivity  | `0x0001` |
| Static sensitivity  | `0x0002` |

**Command Value Structure (18 bytes):**
```
[2B param word 0x0000] [4B gate number (0-8 or 0xFFFF for all)]
[2B param word 0x0001] [4B motion sensitivity (0-100)]
[2B param word 0x0002] [4B static sensitivity (0-100)]
```

**Example:** Set gate 3 to motion=40, static=40:
```
FD FC FB FA 14 00 64 00 00 00 03 00 00 00 01 00 28 00 00 00 02 00 28 00 00 00 04 03 02 01
```

**Example:** Set ALL gates to motion=40, static=40:
```
FD FC FB FA 14 00 64 00 00 00 FF FF 00 00 01 00 28 00 00 00 02 00 28 00 00 00 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 04 00 64 01 00 00 04 03 02 01
```

---

### 4.8 Read Firmware Version

| Field         | Value                                                        |
|---------------|--------------------------------------------------------------|
| Command Word  | `0x00A0`                                                     |
| Command Value | None                                                         |
| Return Value  | 2B status + 2B firmware type (`0x0001`) + 2B major + 4B minor|

**Send:**
```
FD FC FB FA 02 00 A0 00 04 03 02 01
```

**ACK Example (V1.07.22091615):**
```
FD FC FB FA 0C 00 A0 01 00 00 00 01 07 01 16 15 09 22 04 03 02 01
```

**Version Parsing:** `V{major_high}.{major_low}.{minor[3]}{minor[2]}{minor[1]}{minor[0]}`

---

### 4.9 Set Baud Rate

**Changes serial baud rate. Takes effect after restart. Persists across power cycles.**

| Field         | Value                        |
|---------------|------------------------------|
| Command Word  | `0x00A1`                     |
| Command Value | 2B baud rate selection index |
| Return Value  | 2B status                    |

**Send (set to 256000):**
```
FD FC FB FA 04 00 A1 00 07 00 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 04 00 A1 01 00 00 04 03 02 01
```

---

### 4.10 Restore Factory Settings

**Resets all settings to factory defaults. Takes effect after restart.**

| Field         | Value          |
|---------------|----------------|
| Command Word  | `0x00A2`       |
| Command Value | None           |
| Return Value  | 2B status      |

**Send:**
```
FD FC FB FA 02 00 A2 00 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 04 00 A2 01 00 00 04 03 02 01
```

---

### 4.11 Restart Module

**Module restarts after sending ACK.**

| Field         | Value          |
|---------------|----------------|
| Command Word  | `0x00A3`       |
| Command Value | None           |
| Return Value  | 2B status      |

**Send:**
```
FD FC FB FA 02 00 A3 00 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 04 00 A3 01 00 00 04 03 02 01
```

---

### 4.12 Bluetooth Enable/Disable

**Requires restart to take effect.**

| Field         | Value                                        |
|---------------|----------------------------------------------|
| Command Word  | `0x00A4`                                     |
| Command Value | `0x0100` = enable, `0x0000` = disable        |
| Return Value  | 2B status                                    |

**Send (Enable):**
```
FD FC FB FA 04 00 A4 00 01 00 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 04 00 A4 01 00 00 04 03 02 01
```

---

### 4.13 Get MAC Address

| Field         | Value                                        |
|---------------|----------------------------------------------|
| Command Word  | `0x00A5`                                     |
| Command Value | `0x0001`                                     |
| Return Value  | 2B status + 1B type (`0x00`) + 6B MAC (big-endian) |

**Send:**
```
FD FC FB FA 04 00 A5 00 01 00 04 03 02 01
```

**ACK Example (MAC: 8F:27:2E:B8:0F:65):**
```
FD FC FB FA 0A 00 A5 01 00 00 00 8F 27 2E B8 0F 65 04 03 02 01
```

---

### 4.14 Bluetooth Authentication

**Authenticates for Bluetooth access. Response sent via Bluetooth only, not serial.**

| Field         | Value                                        |
|---------------|----------------------------------------------|
| Command Word  | `0x00A8`                                     |
| Command Value | 6B password (2-byte pairs, little-endian)    |
| Return Value  | 2B status (Bluetooth response only)          |

**Default Password:** `HiLink`
- "Hi" = `0x4869`, "Li" = `0x4c69`, "nk" = `0x6e6b`

**Send:**
```
FD FC FB FA 08 00 A8 00 48 69 4C 69 6E 6B 04 03 02 01
```

---

### 4.15 Set Bluetooth Password

| Field         | Value                                        |
|---------------|----------------------------------------------|
| Command Word  | `0x00A9`                                     |
| Command Value | 6B new password (little-endian format)       |
| Return Value  | 2B status                                    |

**Send (set to "HiLink"):**
```
FD FC FB FA 08 00 A9 00 48 69 4C 69 6E 6B 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 04 00 A9 01 00 00 04 03 02 01
```

---

### 4.16 Set Distance Resolution

**Sets the distance per gate. Takes effect after restart. Persists across power cycles.**

| Field         | Value                          |
|---------------|--------------------------------|
| Command Word  | `0x00AA`                       |
| Command Value | 2B resolution index            |
| Return Value  | 2B status                      |

**Resolution Options:**

| Index    | Distance per Gate |
|----------|-------------------|
| `0x0000` | 0.75m             |
| `0x0001` | 0.20m             |

**Send (set to 0.20m):**
```
FD FC FB FA 04 00 AA 00 01 00 04 03 02 01
```

**ACK (Success):**
```
FD FC FB FA 04 00 AA 01 00 00 04 03 02 01
```

---

### 4.17 Query Distance Resolution

| Field         | Value                                    |
|---------------|------------------------------------------|
| Command Word  | `0x00AB`                                 |
| Command Value | None                                     |
| Return Value  | 2B status + 2B resolution index          |

**Send:**
```
FD FC FB FA 02 00 AB 00 04 03 02 01
```

**ACK (0.20m resolution):**
```
FD FC FB FA 06 00 AB 01 00 00 01 00 04 03 02 01
```

---

## 5. Data Output Protocol

### 5.1 Report Frame Format

The module continuously outputs detection data in the following format:

```
┌──────────────┬─────────────────────┬────────────────────────────────┬──────────────┐
│ Frame Header │ Intra-frame Length  │         Intra-frame Data       │ End of Frame │
├──────────────┼─────────────────────┼────────────────────────────────┼──────────────┤
│ F4 F3 F2 F1  │    2 bytes (LE)     │    See data structure below    │ F8 F7 F6 F5  │
└──────────────┴─────────────────────┴────────────────────────────────┴──────────────┘
```

**Constants:**
```
REPORT_FRAME_HEADER = [0xF4, 0xF3, 0xF2, 0xF1]
REPORT_FRAME_END    = [0xF8, 0xF7, 0xF6, 0xF5]
```

### 5.2 Intra-frame Data Structure

```
┌───────────┬────────┬─────────────┬────────┬─────────────┐
│ Data Type │  Head  │ Target Data │  Tail  │ Checksum    │
├───────────┼────────┼─────────────┼────────┼─────────────┤
│  1 byte   │  0xAA  │  variable   │  0x55  │    0x00     │
└───────────┴────────┴─────────────┴────────┴─────────────┘
```

**Data Type Values:**

| Value  | Mode                           |
|--------|--------------------------------|
| `0x01` | Engineering mode data          |
| `0x02` | Basic target information data  |

### 5.3 Target State Values

| Value  | Description                    |
|--------|--------------------------------|
| `0x00` | No target                      |
| `0x01` | Moving target                  |
| `0x02` | Stationary target              |
| `0x03` | Moving & stationary target     |

### 5.4 Basic Target Data (Normal Mode)

**Length:** 9 bytes

| Offset | Size | Field                       | Unit |
|--------|------|-----------------------------|------|
| 0      | 1    | Target state                | -    |
| 1      | 2    | Motion target distance      | cm   |
| 3      | 1    | Motion target energy        | 0-100|
| 4      | 2    | Stationary target distance  | cm   |
| 6      | 1    | Stationary target energy    | 0-100|
| 7      | 2    | Detection distance          | cm   |

**Example Frame (Normal Mode):**
```
F4 F3 F2 F1 0D 00 02 AA 02 51 00 00 00 00 3B 00 00 55 00 F8 F7 F6 F5
            │    │  │  │  └─ Target data (9 bytes)
            │    │  │  └─ Head (0xAA)
            │    │  └─ Data type (0x02 = basic)
            │    └─ Length (13 bytes)
            └─ Header
```

**Parsed:**
- Target state: `0x02` (stationary)
- Motion distance: `0x0051` = 81 cm
- Motion energy: `0x00` = 0
- Stationary distance: `0x0000` = 0 cm
- Stationary energy: `0x3B` = 59
- Detection distance: `0x0000` = 0 cm

### 5.5 Engineering Mode Target Data

**Adds per-gate energy values after basic data.**

| Field                              | Size    |
|------------------------------------|---------|
| Basic target data                  | 9 bytes |
| Max motion distance gate (N)       | 1 byte  |
| Max static distance gate (N)       | 1 byte  |
| Motion energy gate 0               | 1 byte  |
| ...                                | ...     |
| Motion energy gate N               | 1 byte  |
| Static energy gate 0               | 1 byte  |
| ...                                | ...     |
| Static energy gate N               | 1 byte  |
| Reserved/additional data           | M bytes |

**Example Frame (Engineering Mode):**
```
F4 F3 F2 F1 23 00 01 AA 03 1E 00 3C 00 00 39 00 00 08 08
3C 22 05 03 03 04 03 06 05 00 00 39 10 13 06 06 08 04 03 05 55 00 F8 F7 F6 F5
```

---

## 6. Implementation Examples

### 6.1 Python Frame Builder

```python
import struct

COMMAND_HEADER = bytes([0xFD, 0xFC, 0xFB, 0xFA])
COMMAND_END = bytes([0x04, 0x03, 0x02, 0x01])

REPORT_HEADER = bytes([0xF4, 0xF3, 0xF2, 0xF1])
REPORT_END = bytes([0xF8, 0xF7, 0xF6, 0xF5])

def build_command(command_word: int, command_value: bytes = b'') -> bytes:
    """Build a command frame."""
    intra_data = struct.pack('<H', command_word) + command_value
    length = struct.pack('<H', len(intra_data))
    return COMMAND_HEADER + length + intra_data + COMMAND_END

def parse_ack(frame: bytes) -> dict:
    """Parse an ACK frame."""
    if not frame.startswith(COMMAND_HEADER) or not frame.endswith(COMMAND_END):
        raise ValueError("Invalid frame format")
    
    length = struct.unpack('<H', frame[4:6])[0]
    intra_data = frame[6:6+length]
    
    ack_command = struct.unpack('<H', intra_data[0:2])[0]
    status = struct.unpack('<H', intra_data[2:4])[0]
    return_data = intra_data[4:]
    
    return {
        'command': ack_command & 0x00FF,  # Original command
        'success': status == 0,
        'data': return_data
    }

# Command builders
def cmd_enable_config() -> bytes:
    return build_command(0x00FF, struct.pack('<H', 0x0001))

def cmd_end_config() -> bytes:
    return build_command(0x00FE)

def cmd_read_params() -> bytes:
    return build_command(0x0061)

def cmd_read_firmware() -> bytes:
    return build_command(0x00A0)

def cmd_restart() -> bytes:
    return build_command(0x00A3)

def cmd_factory_reset() -> bytes:
    return build_command(0x00A2)

def cmd_enable_engineering() -> bytes:
    return build_command(0x0062)

def cmd_disable_engineering() -> bytes:
    return build_command(0x0063)

def cmd_set_max_gate_and_timeout(max_motion_gate: int, max_static_gate: int, timeout_sec: int) -> bytes:
    """Set maximum detection gates and no-presence timeout."""
    value = struct.pack('<HIHI HI',
        0x0000, max_motion_gate,    # Max motion gate
        0x0001, max_static_gate,    # Max static gate
        0x0002, timeout_sec         # Timeout
    )
    return build_command(0x0060, value)

def cmd_set_gate_sensitivity(gate: int, motion_sens: int, static_sens: int) -> bytes:
    """Set sensitivity for a gate (0-8) or all gates (0xFFFF)."""
    value = struct.pack('<HIHIHI',
        0x0000, gate,           # Gate number
        0x0001, motion_sens,    # Motion sensitivity
        0x0002, static_sens     # Static sensitivity
    )
    return build_command(0x0064, value)

def cmd_set_baud_rate(index: int) -> bytes:
    """Set baud rate by index (1=9600, 2=19200, ..., 7=256000, 8=460800)."""
    return build_command(0x00A1, struct.pack('<H', index))

def cmd_set_distance_resolution(resolution: int) -> bytes:
    """Set distance resolution (0=0.75m, 1=0.20m per gate)."""
    return build_command(0x00AA, struct.pack('<H', resolution))

def cmd_query_distance_resolution() -> bytes:
    return build_command(0x00AB)

def cmd_bluetooth_enable(enable: bool) -> bytes:
    value = 0x0100 if enable else 0x0000
    return build_command(0x00A4, struct.pack('<H', value))

def cmd_get_mac() -> bytes:
    return build_command(0x00A5, struct.pack('<H', 0x0001))
```

### 6.2 Python Report Parser

```python
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, List

class TargetState(IntEnum):
    NO_TARGET = 0x00
    MOVING = 0x01
    STATIONARY = 0x02
    MOVING_AND_STATIONARY = 0x03

@dataclass
class TargetData:
    state: TargetState
    motion_distance_cm: int
    motion_energy: int
    stationary_distance_cm: int
    stationary_energy: int
    detection_distance_cm: int
    
    # Engineering mode only
    max_motion_gate: Optional[int] = None
    max_static_gate: Optional[int] = None
    motion_gate_energy: Optional[List[int]] = None
    static_gate_energy: Optional[List[int]] = None

def parse_report_frame(frame: bytes) -> Optional[TargetData]:
    """Parse a report frame from the radar."""
    if not frame.startswith(REPORT_HEADER) or not frame.endswith(REPORT_END):
        return None
    
    length = struct.unpack('<H', frame[4:6])[0]
    intra_data = frame[6:6+length]
    
    if len(intra_data) < 3:
        return None
    
    data_type = intra_data[0]
    head = intra_data[1]
    
    if head != 0xAA:
        return None
    
    target_data = intra_data[2:-2]  # Exclude tail (0x55) and checksum (0x00)
    
    if len(target_data) < 9:
        return None
    
    result = TargetData(
        state=TargetState(target_data[0]),
        motion_distance_cm=struct.unpack('<H', target_data[1:3])[0],
        motion_energy=target_data[3],
        stationary_distance_cm=struct.unpack('<H', target_data[4:6])[0],
        stationary_energy=target_data[6],
        detection_distance_cm=struct.unpack('<H', target_data[7:9])[0]
    )
    
    # Engineering mode has additional data
    if data_type == 0x01 and len(target_data) > 9:
        result.max_motion_gate = target_data[9]
        result.max_static_gate = target_data[10]
        
        n = result.max_motion_gate + 1
        offset = 11
        
        if len(target_data) >= offset + 2*n:
            result.motion_gate_energy = list(target_data[offset:offset+n])
            result.static_gate_energy = list(target_data[offset+n:offset+2*n])
    
    return result
```

### 6.3 Frame Receiver State Machine

```python
from enum import Enum, auto

class FrameType(Enum):
    COMMAND_ACK = auto()
    REPORT = auto()

class FrameState(Enum):
    WAIT_HEADER = auto()
    READ_LENGTH = auto()
    READ_DATA = auto()
    WAIT_END = auto()

class FrameReceiver:
    def __init__(self):
        self.state = FrameState.WAIT_HEADER
        self.buffer = bytearray()
        self.frame_type = None
        self.expected_length = 0
        self.header_index = 0
        
    def process_byte(self, byte: int) -> Optional[tuple[FrameType, bytes]]:
        """Process a single byte, returns complete frame when available."""
        
        if self.state == FrameState.WAIT_HEADER:
            # Check for command ACK header
            if byte == COMMAND_HEADER[self.header_index]:
                self.buffer.append(byte)
                self.header_index += 1
                if self.header_index == 4:
                    self.frame_type = FrameType.COMMAND_ACK
                    self.state = FrameState.READ_LENGTH
                    self.header_index = 0
            # Check for report header
            elif byte == REPORT_HEADER[self.header_index]:
                self.buffer.append(byte)
                self.header_index += 1
                if self.header_index == 4:
                    self.frame_type = FrameType.REPORT
                    self.state = FrameState.READ_LENGTH
                    self.header_index = 0
            else:
                self.buffer.clear()
                self.header_index = 0
                
        elif self.state == FrameState.READ_LENGTH:
            self.buffer.append(byte)
            if len(self.buffer) == 6:
                self.expected_length = struct.unpack('<H', self.buffer[4:6])[0]
                self.state = FrameState.READ_DATA
                
        elif self.state == FrameState.READ_DATA:
            self.buffer.append(byte)
            if len(self.buffer) == 6 + self.expected_length:
                self.state = FrameState.WAIT_END
                self.header_index = 0
                
        elif self.state == FrameState.WAIT_END:
            expected_end = COMMAND_END if self.frame_type == FrameType.COMMAND_ACK else REPORT_END
            if byte == expected_end[self.header_index]:
                self.buffer.append(byte)
                self.header_index += 1
                if self.header_index == 4:
                    # Complete frame
                    result = (self.frame_type, bytes(self.buffer))
                    self.reset()
                    return result
            else:
                self.reset()
                
        return None
    
    def reset(self):
        self.state = FrameState.WAIT_HEADER
        self.buffer.clear()
        self.frame_type = None
        self.expected_length = 0
        self.header_index = 0
```

---

## 7. Configuration Defaults

### 7.1 Factory Default Values

| Parameter                    | Default Value |
|------------------------------|---------------|
| Maximum motion distance gate | 8             |
| Maximum static distance gate | 8             |
| No-presence timeout          | 5 seconds     |
| Serial baud rate             | 256000        |
| Distance resolution          | 0.75m         |
| Bluetooth                    | Enabled       |
| Bluetooth password           | "HiLink"      |
| Engineering mode             | Disabled      |

### 7.2 Default Gate Sensitivities

| Gate | Motion Sensitivity | Static Sensitivity |
|------|--------------------|--------------------|
| 0    | 50                 | *(not settable)*   |
| 1    | 50                 | *(not settable)*   |
| 2    | 40                 | 40                 |
| 3    | 30                 | 40                 |
| 4    | 20                 | 30                 |
| 5    | 15                 | 30                 |
| 6    | 15                 | 20                 |
| 7    | 15                 | 20                 |
| 8    | 15                 | 20                 |

### 7.3 Distance Gate Mapping

With default 0.75m resolution:

| Gate | Distance Range    |
|------|-------------------|
| 0    | 0.00m - 0.75m     |
| 1    | 0.75m - 1.50m     |
| 2    | 1.50m - 2.25m     |
| 3    | 2.25m - 3.00m     |
| 4    | 3.00m - 3.75m     |
| 5    | 3.75m - 4.50m     |
| 6    | 4.50m - 5.25m     |
| 7    | 5.25m - 6.00m     |
| 8    | 6.00m - 6.75m     |

With 0.20m resolution:

| Gate | Distance Range    |
|------|-------------------|
| 0    | 0.00m - 0.20m     |
| 1    | 0.20m - 0.40m     |
| ...  | ...               |
| 8    | 1.60m - 1.80m     |

---

## Quick Reference: Command Words

| Command                     | Word     | Requires Config Mode |
|-----------------------------|----------|----------------------|
| Enable Configuration        | `0x00FF` | No (starts it)       |
| End Configuration           | `0x00FE` | Yes (ends it)        |
| Set Max Gate & Timeout      | `0x0060` | Yes                  |
| Read Parameters             | `0x0061` | Yes                  |
| Enable Engineering Mode     | `0x0062` | Yes                  |
| Disable Engineering Mode    | `0x0063` | Yes                  |
| Set Gate Sensitivity        | `0x0064` | Yes                  |
| Read Firmware Version       | `0x00A0` | Yes                  |
| Set Baud Rate               | `0x00A1` | Yes                  |
| Factory Reset               | `0x00A2` | Yes                  |
| Restart Module              | `0x00A3` | Yes                  |
| Bluetooth Enable/Disable    | `0x00A4` | Yes                  |
| Get MAC Address             | `0x00A5` | Yes                  |
| Bluetooth Auth              | `0x00A8` | Yes                  |
| Set Bluetooth Password      | `0x00A9` | Yes                  |
| Set Distance Resolution     | `0x00AA` | Yes                  |
| Query Distance Resolution   | `0x00AB` | Yes                  |

---

## Notes for Implementation

1. **Always use little-endian byte order** for multi-byte values
2. **Configuration mode is required** for all commands except normal data reception
3. **Wait for ACK** before sending the next command
4. **Engineering mode is volatile** - lost on power cycle
5. **Sensitivity of 100** effectively disables detection for that gate
6. **Gates 0-1 static sensitivity** cannot be configured
7. **Distance resolution changes require restart** to take effect
8. **Baud rate changes require restart** to take effect

---

*Document generated from HLK-LD2410C Serial Communication Protocol V1.00*
