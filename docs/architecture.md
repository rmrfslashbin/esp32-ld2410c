# LD2410C Driver Architecture

This document describes the internal architecture and design decisions of the LD2410C ESP-IDF driver.

## Overview

The driver implements a non-blocking, event-driven architecture using FreeRTOS tasks and ESP-IDF UART driver.

```
┌─────────────────────────────────────────────────────────────┐
│                      User Application                        │
└────────────────────────┬────────────────────────────────────┘
                         │ Public API
                         │ (ld2410c_init, ld2410c_get_data, etc.)
                         ▼
┌─────────────────────────────────────────────────────────────┐
│                    LD2410C Driver                            │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   Data API   │  │  Config API  │  │  Status API  │      │
│  │              │  │              │  │              │      │
│  │ get_data()   │  │ set_gates()  │  │ is_connected │      │
│  │              │  │ set_sens()   │  │              │      │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘      │
│         │                  │                  │              │
│         ▼                  ▼                  ▼              │
│  ┌──────────────────────────────────────────────────┐      │
│  │          Shared State (s_ld2410c)                │      │
│  │  - UART config                                   │      │
│  │  - Sensor data (protected by mutex)              │      │
│  │  - Frame parsing state                           │      │
│  └──────────────────┬───────────────────────────────┘      │
│                     │                                       │
│  ┌─────────────────▼────────────────────────────┐          │
│  │       UART Receive Task                      │          │
│  │  - Reads bytes from UART                     │          │
│  │  - State machine frame parser                │          │
│  │  - Updates shared data (with mutex)          │          │
│  └─────────────────┬────────────────────────────┘          │
└────────────────────┼────────────────────────────────────────┘
                     │ ESP-IDF UART Driver API
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                    ESP-IDF UART Driver                       │
│  - Hardware UART peripheral                                  │
│  - RX ring buffer                                            │
│  - Interrupt handling                                        │
└─────────────────────┬───────────────────────────────────────┘
                      │ Hardware
                      ▼
                ┌──────────────┐
                │  LD2410C     │
                │  Sensor      │
                └──────────────┘
```

## Components

### 1. Public API Layer

**File:** `ld2410c.c` (public functions)

Provides the user-facing API with:
- Initialization and cleanup (`ld2410c_init`, `ld2410c_deinit`)
- Data access (`ld2410c_get_data`, `ld2410c_is_connected`)
- Configuration (`ld2410c_set_engineering_mode`, `ld2410c_set_gate_sensitivity`)

**Key Responsibilities:**
- Input validation
- State management (initialized/not initialized)
- Error handling and logging
- Thread-safe access to shared data

### 2. Shared State

**Structure:** `s_ld2410c` (static global)

```c
static struct {
    uart_port_t uart_num;           // UART port being used
    ld2410c_data_t data;            // Latest sensor data
    SemaphoreHandle_t data_mutex;   // Protects data access
    TaskHandle_t uart_task_handle;  // UART receive task
    bool initialized;               // Driver state

    // Frame parsing state machine
    uint8_t buffer[64];             // Frame assembly buffer
    uint8_t buffer_pos;             // Current position in buffer
    bool in_frame;                  // Currently parsing a frame
    enum frame_type;                // Type of frame being parsed
} s_ld2410c;
```

**Thread Safety:**
- `data_mutex` protects the `data` structure
- All public API functions that read data acquire this mutex
- UART task acquires mutex when updating data

### 3. UART Receive Task

**Function:** `uart_rx_task()` (FreeRTOS task)

**Operation:**
1. Continuously reads bytes from UART driver
2. Feeds bytes to frame parser state machine
3. On complete frame: validates and processes
4. Updates shared data structure (with mutex)
5. Never blocks indefinitely (100ms timeout on UART reads)

**Priority:** Configurable via Kconfig (default: 5)
**Stack Size:** Configurable via Kconfig (default: 4096 bytes)

### 4. Frame Parser State Machine

**Functions:** `process_byte()`, `parse_data_frame()`, `parse_ack_frame()`

**States:**
- `FRAME_TYPE_UNKNOWN` - Searching for frame header
- `FRAME_TYPE_DATA` - Parsing sensor data frame
- `FRAME_TYPE_CMD_ACK` - Parsing command acknowledgment

**Process:**
```
Byte received
    │
    ▼
Not in frame? ──Yes──► Look for header bytes (0xF4 0xF3 0xF2 0xF1)
    │                   Found all 4? Set in_frame = true
    No                  Determine frame type
    │
    ▼
In frame? ──Yes──► Collect bytes into buffer
    │               Read length field (bytes 4-5)
    │               Wait for complete frame
    │               Validate footer
    │               Parse frame contents
    │               Update shared data
    │               Reset state
    │
    ▼
  Continue
```

### 5. Command/Config Layer

**Functions:** `send_command()`, `set_config_mode()`

**Pattern:**
```c
1. Enter config mode (stop data streaming)
2. Send command
3. Wait for ACK (currently: delay-based)
4. Exit config mode (resume data streaming)
```

**Note:** ACK parsing is implemented but not currently blocking on ACK receipt.

## Data Flow

### Reading Sensor Data

```
Application calls ld2410c_get_data()
    │
    ▼
Validate: initialized? data pointer valid?
    │
    ▼
Acquire data_mutex
    │
    ▼
memcpy shared data to user buffer
    │
    ▼
Release data_mutex
    │
    ▼
Return ESP_OK
```

### Receiving Data from Sensor

```
Sensor transmits UART bytes
    │
    ▼
ESP-IDF UART driver stores in RX buffer
    │
    ▼
UART task reads bytes (uart_read_bytes)
    │
    ▼
For each byte: process_byte()
    │
    ▼
Frame complete?
    │
    ▼
parse_data_frame() or parse_ack_frame()
    │
    ▼
Acquire data_mutex
    │
    ▼
Update s_ld2410c.data
    │
    ▼
Release data_mutex
```

## Design Decisions

### Why FreeRTOS Task Instead of Callbacks?

**Decision:** Use dedicated task for UART reception

**Rationale:**
- Simpler error handling (task can restart if needed)
- Clearer stack usage (dedicated task stack)
- Easier debugging (task-specific context)
- No interrupt context limitations

**Trade-off:** ~4KB RAM for task stack

### Why Mutex Instead of Queue?

**Decision:** Single data structure with mutex protection

**Rationale:**
- User only cares about latest data (not historical)
- Simpler API (no queue management)
- Lower latency (direct memory access)
- Less RAM overhead

**Trade-off:** Can't retrieve historical readings

### Why State Machine Instead of Pattern Matching?

**Decision:** Byte-by-byte state machine for frame parsing

**Rationale:**
- Handles partial frames gracefully
- Minimal buffering needed
- Works with any UART buffer size
- Resilient to corrupted data

**Trade-off:** More complex parsing logic

### Why Static Global Instead of Handle?

**Decision:** Single static instance (`s_ld2410c`)

**Rationale:**
- Typical use: one sensor per device
- Simpler API (no handle passing)
- Compile-time memory allocation
- Matches ESP-IDF component patterns

**Trade-off:** Can't have multiple sensors

## Memory Usage

**Static Allocation:**
- Driver state: ~200 bytes
- Frame buffer: 64 bytes
- UART driver buffers: 2048 bytes (configurable)
- Total: ~2.3 KB

**Dynamic Allocation:**
- UART receive task stack: 4096 bytes (configurable)
- FreeRTOS mutex: ~80 bytes
- Total: ~4.2 KB

**Peak Total:** ~6.5 KB

## Performance Characteristics

**Latency:**
- Data update rate: Limited by sensor (typically ~50ms)
- API read latency: <1ms (mutex + memcpy)
- Command latency: ~200-500ms (mode switch delays)

**CPU Usage:**
- UART task: <1% (mostly blocked on UART reads)
- Frame parsing: ~0.5% (byte processing)

**Reliability:**
- Frame validation: Header, footer, length checks
- Overflow protection: All buffer operations bounds-checked
- Error recovery: Invalid frames discarded, state machine resets

## Thread Safety

**Protected Resources:**
- `s_ld2410c.data` - Protected by `data_mutex`
- `s_ld2410c.initialized` - Checked before mutex operations

**Thread-Safe Functions:**
- `ld2410c_get_data()` - Acquires mutex
- `ld2410c_is_connected()` - Acquires mutex

**Not Thread-Safe (By Design):**
- Configuration functions - Should be called during setup
- Init/deinit - Should be called once

## Future Improvements

**Planned:**
- [ ] ACK-based command verification (instead of delays)
- [ ] Retry logic for failed commands
- [ ] Command queue for multiple config changes
- [ ] Statistics (frames received, CRC errors, etc.)
- [ ] Power management (sleep/wake)

**Under Consideration:**
- [ ] Multi-instance support (multiple sensors)
- [ ] DMA-based UART transfers
- [ ] Circular buffer for historical data
- [ ] Event callbacks for presence changes

## Testing Considerations

**Unit Tests:**
- Input validation (all API functions)
- State management (init/deinit sequences)
- Error conditions (NULL pointers, invalid ranges)

**Integration Tests:**
- Frame parsing (with simulated UART data)
- Command sequences
- Thread safety (concurrent access)

**Hardware Tests:**
- Actual sensor communication
- Long-duration stability
- Power cycling
- Error recovery

## References

- [ESP-IDF UART Driver](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/uart.html)
- [FreeRTOS Task Management](https://www.freertos.org/taskandcr.html)
- [LD2410C Protocol Specification](../datasheets/LD2410C_Protocol.md)
