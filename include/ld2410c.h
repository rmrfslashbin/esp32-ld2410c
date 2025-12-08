/**
 * @file ld2410c.h
 * @brief HLK-LD2410C Human Presence Sensor Driver for ESP-IDF
 *
 * Complete C implementation based on:
 * - Official LD2410C Serial Communication Protocol V1.00
 * - ESPHome ld2410 component
 *
 * Default UART: 256000 baud, 8N1
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Protocol Constants - Frame Headers and Footers
#define LD2410C_CMD_FRAME_HEADER_0    0xFD
#define LD2410C_CMD_FRAME_HEADER_1    0xFC
#define LD2410C_CMD_FRAME_HEADER_2    0xFB
#define LD2410C_CMD_FRAME_HEADER_3    0xFA
#define LD2410C_CMD_FRAME_FOOTER_0    0x04
#define LD2410C_CMD_FRAME_FOOTER_1    0x03
#define LD2410C_CMD_FRAME_FOOTER_2    0x02
#define LD2410C_CMD_FRAME_FOOTER_3    0x01

#define LD2410C_DATA_FRAME_HEADER_0   0xF4
#define LD2410C_DATA_FRAME_HEADER_1   0xF3
#define LD2410C_DATA_FRAME_HEADER_2   0xF2
#define LD2410C_DATA_FRAME_HEADER_3   0xF1
#define LD2410C_DATA_FRAME_FOOTER_0   0xF8
#define LD2410C_DATA_FRAME_FOOTER_1   0xF7
#define LD2410C_DATA_FRAME_FOOTER_2   0xF6
#define LD2410C_DATA_FRAME_FOOTER_3   0xF5

#define LD2410C_HEADER_FOOTER_SIZE    4

// Protocol Constants - Data Frame
#define LD2410C_DATA_FRAME_HEAD       0xAA
#define LD2410C_DATA_FRAME_TAIL       0x55
#define LD2410C_DATA_FRAME_CHECK      0x00

// Command Words (from protocol spec section 2.2)
#define LD2410C_CMD_ENABLE_CONF       0xFF  // Enable configuration mode
#define LD2410C_CMD_DISABLE_CONF      0xFE  // Disable configuration mode (exit config, start streaming)
#define LD2410C_CMD_MAXDIST_DURATION  0x60  // Set max distance and duration
#define LD2410C_CMD_QUERY             0x61  // Read parameters
#define LD2410C_CMD_ENABLE_ENG        0x62  // Enable engineering mode
#define LD2410C_CMD_DISABLE_ENG       0x63  // Disable engineering mode
#define LD2410C_CMD_GATE_SENS         0x64  // Set gate sensitivity
#define LD2410C_CMD_QUERY_VERSION     0xA0  // Read firmware version
#define LD2410C_CMD_SET_BAUD_RATE     0xA1  // Set baud rate
#define LD2410C_CMD_RESET             0xA2  // Factory reset
#define LD2410C_CMD_RESTART           0xA3  // Restart module
#define LD2410C_CMD_BLUETOOTH         0xA4  // Bluetooth on/off
#define LD2410C_CMD_QUERY_MAC         0xA5  // Get MAC address
#define LD2410C_CMD_BT_PASSWORD       0xA9  // Set Bluetooth password
#define LD2410C_CMD_SET_DIST_RES      0xAA  // Set distance resolution
#define LD2410C_CMD_QUERY_DIST_RES    0xAB  // Query distance resolution

// Data Types (from protocol spec Table 11)
#define LD2410C_DATA_TYPE_ENGINEERING 0x01
#define LD2410C_DATA_TYPE_BASIC       0x02

// Target States (from protocol spec Table 13)
#define LD2410C_TARGET_NONE           0x00
#define LD2410C_TARGET_MOVING         0x01
#define LD2410C_TARGET_STATIONARY     0x02
#define LD2410C_TARGET_BOTH           0x03

// Configuration Constants (can be overridden via Kconfig)
#define LD2410C_MAX_GATES             9     // Gates 0-8

#ifndef CONFIG_LD2410C_MAX_FRAME_SIZE
#define LD2410C_MAX_FRAME_SIZE        64
#else
#define LD2410C_MAX_FRAME_SIZE        CONFIG_LD2410C_MAX_FRAME_SIZE
#endif

#ifndef CONFIG_LD2410C_UART_BUF_SIZE
#define LD2410C_UART_BUF_SIZE         1024
#else
#define LD2410C_UART_BUF_SIZE         CONFIG_LD2410C_UART_BUF_SIZE
#endif

#ifndef CONFIG_LD2410C_ACK_TIMEOUT_MS
#define LD2410C_ACK_TIMEOUT_MS        500
#else
#define LD2410C_ACK_TIMEOUT_MS        CONFIG_LD2410C_ACK_TIMEOUT_MS
#endif

// Default Configuration (from protocol spec Table 7)
#ifndef CONFIG_LD2410C_DEFAULT_BAUD_RATE
#define LD2410C_DEFAULT_BAUD          256000
#else
#define LD2410C_DEFAULT_BAUD          CONFIG_LD2410C_DEFAULT_BAUD_RATE
#endif

#define LD2410C_DEFAULT_MAX_GATE      8
#define LD2410C_DEFAULT_TIMEOUT_SEC   5

// Protocol parameter word identifiers (used in command values)
#define LD2410C_PARAM_WORD_MAX_MOVE_DIST   0x0000
#define LD2410C_PARAM_WORD_MAX_STILL_DIST  0x0001
#define LD2410C_PARAM_WORD_TIMEOUT         0x0002
#define LD2410C_PARAM_WORD_GATE_NUM        0x0000
#define LD2410C_PARAM_WORD_MOVE_SENS       0x0001
#define LD2410C_PARAM_WORD_STILL_SENS      0x0002

/**
 * @brief Sensor data structure
 */
typedef struct {
    // Target state
    bool presence_detected;      // Any target detected
    bool moving_target;          // Moving target detected
    bool stationary_target;      // Stationary target detected

    // Distance measurements (in cm)
    uint16_t moving_distance;    // Distance to moving target
    uint16_t stationary_distance;// Distance to stationary target
    uint16_t detection_distance; // Detection distance

    // Energy values (0-100)
    uint8_t moving_energy;       // Moving target energy
    uint8_t stationary_energy;   // Stationary target energy

    // Engineering mode data (9 gates)
    uint8_t gate_move_energy[LD2410C_MAX_GATES];
    uint8_t gate_still_energy[LD2410C_MAX_GATES];

    // Timing
    int64_t last_update_time;    // Timestamp of last update (esp_timer_get_time())
} ld2410c_data_t;

/**
 * @brief Configuration structure
 */
typedef struct {
    uart_port_t uart_num;        // UART port number
    uint8_t tx_pin;              // TX GPIO pin
    uint8_t rx_pin;              // RX GPIO pin
    uint32_t baud_rate;          // Baud rate (default 256000)

    uint8_t max_move_gate;       // Max detection gate for moving (2-8)
    uint8_t max_still_gate;      // Max detection gate for stationary (2-8)
    uint16_t timeout_seconds;    // Unoccupied duration (0-65535)

    bool engineering_mode;       // Enable engineering mode
} ld2410c_config_t;

/**
 * @brief Initialize LD2410C sensor
 *
 * @param config Configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2410c_init(const ld2410c_config_t *config);

/**
 * @brief Deinitialize LD2410C sensor
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2410c_deinit(void);

/**
 * @brief Get current sensor data
 *
 * @param data Pointer to data structure to fill
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2410c_get_data(ld2410c_data_t *data);

/**
 * @brief Check if sensor is connected (received data recently)
 *
 * @param timeout_sec Timeout in seconds (typically 5)
 * @return true if sensor is connected
 */
bool ld2410c_is_connected(uint8_t timeout_sec);

/**
 * @brief Enable engineering mode
 *
 * @param enable true to enable, false to disable
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2410c_set_engineering_mode(bool enable);

/**
 * @brief Set max detection gates and timeout
 *
 * @param max_move_gate Max moving detection gate (2-8)
 * @param max_still_gate Max stationary detection gate (2-8)
 * @param timeout_sec Unoccupied duration in seconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2410c_set_max_distances_timeout(uint8_t max_move_gate,
                                              uint8_t max_still_gate,
                                              uint16_t timeout_sec);

/**
 * @brief Set sensitivity for a specific gate
 *
 * @param gate Gate number (0-8)
 * @param move_sensitivity Moving sensitivity (0-100)
 * @param still_sensitivity Stationary sensitivity (0-100)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2410c_set_gate_sensitivity(uint8_t gate,
                                        uint8_t move_sensitivity,
                                        uint8_t still_sensitivity);

/**
 * @brief Read firmware version
 *
 * @param version Buffer to store version string (min 20 bytes)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2410c_get_version(char *version, size_t version_size);

/**
 * @brief Restart the sensor module
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2410c_restart(void);

/**
 * @brief Factory reset the sensor
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2410c_factory_reset(void);

#ifdef __cplusplus
}
#endif
