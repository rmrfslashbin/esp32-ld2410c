/**
 * @file ld2410c.c
 * @brief HLK-LD2410C Human Presence Sensor Driver Implementation
 *
 * Based on:
 * - Official LD2410C Serial Communication Protocol V1.00
 * - ESPHome ld2410 component
 *
 * Protocol summary:
 * 1. Power on → Sensor boots in config/Bluetooth mode
 * 2. Send ENABLE_CONF (0xFF) → Enter config mode
 * 3. Send configuration commands
 * 4. Send DISABLE_CONF (0xFE) → Exit config mode, start streaming data
 * 5. Sensor continuously streams data frames with detection results
 */

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "ld2410c.h"

static const char *TAG = "ld2410c";

// Driver state (static per ESP-IDF conventions)
static struct {
    uart_port_t uart_num;
    ld2410c_data_t data;
    SemaphoreHandle_t data_mutex;
    TaskHandle_t uart_task_handle;
    bool initialized;

    // Frame parsing state
    uint8_t buffer[LD2410C_MAX_FRAME_SIZE];
    uint8_t buffer_pos;
    bool in_frame;
    enum {
        FRAME_TYPE_UNKNOWN,
        FRAME_TYPE_DATA,
        FRAME_TYPE_CMD_ACK
    } frame_type;
} s_ld2410c = {0};

// Helper functions
static inline uint16_t two_byte_to_int(uint8_t low, uint8_t high) {
    return (uint16_t)(high << 8) | low;
}

static bool validate_header(const uint8_t *buf, bool is_data_frame) {
    if (is_data_frame) {
        return buf[0] == LD2410C_DATA_FRAME_HEADER_0 &&
               buf[1] == LD2410C_DATA_FRAME_HEADER_1 &&
               buf[2] == LD2410C_DATA_FRAME_HEADER_2 &&
               buf[3] == LD2410C_DATA_FRAME_HEADER_3;
    } else {
        return buf[0] == LD2410C_CMD_FRAME_HEADER_0 &&
               buf[1] == LD2410C_CMD_FRAME_HEADER_1 &&
               buf[2] == LD2410C_CMD_FRAME_HEADER_2 &&
               buf[3] == LD2410C_CMD_FRAME_HEADER_3;
    }
}

static bool validate_footer(const uint8_t *buf, bool is_data_frame) {
    if (is_data_frame) {
        return buf[0] == LD2410C_DATA_FRAME_FOOTER_0 &&
               buf[1] == LD2410C_DATA_FRAME_FOOTER_1 &&
               buf[2] == LD2410C_DATA_FRAME_FOOTER_2 &&
               buf[3] == LD2410C_DATA_FRAME_FOOTER_3;
    } else {
        return buf[0] == LD2410C_CMD_FRAME_FOOTER_0 &&
               buf[1] == LD2410C_CMD_FRAME_FOOTER_1 &&
               buf[2] == LD2410C_CMD_FRAME_FOOTER_2 &&
               buf[3] == LD2410C_CMD_FRAME_FOOTER_3;
    }
}

/**
 * @brief Send command to sensor
 *
 * Frame format:
 * [Header 4B] [Length 2B] [Command 2B] [Value NB] [Footer 4B]
 */
static esp_err_t send_command(uint8_t cmd, const uint8_t *value, uint8_t value_len) {
    uint8_t frame[64];
    uint8_t pos = 0;

    // Validate total frame size to prevent buffer overflow
    // Total: 4 (header) + 2 (length) + 2 (command) + value_len + 4 (footer)
    uint16_t total_len = 12 + value_len;
    if (total_len > sizeof(frame)) {
        ESP_LOGE(TAG, "Command frame too large: %d bytes (max %zu)", total_len, sizeof(frame));
        return ESP_ERR_INVALID_SIZE;
    }

    // Header
    frame[pos++] = LD2410C_CMD_FRAME_HEADER_0;
    frame[pos++] = LD2410C_CMD_FRAME_HEADER_1;
    frame[pos++] = LD2410C_CMD_FRAME_HEADER_2;
    frame[pos++] = LD2410C_CMD_FRAME_HEADER_3;

    // Length (2 bytes for command + value length)
    // Note: value_len is uint8_t (max 255), so total_len can't overflow uint16_t
    uint16_t len = 2U + (uint16_t)value_len;
    frame[pos++] = (uint8_t)(len & 0xFFU);        // Low byte
    frame[pos++] = (uint8_t)((len >> 8) & 0xFFU); // High byte

    // Command word (little endian)
    frame[pos++] = cmd;
    frame[pos++] = 0x00;

    // Command value
    if (value != NULL && value_len > 0) {
        memcpy(&frame[pos], value, value_len);
        pos += value_len;
    }

    // Footer
    frame[pos++] = LD2410C_CMD_FRAME_FOOTER_0;
    frame[pos++] = LD2410C_CMD_FRAME_FOOTER_1;
    frame[pos++] = LD2410C_CMD_FRAME_FOOTER_2;
    frame[pos++] = LD2410C_CMD_FRAME_FOOTER_3;

    // Send
    int written = uart_write_bytes(s_ld2410c.uart_num, frame, pos);
    if (written != pos) {
        ESP_LOGW(TAG, "Failed to send command 0x%02X (wrote %d/%d bytes)", cmd, written, pos);
        return ESP_FAIL;
    }

    // Wait for TX to complete
    uart_wait_tx_done(s_ld2410c.uart_num, pdMS_TO_TICKS(100));

    // Delay after command (except for config mode commands)
    if (cmd != LD2410C_CMD_ENABLE_CONF && cmd != LD2410C_CMD_DISABLE_CONF) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGD(TAG, "Sent command 0x%02X (%d bytes)", cmd, pos);
    return ESP_OK;
}

/**
 * @brief Enable or disable configuration mode
 */
static esp_err_t set_config_mode(bool enable) {
    uint8_t cmd = enable ? LD2410C_CMD_ENABLE_CONF : LD2410C_CMD_DISABLE_CONF;
    uint8_t value[] = {0x01, 0x00};  // Only for ENABLE_CONF

    if (enable) {
        ESP_LOGI(TAG, "Enabling configuration mode...");
        return send_command(cmd, value, sizeof(value));
    } else {
        ESP_LOGI(TAG, "Disabling configuration mode (starting data stream)...");
        return send_command(cmd, NULL, 0);
    }
}

/**
 * @brief Parse data frame (sensor readings)
 *
 * Data frame format (from protocol spec 2.3):
 * [Header 4B] [Length 2B] [Type 1B] [Head 0xAA] [Data] [Tail 0x55] [Check 0x00] [Footer 4B]
 */
static void parse_data_frame(const uint8_t *buf, uint8_t len) {
    // Minimum length: 4 header + 2 length + 1 type + 1 head + data + 1 tail + 1 check + 4 footer
    if (len < 13) {
        ESP_LOGW(TAG, "Data frame too short: %d bytes", len);
        return;
    }

    // Validate header and footer
    if (!validate_header(buf, true)) {
        ESP_LOGW(TAG, "Invalid data frame header");
        return;
    }

    if (!validate_footer(&buf[len - 4], true)) {
        ESP_LOGW(TAG, "Invalid data frame footer");
        return;
    }

    // Check data head/tail/check bytes
    if (buf[7] != LD2410C_DATA_FRAME_HEAD ||
        buf[len - 6] != LD2410C_DATA_FRAME_TAIL ||
        buf[len - 5] != LD2410C_DATA_FRAME_CHECK) {
        ESP_LOGW(TAG, "Invalid data frame markers");
        return;
    }

    uint8_t data_type = buf[6];
    bool engineering_mode = (data_type == LD2410C_DATA_TYPE_ENGINEERING);

    // Parse basic target information (Table 12 from protocol spec)
    // Offset from start of buffer:
    // 8:  target_state
    // 9-10: moving_distance (little endian)
    // 11: moving_energy
    // 12-13: stationary_distance (little endian)
    // 14: stationary_energy
    // 15-16: detection_distance (little endian)

    xSemaphoreTake(s_ld2410c.data_mutex, portMAX_DELAY);

    uint8_t target_state = buf[8];
    s_ld2410c.data.presence_detected = (target_state != LD2410C_TARGET_NONE);
    s_ld2410c.data.moving_target = (target_state & LD2410C_TARGET_MOVING) != 0;
    s_ld2410c.data.stationary_target = (target_state & LD2410C_TARGET_STATIONARY) != 0;

    s_ld2410c.data.moving_distance = two_byte_to_int(buf[9], buf[10]);
    s_ld2410c.data.moving_energy = buf[11];
    s_ld2410c.data.stationary_distance = two_byte_to_int(buf[12], buf[13]);
    s_ld2410c.data.stationary_energy = buf[14];
    s_ld2410c.data.detection_distance = two_byte_to_int(buf[15], buf[16]);

    // Engineering mode: parse gate energies (Table 14 from protocol spec)
    if (engineering_mode && len >= 39) {
        // Offset 20-28: moving gate 0-8 energy (9 bytes)
        // Offset 29-37: stationary gate 0-8 energy (9 bytes)
        for (int i = 0; i < LD2410C_MAX_GATES; i++) {
            s_ld2410c.data.gate_move_energy[i] = buf[20 + i];
            s_ld2410c.data.gate_still_energy[i] = buf[29 + i];
        }
    }

    s_ld2410c.data.last_update_time = esp_timer_get_time();

    xSemaphoreGive(s_ld2410c.data_mutex);

    ESP_LOGD(TAG, "Data: presence=%d, moving=%dcm/%d, still=%dcm/%d, detect=%dcm",
             s_ld2410c.data.presence_detected,
             s_ld2410c.data.moving_distance, s_ld2410c.data.moving_energy,
             s_ld2410c.data.stationary_distance, s_ld2410c.data.stationary_energy,
             s_ld2410c.data.detection_distance);
}

/**
 * @brief Parse command ACK frame
 */
static void parse_ack_frame(const uint8_t *buf, uint8_t len) {
    if (len < 10) {
        ESP_LOGW(TAG, "ACK frame too short: %d bytes", len);
        return;
    }

    if (!validate_header(buf, false)) {
        ESP_LOGW(TAG, "Invalid ACK frame header");
        return;
    }

    uint8_t cmd = buf[6];
    uint8_t status = buf[7];

    if (status == 0x00) {
        ESP_LOGD(TAG, "ACK for command 0x%02X: SUCCESS", cmd);
    } else {
        ESP_LOGW(TAG, "ACK for command 0x%02X: FAILED (status=0x%02X)", cmd, status);
    }
}

/**
 * @brief Process one byte from UART stream
 *
 * State machine to detect frame boundaries and parse frames.
 */
static void process_byte(uint8_t byte) {
    if (!s_ld2410c.in_frame) {
        // Look for frame header
        if (s_ld2410c.buffer_pos == 0 &&
            (byte == LD2410C_DATA_FRAME_HEADER_0 || byte == LD2410C_CMD_FRAME_HEADER_0)) {
            s_ld2410c.buffer[s_ld2410c.buffer_pos++] = byte;
        } else if (s_ld2410c.buffer_pos == 1 &&
                   (byte == LD2410C_DATA_FRAME_HEADER_1 || byte == LD2410C_CMD_FRAME_HEADER_1)) {
            s_ld2410c.buffer[s_ld2410c.buffer_pos++] = byte;
        } else if (s_ld2410c.buffer_pos == 2 &&
                   (byte == LD2410C_DATA_FRAME_HEADER_2 || byte == LD2410C_CMD_FRAME_HEADER_2)) {
            s_ld2410c.buffer[s_ld2410c.buffer_pos++] = byte;
        } else if (s_ld2410c.buffer_pos == 3 &&
                   (byte == LD2410C_DATA_FRAME_HEADER_3 || byte == LD2410C_CMD_FRAME_HEADER_3)) {
            s_ld2410c.buffer[s_ld2410c.buffer_pos++] = byte;
            s_ld2410c.in_frame = true;

            // Detect frame type
            if (s_ld2410c.buffer[0] == LD2410C_DATA_FRAME_HEADER_0) {
                s_ld2410c.frame_type = FRAME_TYPE_DATA;
            } else {
                s_ld2410c.frame_type = FRAME_TYPE_CMD_ACK;
            }
        } else {
            s_ld2410c.buffer_pos = 0;  // Reset on mismatch
        }
    } else {
        // Inside frame - collect bytes
        if (s_ld2410c.buffer_pos >= LD2410C_MAX_FRAME_SIZE) {
            ESP_LOGW(TAG, "Frame buffer overflow");
            s_ld2410c.buffer_pos = 0;
            s_ld2410c.in_frame = false;
            return;
        }

        s_ld2410c.buffer[s_ld2410c.buffer_pos++] = byte;

        // Check if we have length bytes (position 4 and 5)
        if (s_ld2410c.buffer_pos >= 6) {
            uint16_t data_len = two_byte_to_int(s_ld2410c.buffer[4], s_ld2410c.buffer[5]);
            uint16_t total_len = 4 + 2 + data_len + 4;  // header + length + data + footer

            // Sanity check
            if (total_len > LD2410C_MAX_FRAME_SIZE) {
                ESP_LOGW(TAG, "Invalid frame length: %d", data_len);
                s_ld2410c.buffer_pos = 0;
                s_ld2410c.in_frame = false;
                return;
            }

            // Check if we have complete frame
            if (s_ld2410c.buffer_pos >= total_len) {
                // Parse frame based on type
                if (s_ld2410c.frame_type == FRAME_TYPE_DATA) {
                    parse_data_frame(s_ld2410c.buffer, s_ld2410c.buffer_pos);
                } else {
                    parse_ack_frame(s_ld2410c.buffer, s_ld2410c.buffer_pos);
                }

                // Reset for next frame
                s_ld2410c.buffer_pos = 0;
                s_ld2410c.in_frame = false;
            }
        }
    }
}

/**
 * @brief UART receive task
 */
static void uart_rx_task(void *arg) {
    uint8_t *data = malloc(LD2410C_UART_BUF_SIZE);
    if (data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate UART buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UART receive task started");

    uint32_t total_bytes = 0;
    uint32_t last_log_time = 0;

    while (1) {
        int len = uart_read_bytes(s_ld2410c.uart_num, data, LD2410C_UART_BUF_SIZE, pdMS_TO_TICKS(100));

        if (len > 0) {
            total_bytes += (uint32_t)len;

            // Process each byte
            for (int i = 0; i < len; i++) {
                process_byte(data[i]);
            }
        }

        // Log status every 10 seconds
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS / 1000;
        if (now - last_log_time >= 10) {
            int64_t time_since_update = (esp_timer_get_time() - s_ld2410c.data.last_update_time) / 1000000;
            bool connected = (s_ld2410c.data.last_update_time > 0) && (time_since_update < 5);
            ESP_LOGI(TAG, "UART Status: %lu bytes received, Sensor: %s, Last update: %lld sec ago",
                     total_bytes, connected ? "CONNECTED" : "NOT CONNECTED", (long long)time_since_update);
            last_log_time = now;
        }
    }

    free(data);
    vTaskDelete(NULL);
}

// Public API Implementation

esp_err_t ld2410c_init(const ld2410c_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ld2410c.initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate configuration parameters
    if (config->uart_num >= UART_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid UART port: %d", config->uart_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (config->tx_pin >= GPIO_NUM_MAX || config->rx_pin >= GPIO_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid GPIO pins: TX=%d, RX=%d", config->tx_pin, config->rx_pin);
        return ESP_ERR_INVALID_ARG;
    }

    // Validate detection gates (protocol spec: gates 0-8, but 2-8 for max detection)
    if (config->max_move_gate < 2 || config->max_move_gate > 8) {
        ESP_LOGE(TAG, "Invalid max_move_gate: %d (must be 2-8)", config->max_move_gate);
        return ESP_ERR_INVALID_ARG;
    }

    if (config->max_still_gate < 2 || config->max_still_gate > 8) {
        ESP_LOGE(TAG, "Invalid max_still_gate: %d (must be 2-8)", config->max_still_gate);
        return ESP_ERR_INVALID_ARG;
    }

    // Validate baud rate (common rates supported by sensor)
    const uint32_t valid_bauds[] = {9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800};
    bool baud_valid = false;
    for (size_t i = 0; i < sizeof(valid_bauds) / sizeof(valid_bauds[0]); i++) {
        if (config->baud_rate == valid_bauds[i]) {
            baud_valid = true;
            break;
        }
    }
    if (!baud_valid) {
        ESP_LOGE(TAG, "Invalid baud rate: %lu (use 256000 for default)", config->baud_rate);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing LD2410C driver");
    ESP_LOGI(TAG, "UART config: TX=%d, RX=%d, Baud=%lu",
             config->tx_pin, config->rx_pin, config->baud_rate);

    // Create mutex
    s_ld2410c.data_mutex = xSemaphoreCreateMutex();
    if (s_ld2410c.data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = (int)config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(config->uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_ld2410c.data_mutex);
        return ret;
    }

    ret = uart_set_pin(config->uart_num, config->tx_pin, config->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_ld2410c.data_mutex);
        return ret;
    }

    ret = uart_driver_install(config->uart_num, LD2410C_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        vSemaphoreDelete(s_ld2410c.data_mutex);
        return ret;
    }

    s_ld2410c.uart_num = config->uart_num;

    // Create UART task
#ifndef CONFIG_LD2410C_UART_TASK_STACK_SIZE
    #define UART_TASK_STACK_SIZE 4096
#else
    #define UART_TASK_STACK_SIZE CONFIG_LD2410C_UART_TASK_STACK_SIZE
#endif

#ifndef CONFIG_LD2410C_UART_TASK_PRIORITY
    #define UART_TASK_PRIORITY 5
#else
    #define UART_TASK_PRIORITY CONFIG_LD2410C_UART_TASK_PRIORITY
#endif

    BaseType_t task_ret = xTaskCreate(uart_rx_task, "ld2410c_uart",
                                      UART_TASK_STACK_SIZE, NULL, UART_TASK_PRIORITY,
                                      &s_ld2410c.uart_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART task");
        uart_driver_delete(config->uart_num);
        vSemaphoreDelete(s_ld2410c.data_mutex);
        return ESP_FAIL;
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for UART to stabilize

    // Initialize sensor: Enable config mode, then disable to start streaming
    ESP_LOGI(TAG, "Sending initialization sequence...");

    ret = set_config_mode(true);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enable config mode (this may be normal if sensor is already streaming)");
    }

    vTaskDelay(pdMS_TO_TICKS(200));  // Wait for config mode

    ret = set_config_mode(false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to disable config mode");
    }

    vTaskDelay(pdMS_TO_TICKS(200));  // Wait for streaming to start

    s_ld2410c.initialized = true;
    ESP_LOGI(TAG, "LD2410C driver initialized successfully");

    return ESP_OK;
}

esp_err_t ld2410c_deinit(void) {
    if (!s_ld2410c.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_ld2410c.uart_task_handle != NULL) {
        vTaskDelete(s_ld2410c.uart_task_handle);
        s_ld2410c.uart_task_handle = NULL;
    }

    uart_driver_delete(s_ld2410c.uart_num);

    if (s_ld2410c.data_mutex != NULL) {
        vSemaphoreDelete(s_ld2410c.data_mutex);
        s_ld2410c.data_mutex = NULL;
    }

    memset(&s_ld2410c, 0, sizeof(s_ld2410c));

    ESP_LOGI(TAG, "LD2410C driver deinitialized");
    return ESP_OK;
}

esp_err_t ld2410c_get_data(ld2410c_data_t *data) {
    if (data == NULL || !s_ld2410c.initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ld2410c.data_mutex == NULL) {
        ESP_LOGE(TAG, "Mutex not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_ld2410c.data_mutex, portMAX_DELAY);
    memcpy(data, &s_ld2410c.data, sizeof(ld2410c_data_t));
    xSemaphoreGive(s_ld2410c.data_mutex);

    return ESP_OK;
}

bool ld2410c_is_connected(uint8_t timeout_sec) {
    if (!s_ld2410c.initialized) {
        return false;
    }

    // Thread-safe read of last_update_time
    xSemaphoreTake(s_ld2410c.data_mutex, portMAX_DELAY);
    int64_t last_update = s_ld2410c.data.last_update_time;
    xSemaphoreGive(s_ld2410c.data_mutex);

    if (last_update == 0) {
        return false;
    }

    int64_t time_since_update = (esp_timer_get_time() - last_update) / 1000000;
    return time_since_update < timeout_sec;
}

esp_err_t ld2410c_set_engineering_mode(bool enable) {
    if (!s_ld2410c.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = set_config_mode(true);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t cmd = enable ? LD2410C_CMD_ENABLE_ENG : LD2410C_CMD_DISABLE_ENG;
    ret = send_command(cmd, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t exit_ret = set_config_mode(false);
    if (ret == ESP_OK && exit_ret != ESP_OK) {
        ESP_LOGW(TAG, "Command succeeded but failed to exit config mode");
        return exit_ret;
    }

    return ret;
}

esp_err_t ld2410c_set_max_distances_timeout(uint8_t max_move_gate,
                                              uint8_t max_still_gate,
                                              uint16_t timeout_sec) {
    if (!s_ld2410c.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Validate parameters
    if (max_move_gate < 2 || max_move_gate > 8) {
        ESP_LOGE(TAG, "Invalid max_move_gate: %d (must be 2-8)", max_move_gate);
        return ESP_ERR_INVALID_ARG;
    }
    if (max_still_gate < 2 || max_still_gate > 8) {
        ESP_LOGE(TAG, "Invalid max_still_gate: %d (must be 2-8)", max_still_gate);
        return ESP_ERR_INVALID_ARG;
    }

    // Build command value (from protocol spec 2.2.3)
    uint8_t value[18];
    value[0] = 0x00;  value[1] = 0x00;  // Max movement distance word
    value[2] = max_move_gate;
    value[3] = 0x00;  value[4] = 0x00;  value[5] = 0x00;

    value[6] = 0x01;  value[7] = 0x00;  // Max resting distance word
    value[8] = max_still_gate;
    value[9] = 0x00;  value[10] = 0x00;  value[11] = 0x00;

    value[12] = 0x02;  value[13] = 0x00;  // No one duration word
    value[14] = (uint8_t)(timeout_sec & 0xFFU);
    value[15] = (uint8_t)((timeout_sec >> 8) & 0xFFU);
    value[16] = 0x00;  value[17] = 0x00;

    esp_err_t ret = set_config_mode(true);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    ret = send_command(LD2410C_CMD_MAXDIST_DURATION, value, sizeof(value));

    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t exit_ret = set_config_mode(false);
    if (ret == ESP_OK && exit_ret != ESP_OK) {
        ESP_LOGW(TAG, "Command succeeded but failed to exit config mode");
        return exit_ret;
    }

    return ret;
}

esp_err_t ld2410c_set_gate_sensitivity(uint8_t gate,
                                        uint8_t move_sensitivity,
                                        uint8_t still_sensitivity) {
    if (!s_ld2410c.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // Validate parameters
    if (gate >= LD2410C_MAX_GATES) {
        ESP_LOGE(TAG, "Invalid gate: %d (must be 0-%d)", gate, LD2410C_MAX_GATES - 1);
        return ESP_ERR_INVALID_ARG;
    }

    if (move_sensitivity > 100 || still_sensitivity > 100) {
        ESP_LOGE(TAG, "Invalid sensitivity values (must be 0-100)");
        return ESP_ERR_INVALID_ARG;
    }

    // Build command value (from protocol spec 2.2.7)
    uint8_t value[18];
    value[0] = 0x00;  value[1] = 0x00;  // Distance door word
    value[2] = gate;
    value[3] = 0x00;  value[4] = 0x00;  value[5] = 0x00;

    value[6] = 0x01;  value[7] = 0x00;  // Movement sensitivity word
    value[8] = move_sensitivity;
    value[9] = 0x00;  value[10] = 0x00;  value[11] = 0x00;

    value[12] = 0x02;  value[13] = 0x00;  // Static sensitivity word
    value[14] = still_sensitivity;
    value[15] = 0x00;  value[16] = 0x00;  value[17] = 0x00;

    esp_err_t ret = set_config_mode(true);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    ret = send_command(LD2410C_CMD_GATE_SENS, value, sizeof(value));

    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t exit_ret = set_config_mode(false);
    if (ret == ESP_OK && exit_ret != ESP_OK) {
        ESP_LOGW(TAG, "Command succeeded but failed to exit config mode");
        return exit_ret;
    }

    return ret;
}

esp_err_t ld2410c_get_version(char *version, size_t version_size) {
    // TODO: Implement version reading with ACK parsing
    if (version == NULL || !s_ld2410c.initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    snprintf(version, version_size, "Unknown");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t ld2410c_restart(void) {
    if (!s_ld2410c.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = set_config_mode(true);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(100));

    ret = send_command(LD2410C_CMD_RESTART, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(2000));  // Wait for restart

    return ret;
}

esp_err_t ld2410c_factory_reset(void) {
    if (!s_ld2410c.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = set_config_mode(true);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    ret = send_command(LD2410C_CMD_RESET, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t exit_ret = set_config_mode(false);
    if (ret == ESP_OK && exit_ret != ESP_OK) {
        ESP_LOGW(TAG, "Reset succeeded but failed to exit config mode");
        return exit_ret;
    }

    return ret;
}
