/**
 * @file main.c
 * @brief Basic example for LD2410C sensor
 *
 * This example demonstrates:
 * - Initializing the sensor with default settings
 * - Reading presence detection data
 * - Checking connection status
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ld2410c.h"

static const char *TAG = "ld2410c_example";

// Pin configuration - adjust for your hardware
#define LD2410C_UART_NUM    UART_NUM_1
#define LD2410C_TX_PIN      17  // ESP32 TX -> LD2410C RX
#define LD2410C_RX_PIN      18  // ESP32 RX -> LD2410C TX

void app_main(void)
{
    ESP_LOGI(TAG, "LD2410C Basic Example");

    // Configure sensor
    ld2410c_config_t config = {
        .uart_num = LD2410C_UART_NUM,
        .tx_pin = LD2410C_TX_PIN,
        .rx_pin = LD2410C_RX_PIN,
        .baud_rate = LD2410C_DEFAULT_BAUD,  // 256000
        .max_move_gate = 8,      // Maximum detection range for moving targets
        .max_still_gate = 8,     // Maximum detection range for stationary targets
        .timeout_seconds = 5,    // Unoccupied timeout
        .engineering_mode = false
    };

    // Initialize driver
    esp_err_t ret = ld2410c_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LD2410C: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "LD2410C initialized successfully");
    ESP_LOGI(TAG, "Waiting for sensor data...");

    // Wait for first data
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Main loop
    while (1) {
        // Check connection status
        if (!ld2410c_is_connected(10)) {
            ESP_LOGW(TAG, "Sensor disconnected or not responding");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Read sensor data
        ld2410c_data_t data;
        ret = ld2410c_get_data(&data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to get data: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Display results
        if (data.presence_detected) {
            ESP_LOGI(TAG, "=== PRESENCE DETECTED ===");

            if (data.moving_target) {
                ESP_LOGI(TAG, "  Moving target: %d cm (energy: %d)",
                         data.moving_distance, data.moving_energy);
            }

            if (data.stationary_target) {
                ESP_LOGI(TAG, "  Stationary target: %d cm (energy: %d)",
                         data.stationary_distance, data.stationary_energy);
            }

            ESP_LOGI(TAG, "  Detection distance: %d cm", data.detection_distance);
        } else {
            ESP_LOGI(TAG, "No presence detected");
        }

        // Update every second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
