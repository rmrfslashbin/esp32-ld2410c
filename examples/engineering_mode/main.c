/**
 * @file main.c
 * @brief Engineering mode example for LD2410C sensor
 *
 * This example demonstrates:
 * - Enabling engineering mode for detailed gate energy readings
 * - Configuring sensitivity for specific gates
 * - Displaying per-gate energy values for calibration
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ld2410c.h"

static const char *TAG = "ld2410c_eng_example";

// Pin configuration - adjust for your hardware
#define LD2410C_UART_NUM    UART_NUM_1
#define LD2410C_TX_PIN      17
#define LD2410C_RX_PIN      18

void app_main(void)
{
    ESP_LOGI(TAG, "LD2410C Engineering Mode Example");

    // Configure sensor
    ld2410c_config_t config = {
        .uart_num = LD2410C_UART_NUM,
        .tx_pin = LD2410C_TX_PIN,
        .rx_pin = LD2410C_RX_PIN,
        .baud_rate = LD2410C_DEFAULT_BAUD,
        .max_move_gate = 8,
        .max_still_gate = 8,
        .timeout_seconds = 5,
        .engineering_mode = false  // Will enable after init
    };

    // Initialize driver
    esp_err_t ret = ld2410c_init(&config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LD2410C: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "LD2410C initialized");

    // Enable engineering mode for detailed gate data
    ESP_LOGI(TAG, "Enabling engineering mode...");
    ret = ld2410c_set_engineering_mode(true);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enable engineering mode: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Engineering mode enabled");
    }

    // Optional: Adjust sensitivity for specific gates
    // Example: Increase sensitivity for gate 3
    ESP_LOGI(TAG, "Configuring gate sensitivities...");
    ret = ld2410c_set_gate_sensitivity(3, 60, 60);  // Gate 3: 60% sensitivity
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set gate 3 sensitivity");
    }

    // Wait for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Starting gate energy monitoring...");
    ESP_LOGI(TAG, "Gate distances: 0=0.75m, 1=1.5m, 2=2.25m, 3=3m, 4=3.75m, 5=4.5m, 6=5.25m, 7=6m, 8=6.75m\n");

    // Main loop
    while (1) {
        if (!ld2410c_is_connected(10)) {
            ESP_LOGW(TAG, "Sensor disconnected");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ld2410c_data_t data;
        ret = ld2410c_get_data(&data);
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Display gate energy table
        printf("\n========== Gate Energy Report ==========\n");
        printf("Presence: %s\n", data.presence_detected ? "DETECTED" : "None");
        if (data.presence_detected) {
            printf("Moving: %dcm (energy:%d) | Still: %dcm (energy:%d)\n\n",
                   data.moving_distance, data.moving_energy,
                   data.stationary_distance, data.stationary_energy);
        } else {
            printf("\n");
        }

        printf("Gate | Distance | Move Energy | Still Energy\n");
        printf("-----|----------|-------------|-------------\n");

        for (int i = 0; i < LD2410C_MAX_GATES; i++) {
            // Calculate distance in meters (each gate = 0.75m)
            float distance = (i + 1) * 0.75;

            // Highlight gates with high energy
            char move_marker = (data.gate_move_energy[i] > 40) ? '*' : ' ';
            char still_marker = (data.gate_still_energy[i] > 40) ? '*' : ' ';

            printf("  %d  |  %.2fm   |     %3d %c   |     %3d %c\n",
                   i, distance,
                   data.gate_move_energy[i], move_marker,
                   data.gate_still_energy[i], still_marker);
        }

        printf("========================================\n");
        printf("* = Energy > 40 (high activity)\n\n");

        // Update every 2 seconds for readability
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
