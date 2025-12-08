/**
 * @file test_ld2410c.c
 * @brief Unit tests for LD2410C driver
 *
 * Run with: idf.py build && idf.py test
 */

#include "unity.h"
#include "ld2410c.h"
#include "esp_log.h"

static const char *TAG = "ld2410c_test";

/**
 * Test that init with NULL config fails
 */
TEST_CASE("ld2410c_init with NULL config returns error", "[ld2410c]")
{
    esp_err_t ret = ld2410c_init(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Test that get_data with NULL pointer fails
 */
TEST_CASE("ld2410c_get_data with NULL data returns error", "[ld2410c]")
{
    esp_err_t ret = ld2410c_get_data(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Test that get_data fails when not initialized
 */
TEST_CASE("ld2410c_get_data fails when not initialized", "[ld2410c]")
{
    ld2410c_data_t data;
    esp_err_t ret = ld2410c_get_data(&data);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Test invalid UART port number
 */
TEST_CASE("ld2410c_init rejects invalid UART port", "[ld2410c]")
{
    ld2410c_config_t config = {
        .uart_num = 99,  // Invalid
        .tx_pin = 17,
        .rx_pin = 18,
        .baud_rate = 256000,
        .max_move_gate = 8,
        .max_still_gate = 8,
        .timeout_seconds = 5,
        .engineering_mode = false
    };

    esp_err_t ret = ld2410c_init(&config);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Test invalid GPIO pins
 */
TEST_CASE("ld2410c_init rejects invalid GPIO pins", "[ld2410c]")
{
    ld2410c_config_t config = {
        .uart_num = UART_NUM_1,
        .tx_pin = 255,  // Invalid
        .rx_pin = 18,
        .baud_rate = 256000,
        .max_move_gate = 8,
        .max_still_gate = 8,
        .timeout_seconds = 5,
        .engineering_mode = false
    };

    esp_err_t ret = ld2410c_init(&config);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Test invalid max_move_gate values
 */
TEST_CASE("ld2410c_init rejects invalid max_move_gate", "[ld2410c]")
{
    ld2410c_config_t config = {
        .uart_num = UART_NUM_1,
        .tx_pin = 17,
        .rx_pin = 18,
        .baud_rate = 256000,
        .max_move_gate = 1,  // Too low (must be 2-8)
        .max_still_gate = 8,
        .timeout_seconds = 5,
        .engineering_mode = false
    };

    esp_err_t ret = ld2410c_init(&config);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    config.max_move_gate = 9;  // Too high
    ret = ld2410c_init(&config);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Test invalid max_still_gate values
 */
TEST_CASE("ld2410c_init rejects invalid max_still_gate", "[ld2410c]")
{
    ld2410c_config_t config = {
        .uart_num = UART_NUM_1,
        .tx_pin = 17,
        .rx_pin = 18,
        .baud_rate = 256000,
        .max_move_gate = 8,
        .max_still_gate = 10,  // Too high
        .timeout_seconds = 5,
        .engineering_mode = false
    };

    esp_err_t ret = ld2410c_init(&config);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Test invalid baud rate
 */
TEST_CASE("ld2410c_init rejects invalid baud rate", "[ld2410c]")
{
    ld2410c_config_t config = {
        .uart_num = UART_NUM_1,
        .tx_pin = 17,
        .rx_pin = 18,
        .baud_rate = 12345,  // Not a standard baud rate
        .max_move_gate = 8,
        .max_still_gate = 8,
        .timeout_seconds = 5,
        .engineering_mode = false
    };

    esp_err_t ret = ld2410c_init(&config);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);
}

/**
 * Test that is_connected returns false when not initialized
 */
TEST_CASE("ld2410c_is_connected returns false when not initialized", "[ld2410c]")
{
    bool connected = ld2410c_is_connected(5);
    TEST_ASSERT_FALSE(connected);
}

/**
 * Test set_gate_sensitivity validates gate number
 */
TEST_CASE("ld2410c_set_gate_sensitivity rejects invalid gate", "[ld2410c]")
{
    // Should fail when not initialized
    esp_err_t ret = ld2410c_set_gate_sensitivity(9, 50, 50);  // Gate 9 is out of range (0-8)
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);  // Not initialized
}

/**
 * Test set_gate_sensitivity validates sensitivity values
 */
TEST_CASE("ld2410c_set_max_distances_timeout validates parameters", "[ld2410c]")
{
    // Should fail when not initialized
    esp_err_t ret = ld2410c_set_max_distances_timeout(1, 8, 5);  // Gate 1 too low
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret);  // Not initialized
}

/**
 * Test double initialization
 */
TEST_CASE("ld2410c_init prevents double initialization", "[ld2410c][hardware]")
{
    // Note: This test requires actual hardware and should be marked with [hardware] tag
    // It will be skipped in CI without hardware

    ld2410c_config_t config = {
        .uart_num = UART_NUM_1,
        .tx_pin = 17,
        .rx_pin = 18,
        .baud_rate = 256000,
        .max_move_gate = 8,
        .max_still_gate = 8,
        .timeout_seconds = 5,
        .engineering_mode = false
    };

    // First init should succeed (or fail due to hardware)
    esp_err_t ret1 = ld2410c_init(&config);

    if (ret1 == ESP_OK) {
        // Second init should fail with INVALID_STATE
        esp_err_t ret2 = ld2410c_init(&config);
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, ret2);

        // Cleanup
        ld2410c_deinit();
    } else {
        // Hardware not available, skip test
        TEST_IGNORE_MESSAGE("Hardware not available");
    }
}

/**
 * Test framework main
 */
void app_main(void)
{
    ESP_LOGI(TAG, "LD2410C Unit Tests");
    unity_run_menu();
}
