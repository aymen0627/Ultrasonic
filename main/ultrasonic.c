#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#define TRIGGER_PIN  GPIO_NUM_5
#define ECHO_PIN_1   GPIO_NUM_25
#define ECHO_PIN_2   GPIO_NUM_27
#define ECHO_PIN_3   GPIO_NUM_26
#define ECHO_PIN_4   GPIO_NUM_32

#define MAX_DISTANCE_CM 300
#define TIMEOUT_US      20000

static const char *TAG = "ultrasonic";

void ultrasonic_task(void *pvParameters) {
    gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN_1, GPIO_MODE_INPUT);
    gpio_set_direction(ECHO_PIN_2, GPIO_MODE_INPUT);
    gpio_set_direction(ECHO_PIN_3, GPIO_MODE_INPUT);
    gpio_set_direction(ECHO_PIN_4, GPIO_MODE_INPUT);

    while (1) {
        // Trigger the ultrasonic sensors
        gpio_set_level(TRIGGER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(TRIGGER_PIN, 0);

        // Measure the pulse lengths of each sensor
        int64_t pulse_start_1, pulse_end_1;
        int64_t pulse_start_2, pulse_end_2;
        int64_t pulse_start_3, pulse_end_3;
        int64_t pulse_start_4, pulse_end_4;

        // Wait for the echo signals to start
        int64_t start_time = esp_timer_get_time();
        while (gpio_get_level(ECHO_PIN_1) == 0) {
            if ((esp_timer_get_time() - start_time) > TIMEOUT_US) {
                ESP_LOGI(TAG, "Timeout waiting for echo start (Sensor 1)");
                break;
            }
        }
        pulse_start_1 = esp_timer_get_time();

        start_time = esp_timer_get_time();
        while (gpio_get_level(ECHO_PIN_2) == 0) {
            if ((esp_timer_get_time() - start_time) > TIMEOUT_US) {
                ESP_LOGI(TAG, "Timeout waiting for echo start (Sensor 2)");
                break;
            }
        }
        pulse_start_2 = esp_timer_get_time();

        start_time = esp_timer_get_time();
        while (gpio_get_level(ECHO_PIN_3) == 0) {
            if ((esp_timer_get_time() - start_time) > TIMEOUT_US) {
                ESP_LOGI(TAG, "Timeout waiting for echo start (Sensor 3)");
                break;
            }
        }
        pulse_start_3 = esp_timer_get_time();

        start_time = esp_timer_get_time();
        while (gpio_get_level(ECHO_PIN_4) == 0) {
            if ((esp_timer_get_time() - start_time) > TIMEOUT_US) {
                ESP_LOGI(TAG, "Timeout waiting for echo start (Sensor 4)");
                break;
            }
        }
        pulse_start_4 = esp_timer_get_time();

        // Measure the pulse end times
        start_time = esp_timer_get_time();
        while (gpio_get_level(ECHO_PIN_1) == 1) {
            if ((esp_timer_get_time() - start_time) > TIMEOUT_US) {
                ESP_LOGI(TAG, "Timeout waiting for echo end (Sensor 1)");
                break;
            }
        }
        pulse_end_1 = esp_timer_get_time();

        start_time = esp_timer_get_time();
        while (gpio_get_level(ECHO_PIN_2) == 1) {
            if ((esp_timer_get_time() - start_time) > TIMEOUT_US) {
                ESP_LOGI(TAG, "Timeout waiting for echo end (Sensor 2)");
                break;
            }
        }
        pulse_end_2 = esp_timer_get_time();

        start_time = esp_timer_get_time();
        while (gpio_get_level(ECHO_PIN_3) == 1) {
            if ((esp_timer_get_time() - start_time) > TIMEOUT_US) {
                ESP_LOGI(TAG, "Timeout waiting for echo end (Sensor 3)");
                break;
            }
        }
        pulse_end_3 = esp_timer_get_time();

        start_time = esp_timer_get_time();
        while (gpio_get_level(ECHO_PIN_4) == 1) {
            if ((esp_timer_get_time() - start_time) > TIMEOUT_US) {
                ESP_LOGI(TAG, "Timeout waiting for echo end (Sensor 4)");
                break;
            }
        }
        pulse_end_4 = esp_timer_get_time();

        // Calculate the pulse durations
        int64_t pulse_duration_1 = pulse_end_1 - pulse_start_1;
        int64_t pulse_duration_2 = pulse_end_2 - pulse_start_2;
        int64_t pulse_duration_3 = pulse_end_3 - pulse_start_3;
        int64_t pulse_duration_4 = pulse_end_4 - pulse_start_4;

        // Calculate the distances
        double distance_1 = pulse_duration_1 * 0.034 / 2;
        double distance_2 = pulse_duration_2 * 0.034 / 2;
        double distance_3 = pulse_duration_3 * 0.034 / 2;
        double distance_4 = pulse_duration_4 * 0.034 / 2;

        // Limit the distances within the maximum range
        if (distance_1 > MAX_DISTANCE_CM) {
            distance_1 = MAX_DISTANCE_CM;
        }
        if (distance_2 > MAX_DISTANCE_CM) {
            distance_2 = MAX_DISTANCE_CM;
        }
        if (distance_3 > MAX_DISTANCE_CM) {
            distance_3 = MAX_DISTANCE_CM;
        }
        if (distance_4 > MAX_DISTANCE_CM) {
            distance_4 = MAX_DISTANCE_CM;
        }

        // Print the distances
        ESP_LOGI(TAG, "Distance 1: %.2f cm", distance_1);
        ESP_LOGI(TAG, "Distance 2: %.2f cm", distance_2);
        ESP_LOGI(TAG, "Distance 3: %.2f cm", distance_3);
        ESP_LOGI(TAG, "Distance 4: %.2f cm", distance_4);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main() {
    // Configure UART for logging
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    // Create the ultrasonic task
    xTaskCreate(ultrasonic_task, "ultrasonic_task", 4096, NULL, 5, NULL);
}
