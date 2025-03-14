#include "pelco_bus.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "pelco_bus";
#define PELCO_MSG_LEN 7

// Declare the debug task handle at the top for global access.
static TaskHandle_t s_debugTaskHandle = NULL;

static void pelco_bus_print_debug(const char *direction, const uint8_t *buffer, int length)
{
    printf("Debug: %s %d bytes: ", direction, length);
    for (int index = 0; index < length; index++) {
        printf("%02X ", buffer[index]);
    }
    printf("\n");
}

static bool calculate_checksum(const uint8_t *message, uint8_t *checksum)
{
    uint16_t sum = 0;
    for (int index = 1; index < 6; index++) {
        sum += message[index];
    }
    *checksum = sum % 0x100;
    return true;
}

esp_err_t pelco_bus_init(pelco_bus_t *bus, pelco_baud_rate_t baud_rate)
{
    uart_config_t uartConfig = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    esp_err_t initializationError = uart_param_config(bus->uart_num, &uartConfig);
    if (initializationError != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed");
        return initializationError;
    }
    initializationError = uart_driver_install(bus->uart_num, 1024, 0, 0, NULL, 0);
    if (initializationError != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed");
        return initializationError;
    }
    uart_set_pin(bus->uart_num, bus->tx_pin, bus->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (bus->enable_pin != GPIO_NUM_NC) {
        esp_rom_gpio_pad_select_gpio(bus->enable_pin);
        gpio_set_direction(bus->enable_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(bus->enable_pin, 1);
    }
    return ESP_OK;
}


bool pelco_bus_command(pelco_bus_t *bus, uint8_t command, uint8_t data1, uint8_t data2)
{
    uint8_t message[PELCO_MSG_LEN] = {0};
    message[0] = 0xFF;
    message[1] = bus->camera_address;
    message[2] = 0x00;
    message[3] = command;
    message[4] = data1;
    message[5] = data2;
    calculate_checksum(message, &message[6]);

    if (bus->enable_pin != GPIO_NUM_NC) {
        gpio_set_level(bus->enable_pin, 1);
    }
    int writtenBytes = uart_write_bytes(bus->uart_num, (const char *)message, PELCO_MSG_LEN);
    if (writtenBytes != PELCO_MSG_LEN) {
        ESP_LOGE(TAG, "Incomplete command write");
        return false;
    }
    if (s_debugTaskHandle != NULL) {
        pelco_bus_print_debug("Sent", message, PELCO_MSG_LEN);
    }
    // ACK processing removed as it's not part of the standard protocol.
    return true;
}


static void pelco_bus_debug_task(void *pvParameters)
{
    pelco_bus_t *bus = (pelco_bus_t *)pvParameters;
    uint8_t debugBuffer[256];
    while (1) {
        int bytesRead = uart_read_bytes(bus->uart_num, debugBuffer, sizeof(debugBuffer), pdMS_TO_TICKS(100));
        if (bytesRead > 0) {
            printf("Debug: Received %d bytes: ", bytesRead);
            for (int index = 0; index < bytesRead; index++) {
                printf("%02X ", debugBuffer[index]);
            }
            printf("\n");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool pelco_bus_debug_start(pelco_bus_t *bus)
{
    if (s_debugTaskHandle != NULL) {
        ESP_LOGW(TAG, "Debug bus already running");
        return false;
    }
    if (xTaskCreate(pelco_bus_debug_task, "pelco_bus_debug", 4096, (void *)bus, 5, &s_debugTaskHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create debug task");
        return false;
    }
    ESP_LOGI(TAG, "Debug bus started");
    return true;
}

void pelco_bus_debug_stop(void)
{
    if (s_debugTaskHandle != NULL) {
        vTaskDelete(s_debugTaskHandle);
        s_debugTaskHandle = NULL;
        ESP_LOGI(TAG, "Debug bus stopped");
    } else {
        ESP_LOGW(TAG, "Debug bus is not running");
    }
}

