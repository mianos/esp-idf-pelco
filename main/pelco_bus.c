
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

static bool calculate_checksum(const uint8_t *msg, uint8_t *checksum)
{
    uint16_t sum = 0;
    for (int index = 1; index < 6; index++) {
        sum += msg[index];
    }
    *checksum = sum % 0x100;
    return true;
}

esp_err_t pelco_bus_init(pelco_bus_t *bus, pelco_baud_rate_t baud_rate)
{
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    esp_err_t initializationError = uart_param_config(bus->uart_num, &uart_config);
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

bool pelco_bus_command(pelco_bus_t *bus, bool disable_ack, uint8_t command, uint16_t data1, uint8_t data2)
{
    uint8_t msg[PELCO_MSG_LEN] = {0};
    msg[0] = 0xFF;
    msg[1] = bus->camera_address;
    msg[2] = 0x00;
    msg[3] = command;
    msg[4] = (uint8_t)(data1 & 0xFF);
    msg[5] = data2;
    calculate_checksum(msg, &msg[6]);

    if (bus->enable_pin != GPIO_NUM_NC) {
        gpio_set_level(bus->enable_pin, 1);
    }
    int written = uart_write_bytes(bus->uart_num, (const char *)msg, PELCO_MSG_LEN);
    if (written != PELCO_MSG_LEN) {
        ESP_LOGE(TAG, "Incomplete command write");
        return false;
    }
    if (!disable_ack) {
        if (bus->enable_pin != GPIO_NUM_NC) {
            gpio_set_level(bus->enable_pin, 0);
        }
        uint8_t ack[4] = {0};
        int ret = uart_read_bytes(bus->uart_num, ack, sizeof(ack), 100 / portTICK_PERIOD_MS);
        if (bus->enable_pin != GPIO_NUM_NC) {
            gpio_set_level(bus->enable_pin, 1);
        }
        if (ret != sizeof(ack)) {
            ESP_LOGE(TAG, "ACK timeout or wrong length");
            return false;
        }
        if (ack[0] != 0xFF || ack[1] != bus->camera_address || ack[2] != 0x00 || ack[3] != msg[6]) {
            ESP_LOGE(TAG, "Invalid ACK received");
            return false;
        }
        ESP_LOGI(TAG, "Command sent and ACK received");
    }
    return true;
}

uint16_t pelco_bus_request(pelco_bus_t *bus, uint8_t request, int timeout_ms)
{
    if (!pelco_bus_command(bus, true, request, 0x00, 0x00)) {
        return 0xFFFF;
    }
    if (bus->enable_pin != GPIO_NUM_NC) {
        gpio_set_level(bus->enable_pin, 0);
    }
    uint8_t response[PELCO_MSG_LEN] = {0};
    int ret = uart_read_bytes(bus->uart_num, response, sizeof(response), timeout_ms / portTICK_PERIOD_MS);
    if (bus->enable_pin != GPIO_NUM_NC) {
        gpio_set_level(bus->enable_pin, 1);
    }
    if (ret != sizeof(response)) {
        ESP_LOGE(TAG, "Request timeout or wrong response length");
        return 0xFFFF;
    }
    if (response[0] != 0xFF) {
        ESP_LOGE(TAG, "Response sync byte error");
        return 0xFFFF;
    }
    uint8_t checksum;
    calculate_checksum(response, &checksum);
    if (checksum != response[6]) {
        ESP_LOGE(TAG, "Response checksum error");
        return 0xFFFF;
    }
    return (((uint16_t)response[4]) << 8) | response[5];
}

bool pelco_bus_send_raw(pelco_bus_t *bus, const char *hex_string)
{
    char hexBuffer[128];
    strncpy(hexBuffer, hex_string, sizeof(hexBuffer));
    hexBuffer[sizeof(hexBuffer) - 1] = '\0';
    char *source = hexBuffer, *destination = hexBuffer;
    while (*source) {
        if (*source != ' ') {
            *destination++ = *source;
        }
        source++;
    }
    *destination = '\0';
    size_t hexLength = strlen(hexBuffer);
    if (hexLength % 2 != 0 || hexLength / 2 != PELCO_MSG_LEN) {
        ESP_LOGE(TAG, "Hex string length error");
        return false;
    }
    uint8_t rawCommand[PELCO_MSG_LEN];
    char byteString[3] = {0};
    for (size_t index = 0; index < PELCO_MSG_LEN; index++) {
        byteString[0] = hexBuffer[index * 2];
        byteString[1] = hexBuffer[index * 2 + 1];
        rawCommand[index] = (uint8_t)strtol(byteString, NULL, 16);
    }
    if (rawCommand[0] != 0xFF) {
        ESP_LOGW(TAG, "Fixing sync byte");
        rawCommand[0] = 0xFF;
    }
    uint8_t checksum;
    calculate_checksum(rawCommand, &checksum);
    if (checksum != rawCommand[6]) {
        ESP_LOGW(TAG, "Fixing checksum");
        rawCommand[6] = checksum;
    }
    if (bus->enable_pin != GPIO_NUM_NC) {
        gpio_set_level(bus->enable_pin, 1);
    }
    int writtenBytes = uart_write_bytes(bus->uart_num, (const char *)rawCommand, PELCO_MSG_LEN);
    if (writtenBytes != PELCO_MSG_LEN) {
        ESP_LOGE(TAG, "Failed to send raw command");
        return false;
    }
    return true;
}

static TaskHandle_t s_debugTaskHandle = NULL;

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

