
#include "pelco_bus.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

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
    esp_err_t error = uart_param_config(bus->uart_num, &uart_config);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed");
        return error;
    }
    error = uart_driver_install(bus->uart_num, 1024, 0, 0, NULL, 0);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed");
        return error;
    }
	uart_set_pin(bus->uart_num, bus->tx_pin, bus->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // num, tx, rx ...
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
    char hex_buf[128];
    strncpy(hex_buf, hex_string, sizeof(hex_buf));
    hex_buf[sizeof(hex_buf)-1] = '\0';
    char *src = hex_buf, *dst = hex_buf;
    while (*src) {
        if (*src != ' ') {
            *dst++ = *src;
        }
        src++;
    }
    *dst = '\0';
    size_t hex_len = strlen(hex_buf);
    if (hex_len % 2 != 0 || hex_len / 2 != PELCO_MSG_LEN) {
        ESP_LOGE(TAG, "Hex string length error");
        return false;
    }
    uint8_t raw_command[PELCO_MSG_LEN];
    char byte_str[3] = {0};
    for (size_t index = 0; index < PELCO_MSG_LEN; index++) {
        byte_str[0] = hex_buf[index * 2];
        byte_str[1] = hex_buf[index * 2 + 1];
        raw_command[index] = (uint8_t)strtol(byte_str, NULL, 16);
    }
    if (raw_command[0] != 0xFF) {
        ESP_LOGW(TAG, "Fixing sync byte");
        raw_command[0] = 0xFF;
    }
    uint8_t checksum;
    calculate_checksum(raw_command, &checksum);
    if (checksum != raw_command[6]) {
        ESP_LOGW(TAG, "Fixing checksum");
        raw_command[6] = checksum;
    }
    if (bus->enable_pin != GPIO_NUM_NC) {
        gpio_set_level(bus->enable_pin, 1);
    }
    int written = uart_write_bytes(bus->uart_num, (const char *)raw_command, PELCO_MSG_LEN);
    if (written != PELCO_MSG_LEN) {
        ESP_LOGE(TAG, "Failed to send raw command");
        return false;
    }
    return true;
}


bool pelco_bus_send_ef(pelco_bus_t *bus)
{
    const size_t messageLength = 100;
    uint8_t rawCommand[messageLength];
    memset(rawCommand, 0xEF, messageLength);

    if (bus->enable_pin != GPIO_NUM_NC) {
        gpio_set_level(bus->enable_pin, 1);
    }

    int writtenBytes = uart_write_bytes(bus->uart_num, (const char *)rawCommand, messageLength);
    if (writtenBytes != messageLength) {
        ESP_LOGE(TAG, "Failed to send EF command");
        return false;
    }
    return true;
}
