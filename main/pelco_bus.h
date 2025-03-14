#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

/* Define Pelco command codes for setting tilt and pan (rotate) positions.
   These values are exemplary and should be adjusted per your camera's documentation. */
//#define PELCO_CMD_TILT_SET   0x51
//#define PELCO_CMD_ROTATE_SET 0x53

#define PELCO_CMD_TILT_CMD 0x10
#define PELCO_CMD_ROTATE_CMD 0x04

typedef enum {
    PELCO_BAUD_2400 = 2400,
    PELCO_BAUD_4800 = 4800,
    PELCO_BAUD_9600 = 9600,
} pelco_baud_rate_t;

typedef struct {
    uart_port_t uart_num;
	gpio_num_t tx_pin;
	gpio_num_t rx_pin;
    gpio_num_t enable_pin;      /* If < 0, pin is unused */
    uint8_t camera_address;
} pelco_bus_t;

esp_err_t pelco_bus_init(pelco_bus_t *bus, pelco_baud_rate_t baud_rate);
bool pelco_bus_command(pelco_bus_t *bus, uint8_t command, uint8_t data1, uint8_t data2);
bool pelco_bus_debug_start(pelco_bus_t *bus);
void pelco_bus_debug_stop(void);

#ifdef __cplusplus
}
#endif

