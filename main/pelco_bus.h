#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

/* Define Pelco command codes for setting tilt and pan (rotate) positions.
   These values are exemplary and should be adjusted per your camera's documentation. */
#define PELCO_CMD_TILT_SET   0x51
#define PELCO_CMD_ROTATE_SET 0x53

typedef enum {
    PELCO_BAUD_2400 = 2400,
    PELCO_BAUD_4800 = 4800,
    PELCO_BAUD_9600 = 9600,
} pelco_baud_rate_t;

typedef struct {
    int uart_num;
    int enable_pin;      /* If < 0, pin is unused */
    uint8_t camera_address;
} pelco_bus_t;

esp_err_t pelco_bus_init(pelco_bus_t *bus, pelco_baud_rate_t baud_rate);
bool pelco_bus_command(pelco_bus_t *bus, bool disable_ack, uint8_t command, uint16_t data1, uint8_t data2);
uint16_t pelco_bus_request(pelco_bus_t *bus, uint8_t request, int timeout_ms);
bool pelco_bus_send_raw(pelco_bus_t *bus, const char *hex_string);

#ifdef __cplusplus
}
#endif

