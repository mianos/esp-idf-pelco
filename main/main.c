
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cmd_system.h"
#include "cmd_nvs.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "pelco_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if SOC_USB_SERIAL_JTAG_SUPPORTED
#if !CONFIG_ESP_CONSOLE_SECONDARY_NONE
#warning "A secondary serial console is not useful when using the console component. Please disable it in menuconfig."
#endif
#endif

static const char *TAG = "tilt";
static const char *BUILD_VER = "1n";

#define PROMPT_STR "tilt"
#define MOUNT_PATH "/data"
#define HISTORY_PATH MOUNT_PATH "/history.txt"
#define RS485_UART_NUM         UART_NUM_1
#define RS485_RX_PIN           GPIO_NUM_3
#define RS485_TX_PIN           GPIO_NUM_2
#define RS485_ENABLE_PIN       GPIO_NUM_NC
#define RS485_UART_BUFFER_SIZE (1024)

static void initialize_filesystem(void)
{
    static wl_handle_t wearLevelHandle;
    const esp_vfs_fat_mount_config_t mountConfig = {
        .max_files = 4,
        .format_if_mount_failed = true
    };
    esp_err_t error = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mountConfig, &wearLevelHandle);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(error));
    }
}

static void initialize_nvs(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);
}

typedef struct {
    pelco_bus_t *dev;
} cmd_context_t;

/**
 * @brief Handle tilt command with signed speed:
 *        Negative = tilt down, Positive = tilt up, Zero = stop.
 *        Speed range limited to -63..63 for typical Pelco-D cameras.
 */
static int cmd_tilt(void *context, int argc, char **argv)
{
    cmd_context_t *cmdContext = (cmd_context_t *)context;
    if (argc < 2) {
        printf("Usage: tilt <speed -63..63>\n");
        return 1;
    }
    int speedValue = atoi(argv[1]);
    if (speedValue < -63 || speedValue > 63) {
        printf("Tilt speed must be between -63 and 63.\n");
        return 1;
    }

    // Pelco-D bit definitions (commonly):
    // 0x08 = tilt up, 0x10 = tilt down, 0x00 = stop.
    uint8_t tiltCommand;
    uint8_t tiltSpeedByte = (uint8_t)((speedValue < 0) ? -speedValue : speedValue);
    uint8_t panSpeedByte = 0;

    if (speedValue > 0) {
        tiltCommand = 0x08; // tilt up
    } else if (speedValue < 0) {
        tiltCommand = 0x10; // tilt down
    } else {
        tiltCommand = 0x00; // stop
        tiltSpeedByte = 0;
    }

    if (!pelco_bus_command(cmdContext->dev, tiltCommand, panSpeedByte, tiltSpeedByte)) {
        printf("Failed to send tilt command.\n");
        return 1;
    }
    printf("Tilting with speed %d.\n", speedValue);
    return 0;
}

/**
 * @brief Handle pan command with signed speed:
 *        Negative = pan left, Positive = pan right, Zero = stop.
 *        Speed range limited to -63..63 for typical Pelco-D cameras.
 */
static int cmd_pan(void *context, int argc, char **argv)
{
    cmd_context_t *cmdContext = (cmd_context_t *)context;
    if (argc < 2) {
        printf("Usage: pan <speed -63..63>\n");
        return 1;
    }
    int speedValue = atoi(argv[1]);
    if (speedValue < -63 || speedValue > 63) {
        printf("Pan speed must be between -63 and 63.\n");
        return 1;
    }

    // Pelco-D bit definitions (commonly):
    // 0x02 = pan right, 0x04 = pan left, 0x00 = stop.
    uint8_t panCommand;
    uint8_t panSpeedByte = (uint8_t)((speedValue < 0) ? -speedValue : speedValue);
    uint8_t tiltSpeedByte = 0;

    if (speedValue > 0) {
        panCommand = 0x02; // pan right
    } else if (speedValue < 0) {
        panCommand = 0x04; // pan left
    } else {
        panCommand = 0x00; // stop
        panSpeedByte = 0;
    }

    if (!pelco_bus_command(cmdContext->dev, panCommand, panSpeedByte, tiltSpeedByte)) {
        printf("Failed to send pan command.\n");
        return 1;
    }
    printf("Rotating with speed %d.\n", speedValue);
    return 0;
}

/**
 * @brief Handle absolute pan command using extended Pelco-D opcode 0x4B:
 *        Byte4 = MSB, Byte5 = LSB of position (0..65535).
 */
static int cmd_pan_abs(void *context, int argc, char **argv)
{
    cmd_context_t *cmdContext = (cmd_context_t *)context;
    if (argc < 2) {
        printf("Usage: pan_abs <position 0..65535>\n");
        return 1;
    }
    int positionValue = atoi(argv[1]);
    if (positionValue < 0 || positionValue > 65535) {
        printf("Position must be between 0 and 65535.\n");
        return 1;
    }
    uint16_t positionWord = (uint16_t)positionValue;
    uint8_t msbValue = (uint8_t)((positionWord >> 8) & 0xFF);
    uint8_t lsbValue = (uint8_t)(positionWord & 0xFF);

    // 0x4B is often used for "Set Pan Position" in extended Pelco variants.
    if (!pelco_bus_command(cmdContext->dev, 0x4B, msbValue, lsbValue)) {
        printf("Failed to set absolute pan position.\n");
        return 1;
    }
    printf("Setting absolute pan position to %d.\n", positionValue);
    return 0;
}

/**
 * @brief Handle absolute tilt command using extended Pelco-D opcode 0x3D:
 *        Byte4 = MSB, Byte5 = LSB of position (0..65535).
 */
static int cmd_tilt_abs(void *context, int argc, char **argv)
{
    cmd_context_t *cmdContext = (cmd_context_t *)context;
    if (argc < 2) {
        printf("Usage: tilt_abs <position 0..65535>\n");
        return 1;
    }
    int positionValue = atoi(argv[1]);
    if (positionValue < 0 || positionValue > 65535) {
        printf("Position must be between 0 and 65535.\n");
        return 1;
    }
    uint16_t positionWord = (uint16_t)positionValue;
    uint8_t msbValue = (uint8_t)((positionWord >> 8) & 0xFF);
    uint8_t lsbValue = (uint8_t)(positionWord & 0xFF);

    // 0x3D is often used for "Set Tilt Position" in extended Pelco variants.
    if (!pelco_bus_command(cmdContext->dev, 0x3D, msbValue, lsbValue)) {
        printf("Failed to set absolute tilt position.\n");
        return 1;
    }
    printf("Setting absolute tilt position to %d.\n", positionValue);
    return 0;
}

static int cmd_build_ver(void *context, int argc, char **argv)
{
    printf("Build ver is '%s'\n", BUILD_VER);
    return 0;
}

static int cmd_pelco_log(void *context, int argc, char **argv)
{
    cmd_context_t *cmdContext = (cmd_context_t *)context;
    if (argc < 2) {
        printf("Usage: pelco_log <on|off>\n");
        return 1;
    }
    if (strcmp(argv[1], "on") == 0) {
        if (pelco_bus_debug_start(cmdContext->dev)) {
            printf("Pelco bus debugging started.\n");
        } else {
            printf("Failed to start pelco bus debugging or already running.\n");
        }
    } else if (strcmp(argv[1], "off") == 0) {
        pelco_bus_debug_stop();
        printf("Pelco bus debugging stopped.\n");
    } else {
        printf("Invalid argument. Usage: pelco_log <on|off>\n");
        return 1;
    }
    return 0;
}

void app_main(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t replConfig = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    replConfig.prompt = PROMPT_STR ">";
    replConfig.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;

    initialize_nvs();

#if CONFIG_CONSOLE_STORE_HISTORY
    initialize_filesystem();
    replConfig.history_save_path = HISTORY_PATH;
    ESP_LOGI(TAG, "Command history enabled");
#else
    ESP_LOGI(TAG, "Command history disabled");
#endif

    esp_console_register_help_command();
    register_system_common();
    register_nvs();

    static pelco_bus_t pelcoDevice = {
        .uart_num = RS485_UART_NUM,
        .tx_pin = RS485_TX_PIN,
        .rx_pin = RS485_RX_PIN,
        .enable_pin = RS485_ENABLE_PIN,
        .camera_address = 1,
    };

    esp_err_t initializationError = pelco_bus_init(&pelcoDevice, PELCO_BAUD_2400);
    if (initializationError != ESP_OK) {
        ESP_LOGE("APP_MAIN", "Pelco bus initialization failed: %s", esp_err_to_name(initializationError));
        return;
    }
    ESP_LOGI("APP_MAIN", "Pelco bus initialized successfully");

    static cmd_context_t ctx = {
        .dev = &pelcoDevice,
    };

    // Signed tilt command
    const esp_console_cmd_t tiltCmd = {
        .command = "tilt",
        .help = "Tilt turret: negative=down, positive=up, 0=stop (-63..63)",
        .hint = "<speed>",
        .func_w_context = cmd_tilt,
        .context = &ctx,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&tiltCmd));

    // Signed pan command
    const esp_console_cmd_t panCmd = {
        .command = "pan",
        .help = "Pan turret: negative=left, positive=right, 0=stop (-63..63)",
        .hint = "<speed>",
        .func_w_context = cmd_pan,
        .context = &ctx,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&panCmd));

    // Absolute pan command
    const esp_console_cmd_t panAbsCmd = {
        .command = "pan_abs",
        .help = "Set absolute pan position (0..65535)",
        .hint = "<position>",
        .func_w_context = cmd_pan_abs,
        .context = &ctx,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&panAbsCmd));

    // Absolute tilt command
    const esp_console_cmd_t tiltAbsCmd = {
        .command = "tilt_abs",
        .help = "Set absolute tilt position (0..65535)",
        .hint = "<position>",
        .func_w_context = cmd_tilt_abs,
        .context = &ctx,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&tiltAbsCmd));

    const esp_console_cmd_t buildVerCmd = {
        .command = "build_ver",
        .help = "Show build version",
        .func_w_context = cmd_build_ver,
        .context = &ctx,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&buildVerCmd));

    const esp_console_cmd_t pelcoLogCmd = {
        .command = "pelco_log",
        .help = "Control pelco bus debugging (on/off)",
        .hint = "<on|off>",
        .func_w_context = cmd_pelco_log,
        .context = &ctx,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&pelcoLogCmd));

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    {
        esp_console_dev_uart_config_t hwConfig = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_console_new_repl_uart(&hwConfig, &replConfig, &repl));
    }
#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    {
        esp_console_dev_usb_cdc_config_t hwConfig = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hwConfig, &replConfig, &repl));
    }
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    {
        esp_console_dev_usb_serial_jtag_config_t hwConfig = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hwConfig, &replConfig, &repl));
    }
#else
#error Unsupported console type
#endif

    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

