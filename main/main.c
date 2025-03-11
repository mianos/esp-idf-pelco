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

#if SOC_USB_SERIAL_JTAG_SUPPORTED
#if !CONFIG_ESP_CONSOLE_SECONDARY_NONE
#warning "A secondary serial console is not useful when using the console component. Please disable it in menuconfig."
#endif
#endif

static const char* TAG = "tilt";
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
    static wl_handle_t wl_handle;
    const esp_vfs_fat_mount_config_t mount_config = {
        .max_files = 4,
        .format_if_mount_failed = true
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(MOUNT_PATH, "storage", &mount_config, &wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
}


static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

typedef struct {
	pelco_bus_t *dev;
} cmd_context_t;


/* New command: tilt X degrees */
static int cmd_tilt(void *context, int argc, char **argv)
{
    cmd_context_t *ctx = (cmd_context_t *)context;
    if (argc < 2) {
        printf("Usage: tilt <degrees>\n");
        return 1;
    }
    int degrees = atoi(argv[1]);
    /* Convert degrees to a 16-bit value in hundredths of a degree */
    uint16_t angle = (uint16_t)(degrees * 100);
    uint8_t high_byte = (uint8_t)(angle >> 8);
    uint8_t low_byte = (uint8_t)(angle & 0xFF);

    if (!pelco_bus_command(ctx->dev, false, PELCO_CMD_TILT_SET, high_byte, low_byte)) {
        printf("Failed to send tilt command.\n");
        return 1;
    }
    printf("Tilting turret to %d degrees.\n", degrees);
    return 0;
}

/* New command: rotate Y degrees */
static int cmd_rotate(void *context, int argc, char **argv)
{
    cmd_context_t *ctx = (cmd_context_t *)context;
    if (argc < 2) {
        printf("Usage: rotate <degrees>\n");
        return 1;
    }
    int degrees = atoi(argv[1]);
    /* Convert degrees to a 16-bit value in hundredths of a degree */
    uint16_t angle = (uint16_t)(degrees * 100);
    uint8_t high_byte = (uint8_t)(angle >> 8);
    uint8_t low_byte = (uint8_t)(angle & 0xFF);

    if (!pelco_bus_command(ctx->dev, false, PELCO_CMD_ROTATE_SET, high_byte, low_byte)) {
        printf("Failed to send rotate command.\n");
        return 1;
    }
    printf("Rotating turret to %d degrees.\n", degrees);
    return 0;
}

static int cmd_build_ver(void *context, int argc, char **argv) {
//    cmd_context_t *ctx = (cmd_context_t *)context;
    printf("Build ver is '%s'\n", BUILD_VER);
    return 0;
}

static int cmd_send_ef(void *context, int argc, char **argv) {
    cmd_context_t *ctx = (cmd_context_t *)context;
    printf("sent status %d\n", pelco_bus_send_ef(ctx->dev));
    return 0;
}


void app_main(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    /* Prompt to be printed before each line.
       This can be customized, made dynamic, etc.
    */
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;
    
    initialize_nvs();
    
#if CONFIG_CONSOLE_STORE_HISTORY
    initialize_filesystem();
    repl_config.history_save_path = HISTORY_PATH;
    ESP_LOGI(TAG, "Command history enabled");
#else
    ESP_LOGI(TAG, "Command history disabled");
#endif
    
    /* Register built-in commands */
    esp_console_register_help_command();
    register_system_common();
    register_nvs();



    /* Configure the Pelco bus structure with member-by-member initialization */
    static pelco_bus_t pelcoDevice = {
        .uart_num = RS485_UART_NUM,
		.tx_pin = RS485_TX_PIN,
		.rx_pin = RS485_RX_PIN,
        .enable_pin = RS485_ENABLE_PIN,
        .camera_address = 1,  /* Change to your camera's address if necessary */
    };

    esp_err_t initializationError = pelco_bus_init(&pelcoDevice, PELCO_BAUD_9600);
    if (initializationError != ESP_OK) {
        ESP_LOGE("APP_MAIN", "Pelco bus initialization failed: %s", esp_err_to_name(initializationError));
        return;
    }
    ESP_LOGI("APP_MAIN", "Pelco bus initialized successfully");

	static cmd_context_t	ctx = {
		.dev =  &pelcoDevice,
	};

    /* Register turret commands */
    {
        const esp_console_cmd_t tilt_cmd = {
            .command = "tilt",
            .help = "Tilt turret by specified degrees",
            .hint = "<degrees>",
            .func_w_context = cmd_tilt,
			.context = &ctx,
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&tilt_cmd));
    }
    {
        const esp_console_cmd_t rotate_cmd = {
            .command = "rotate",
            .help = "Rotate turret by specified degrees",
            .hint = "<degrees>",
            .func_w_context = cmd_rotate,
			.context = &ctx,
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&rotate_cmd));
	}
	{
        const esp_console_cmd_t build_ver_cmd = {
            .command = "build_ver",
            .help = "Show build version",
            .func_w_context = cmd_build_ver,
			.context = &ctx,
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&build_ver_cmd));
    }
	{
        const esp_console_cmd_t send_ef_cmd = {
            .command = "send_ef",
            .help = "send 0xEF for testing",
            .func_w_context = cmd_send_ef,
			.context = &ctx,
        };
        ESP_ERROR_CHECK(esp_console_cmd_register(&send_ef_cmd));
    }
    
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    {
        esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
    }
#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    {
        esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));
    }
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    {
        esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
    }
#else
#error Unsupported console type
#endif
    
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
	while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

