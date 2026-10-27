#include "transport_uart.h"
#include "app_proto.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "transport_uart";

#define USJ_RX_BUF          1024
#define USJ_TX_BUF          1024
#define RX_CHUNK            256

static const app_bus_t   *s_bus = NULL;
static app_proto_parser_t s_parser;   /* reassembly for the inbound byte stream */

/* app_proto sink: write a built frame straight out the USB-Serial/JTAG. */
static void uart_sink(const uint8_t *frame, size_t len, void *ctx)
{
    (void)ctx;
    /* Short timeout: the supervisor calls this from its 10 ms control loop,
     * and a host that stops reading must not stretch that loop. A frame cut
     * short is dropped by the host's CRC check. */
    usb_serial_jtag_write_bytes(frame, len, pdMS_TO_TICKS(5));
}

/* decoded command -> supervisor queue */
static void uart_cmd_cb(const app_cmd_t *cmd, void *ctx)
{
    (void)ctx;
    if (s_bus && s_bus->ctrl_q) {
        xQueueSend(s_bus->ctrl_q, cmd, 0);
    }
}

static void uart_rx_task(void *arg)
{
    (void)arg;
    uint8_t *buf = malloc(RX_CHUNK);
    if (!buf) { ESP_LOGE(TAG, "no memory for RX buffer"); vTaskDelete(NULL); return; }
    ESP_LOGI(TAG, "uart_rx_task running");
    for (;;) {
        int n = usb_serial_jtag_read_bytes(buf, RX_CHUNK, pdMS_TO_TICKS(20));
        if (n > 0) {
            app_proto_feed(&s_parser, APP_TR_USB, buf, (size_t)n, uart_cmd_cb, NULL);
        }
    }
}

int transport_uart_start(const app_bus_t *bus)
{
    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }
    s_bus = bus;
    s_parser.len = 0;

    /* Raw byte I/O on the native USB. Coexists with the console: app_proto frames
     * (SOF+CRC) and ESP_LOG text share the link; the host parser skips the text. */
    usb_serial_jtag_driver_config_t cfg = {
        .rx_buffer_size = USJ_RX_BUF,
        .tx_buffer_size = USJ_TX_BUF,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {   /* already installed = OK */
        ESP_LOGE(TAG, "usb_serial_jtag install failed: %s", esp_err_to_name(err));
        return err;
    }

    app_proto_set_sink(APP_TR_USB, uart_sink, NULL);

    if (xTaskCreate(uart_rx_task, "usj_rx", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create uart_rx_task");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "UART transport up (USB-Serial/JTAG, app_proto)");
    return ESP_OK;
}
