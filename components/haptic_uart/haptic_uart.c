#include "haptic_uart.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "haptic_uart";

/* PaXini "通信盒" protocol (通信盒通信协议 V1.0.5). Everything is little-endian.
 *   Request head : 0x55 0xAA   (host -> box)
 *   Response head: 0xAA 0x55   (box  -> host, for read/write)
 *   Auto-push    : 0xAA 0x56   (box  -> host, streamed sensor data)
 * Checksum is an LRC: two's complement of the 8-bit sum of all prior bytes. */
#define REQ_HEAD0        0x55
#define REQ_HEAD1        0xAA
#define PUSH_HEAD0       0xAA
#define PUSH_HEAD1       0x56

#define FUNC_READ        0x03
#define FUNC_WRITE       0x10

#define REG_DATA_TYPE    0x0016   /* bit0 = resultant force, bit1 = distributed */
#define REG_AUTO_PUSH_EN 0x0017   /* 0 = off, 1 = on */

#define DATA_TYPE_RESULTANT_ONLY 0x01
#define MODULE_RESULTANT_BYTES   6   /* Fx,Fy,Fz : 3 x int16 LE */
#define FORCE_SCALE_N            0.1f /* raw LSB -> Newtons */

/* Reassembly buffer for the streaming parser, and per-read chunk. */
#define PARSE_BUF_SIZE   1024
#define UART_READ_CHUNK  256

typedef struct {
    haptic_uart_cfg_t cfg;
    const app_bus_t  *bus;
    uint8_t  acc[PARSE_BUF_SIZE];
    size_t   acc_len;
} haptic_ctx_t;

/* LRC = ((~sum)+1) over the given bytes (matches the vendor reference). */
static uint8_t haptic_lrc(const uint8_t *p, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += p[i];
    }
    return (uint8_t)(~sum + 1);
}

/* Build a request frame into out[] and return its total length (0 on overflow).
 * Layout: 55 AA | 00 reserved | func | addr[2 LE] | len[2 LE] | data[N] | LRC */
static size_t haptic_build_req(uint8_t *out, size_t out_cap, uint8_t func,
                               uint16_t addr, const uint8_t *data, uint16_t n)
{
    size_t total = 8 + n + 1;
    if (total > out_cap) {
        return 0;
    }
    out[0] = REQ_HEAD0;
    out[1] = REQ_HEAD1;
    out[2] = 0x00;
    out[3] = func;
    out[4] = (uint8_t)(addr & 0xFF);
    out[5] = (uint8_t)(addr >> 8);
    out[6] = (uint8_t)(n & 0xFF);
    out[7] = (uint8_t)(n >> 8);
    if (n && data) {
        memcpy(&out[8], data, n);
    }
    out[total - 1] = haptic_lrc(out, total - 1);
    return total;
}

static void haptic_write_reg8(haptic_ctx_t *ctx, uint16_t addr, uint8_t value)
{
    uint8_t frame[16];
    size_t len = haptic_build_req(frame, sizeof(frame), FUNC_WRITE, addr, &value, 1);
    if (len) {
        uart_write_bytes(ctx->cfg.uart_port, (const char *)frame, len);
    }
}

/* Decode one validated auto-push frame's payload region. */
static void haptic_emit(haptic_ctx_t *ctx, uint8_t error_code,
                        const uint8_t *payload, size_t payload_len)
{
    static uint32_t seq = 0;
    haptic_sample_t s = { .seq = ++seq, .error_code = error_code };

    size_t n = payload_len / MODULE_RESULTANT_BYTES;
    if (n > HAPTIC_MAX_MODULES) {
        n = HAPTIC_MAX_MODULES;
    }
    for (size_t i = 0; i < n; i++) {
        const uint8_t *m = &payload[i * MODULE_RESULTANT_BYTES];
        int16_t fx = (int16_t)(m[0] | (m[1] << 8));
        int16_t fy = (int16_t)(m[2] | (m[3] << 8));
        int16_t fz = (int16_t)(m[4] | (m[5] << 8));
        s.force[i].fx = fx * FORCE_SCALE_N;
        s.force[i].fy = fy * FORCE_SCALE_N;
        s.force[i].fz = fz * FORCE_SCALE_N;
    }
    s.module_count = (uint16_t)n;

    if (xQueueSend(ctx->bus->haptic_q, &s, 0) != pdTRUE) {
        ESP_LOGD(TAG, "haptic queue full, dropping sample %u", (unsigned)seq);
    }
}

/*
 * Streaming parser. Auto-push frame:
 *   [0] 0xAA [1] 0x56 [2] reserved
 *   [3..4] valid_frame_len (LE)      == payload_len + 1 (the +1 is the error byte)
 *   [5] total error code
 *   [6 .. 6+payload_len-1] payload   (payload_len = valid_frame_len - 1)
 *   [last] LRC over all previous bytes
 * Total length = valid_frame_len + 6.
 *
 * TODO: distributed-force (register 0x0016 bit1) and per-module slicing by the
 * connection bitmap (0x0010-0x0013) are not decoded; we run in resultant-only
 * mode so payload is a flat array of 6-byte modules.
 */
static void haptic_consume(haptic_ctx_t *ctx, const uint8_t *data, size_t len)
{
    /* append into the accumulator, dropping oldest on overflow */
    if (ctx->acc_len + len > PARSE_BUF_SIZE) {
        size_t drop = (ctx->acc_len + len) - PARSE_BUF_SIZE;
        if (drop >= ctx->acc_len) {
            ctx->acc_len = 0;
        } else {
            memmove(ctx->acc, ctx->acc + drop, ctx->acc_len - drop);
            ctx->acc_len -= drop;
        }
    }
    memcpy(ctx->acc + ctx->acc_len, data, len);
    ctx->acc_len += len;

    size_t i = 0;
    while (ctx->acc_len - i >= 6) {  /* need at least head..len fields */
        if (ctx->acc[i] != PUSH_HEAD0 || ctx->acc[i + 1] != PUSH_HEAD1) {
            i++;                     /* resync one byte at a time */
            continue;
        }
        uint16_t vlen = ctx->acc[i + 3] | (ctx->acc[i + 4] << 8);
        size_t frame_len = (size_t)vlen + 6;
        if (vlen == 0 || frame_len > PARSE_BUF_SIZE) {
            i++;                     /* implausible length, skip this head */
            continue;
        }
        if (ctx->acc_len - i < frame_len) {
            break;                   /* wait for the rest of the frame */
        }
        uint8_t lrc = haptic_lrc(&ctx->acc[i], frame_len - 1);
        if (lrc == ctx->acc[i + frame_len - 1]) {
            uint8_t err = ctx->acc[i + 5];
            haptic_emit(ctx, err, &ctx->acc[i + 6], (size_t)vlen - 1);
            i += frame_len;
        } else {
            ESP_LOGD(TAG, "LRC mismatch, resync");
            i++;
        }
    }

    /* keep the unparsed tail */
    if (i > 0) {
        memmove(ctx->acc, ctx->acc + i, ctx->acc_len - i);
        ctx->acc_len -= i;
    }
}

static void haptic_uart_task(void *arg)
{
    haptic_ctx_t *ctx = (haptic_ctx_t *)arg;
    uint8_t *buf = malloc(UART_READ_CHUNK);
    if (buf == NULL) {
        ESP_LOGE(TAG, "no memory for RX buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "haptic_uart_task on UART%d @ %d baud", ctx->cfg.uart_port,
             ctx->cfg.baud_rate);

    /* Configure the box: resultant-force content, then enable auto-push. */
    vTaskDelay(pdMS_TO_TICKS(50));
    haptic_write_reg8(ctx, REG_DATA_TYPE, DATA_TYPE_RESULTANT_ONLY);
    vTaskDelay(pdMS_TO_TICKS(10));
    haptic_write_reg8(ctx, REG_AUTO_PUSH_EN, 0x01);
    ESP_LOGI(TAG, "auto-push enabled");

    for (;;) {
        int n = uart_read_bytes(ctx->cfg.uart_port, buf, UART_READ_CHUNK,
                                pdMS_TO_TICKS(20));
        if (n > 0) {
            haptic_consume(ctx, buf, (size_t)n);
        }
    }
}

int haptic_uart_start(const haptic_uart_cfg_t *cfg, const app_bus_t *bus)
{
    if (!cfg || !bus || !bus->haptic_q) {
        return ESP_ERR_INVALID_ARG;
    }

    haptic_ctx_t *ctx = calloc(1, sizeof(haptic_ctx_t));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }
    ctx->cfg = *cfg;
    ctx->bus = bus;

    const uart_config_t uart_cfg = {
        .baud_rate  = cfg->baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(cfg->uart_port, cfg->rx_buf_size,
                                        cfg->tx_buf_size, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(cfg->uart_port, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(cfg->uart_port, cfg->tx_gpio, cfg->rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (xTaskCreate(haptic_uart_task, "haptic_uart", 4096, ctx, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create haptic_uart_task");
        free(ctx);
        return ESP_FAIL;
    }
    return ESP_OK;
}
