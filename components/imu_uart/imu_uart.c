#include "imu_uart.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "imu_uart";

/* WitMotion WIT-standard protocol (JY901P). Fixed 11-byte output frames:
 *   [0] 0x55  [1] type  [2..9] four int16 LE  [10] checksum
 * checksum = (sum of bytes 0..9) & 0xFF. */
#define IMU_FRAME_HEAD   0x55
#define IMU_FRAME_LEN    11

#define IMU_TYPE_TIME    0x50
#define IMU_TYPE_ACC     0x51
#define IMU_TYPE_GYRO    0x52
#define IMU_TYPE_ANGLE   0x53   /* roll/pitch/yaw + version */
#define IMU_TYPE_MAG     0x54
#define IMU_TYPE_QUAT    0x59   /* q0,q1,q2,q3 (scalar-first) */

#define PARSE_BUF_SIZE   64
#define UART_READ_CHUNK  128

typedef struct {
    imu_uart_cfg_t   cfg;
    const app_bus_t *bus;
    uint8_t  acc[PARSE_BUF_SIZE];
    size_t   acc_len;
    imu_sample_t cur;   /* accumulates fields across angle/quat frames */
} imu_ctx_t;

static inline int16_t rd_i16le(const uint8_t *p)
{
    return (int16_t)(p[0] | (p[1] << 8));
}

/* Decode one validated 11-byte frame. Returns true if a "publishable" frame
 * (angle or quaternion) was seen, so the task can post the sample. */
static bool imu_decode(imu_ctx_t *ctx, const uint8_t *f)
{
    switch (f[1]) {
    case IMU_TYPE_ANGLE:
        ctx->cur.roll  = rd_i16le(&f[2]) / 32768.0f * 180.0f;
        ctx->cur.pitch = rd_i16le(&f[4]) / 32768.0f * 180.0f;
        ctx->cur.yaw   = rd_i16le(&f[6]) / 32768.0f * 180.0f;
        return true;
    case IMU_TYPE_QUAT:
        ctx->cur.quat[0] = rd_i16le(&f[2]) / 32768.0f;  /* w */
        ctx->cur.quat[1] = rd_i16le(&f[4]) / 32768.0f;  /* x */
        ctx->cur.quat[2] = rd_i16le(&f[6]) / 32768.0f;  /* y */
        ctx->cur.quat[3] = rd_i16le(&f[8]) / 32768.0f;  /* z */
        return true;
    /* TODO: 0x51 accel / 0x52 gyro / 0x54 mag are decodable here too; not
     * currently propagated because imu_sample_t only carries orientation. */
    default:
        return false;
    }
}

static void imu_consume(imu_ctx_t *ctx, const uint8_t *data, size_t len)
{
    if (ctx->acc_len + len > PARSE_BUF_SIZE) {
        ctx->acc_len = 0;   /* orientation is periodic; safe to drop backlog */
    }
    memcpy(ctx->acc + ctx->acc_len, data, len);
    ctx->acc_len += len;

    size_t i = 0;
    while (ctx->acc_len - i >= IMU_FRAME_LEN) {
        if (ctx->acc[i] != IMU_FRAME_HEAD) {
            i++;
            continue;
        }
        const uint8_t *f = &ctx->acc[i];
        uint8_t sum = 0;
        for (int k = 0; k < 10; k++) {
            sum += f[k];
        }
        if (sum != f[10]) {
            i++;            /* bad checksum: resync from next byte */
            continue;
        }
        if (imu_decode(ctx, f)) {
            static uint32_t seq = 0;
            ctx->cur.seq = ++seq;
            if (xQueueSend(ctx->bus->imu_q, &ctx->cur, 0) != pdTRUE) {
                ESP_LOGD(TAG, "imu queue full, dropping sample");
            }
        }
        i += IMU_FRAME_LEN;
    }

    if (i > 0) {
        memmove(ctx->acc, ctx->acc + i, ctx->acc_len - i);
        ctx->acc_len -= i;
    }
}

static void imu_uart_task(void *arg)
{
    imu_ctx_t *ctx = (imu_ctx_t *)arg;
    uint8_t *buf = malloc(UART_READ_CHUNK);
    if (buf == NULL) {
        ESP_LOGE(TAG, "no memory for RX buffer");
        vTaskDelete(NULL);
        return;
    }

    ctx->cur.quat[0] = 1.0f;   /* identity until a quaternion frame arrives */

    ESP_LOGI(TAG, "imu_uart_task on UART%d @ %d baud", ctx->cfg.uart_port,
             ctx->cfg.baud_rate);

    /* NOTE: JY901P emits accel+gyro+angle by default at 9600. To also receive
     * quaternion (0x59) and/or raise the rate, configure the unit once (unlock
     * 0xFF 0xAA 0x69 0x88 0xB5, set RSW reg 0x02, save reg 0x00). See README.
     * TODO: send that config here if your unit is not already set up. */

    for (;;) {
        int n = uart_read_bytes(ctx->cfg.uart_port, buf, UART_READ_CHUNK,
                                pdMS_TO_TICKS(50));
        if (n > 0) {
            imu_consume(ctx, buf, (size_t)n);
        }
    }
}

int imu_uart_start(const imu_uart_cfg_t *cfg, const app_bus_t *bus)
{
    if (!cfg || !bus || !bus->imu_q) {
        return ESP_ERR_INVALID_ARG;
    }

    imu_ctx_t *ctx = calloc(1, sizeof(imu_ctx_t));
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

    ESP_ERROR_CHECK(uart_driver_install(cfg->uart_port, cfg->rx_buf_size, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(cfg->uart_port, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(cfg->uart_port, cfg->tx_gpio, cfg->rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    if (xTaskCreate(imu_uart_task, "imu_uart", 4096, ctx, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create imu_uart_task");
        free(ctx);
        return ESP_FAIL;
    }
    return ESP_OK;
}
