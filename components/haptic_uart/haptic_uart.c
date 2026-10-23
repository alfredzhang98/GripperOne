#include "haptic_uart.h"

#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "haptic_uart";

/* PaXini PX-6AX GEN3 "高速通信集成板" UART protocol (通信盒通信协议 V1.0.5
 * sections 4.4-4.6). The ESP talks to the integration board's UART header, not
 * to a sensor module's own pins, so this is the board's register protocol -
 * NOT the sensor-direct 0x7B/0x79 protocol in the sensor manual section 5.4,
 * and NOT the raw 0xAA 0x56 stream on its own. Everything is little-endian.
 *
 *   request  : 55 AA | 00 | func | reg_addr(2) | num(2) | send_data[N]    | LRC
 *   response : AA 55 | 00 | func | reg_addr(2) | num(2) | recv_data[M]    | LRC
 *   auto-push: AA 56 | 00 | frame_len(2)       | total_err(1) | data[M]   | LRC
 *
 * LRC is the two's complement of the sum of every preceding byte.
 */
#define BOX_REQ_HEAD0   0x55
#define BOX_REQ_HEAD1   0xAA
#define BOX_RESP_HEAD0  0xAA
#define BOX_RESP_HEAD1  0x55
#define BOX_FUNC_READ   0x03 /* read consecutive registers, max 512 bytes  */
#define BOX_FUNC_WRITE  0x10 /* write consecutive registers, max 10 bytes  */
#define BOX_FUNC_CALIB  0x17 /* re-zero, seen in the vendor Hand_UI tool   */

/* Register map, section 4.6. Every register holds one byte. */
#define REG_VERSION      0x0000u /* 16 ASCII bytes, e.g. "V1.0.0"          */
#define REG_SENSOR_COMB  0x0010u /* 4 bytes: which of the 28 modules exist */
#define REG_DATA_TYPE    0x0016u /* bit0 resultant, bit1 distributed       */
#define REG_AUTO_POST_EN 0x0017u /* 1 = stream, 0 = request/response only  */
#define REG_CALIB        0x0002u /* used with BOX_FUNC_CALIB, write 1      */
#define REG_TOTAL_ERR    0x0023u
#define REG_ERR_BASE     0x0700u /* uint16 error code per module          */
#define BOX_ERR_ABSENT   0x0001u /* code every unpopulated slot reports   */
#define BOX_CLAIM_WAIT_MS 4000   /* how long the board may take to rescan */
#define REG_POINTS_BASE  0x0030u /* uint16 point count per module          */
#define REG_FORCE_BASE   0x0500u /* 6 bytes per module: fx, fy, fz int16   */
#define REG_DIST_BASE    0x1000u /* per-module distributed force block     */
#define REG_DIST_STRIDE  0x0200u /* valid for the 20 finger modules (0..19)*/

/* Module ordering, section 4.6: thumb, index, middle, ring, little, palm,
 * each finger as proximal / middle / fingertip / nail. A CP-M3025 is the
 * proximal ("近节") module of its finger. */
#define BOX_MODULE_COUNT   28u  /* 5 fingers x 4 segments + 8 palm          */
#define BOX_FINGER_MODULES 20u
#define REG_FORCE_BYTES     6u
#define BOX_POINT_BYTES     3u
#define BOX_DIST_BYTES      (HAPTIC_POINT_COUNT * BOX_POINT_BYTES) /* 231 */

#define BOX_FRAME_OVERHEAD  9u  /* head(2) reserved(1) func(1) addr(2) num(2) lrc(1) */
#define BOX_MAX_PAYLOAD     BOX_DIST_BYTES
#define BOX_RESP_MAX        (BOX_FRAME_OVERHEAD + BOX_MAX_PAYLOAD)
#define BOX_FORCE_SCALE_N   0.1f

/* A 240-byte response is 2.6 ms at 921600 baud; 30 ms leaves >10x margin
 * without stalling the poll loop when a module is missing. Calibration
 * physically takes about a second (the vendor tool waits 1 s). */
#define BOX_REPLY_TIMEOUT_MS  30
#define BOX_CALIB_TIMEOUT_MS  1500

/* The ESP is running about 130 ms after power-on; the integration board's own
 * MCU takes noticeably longer, and it shares the supply, so a single probe at
 * start-up asks a peer that is not listening yet. Keep asking until it answers
 * rather than declaring the link dead - the earlier version of this driver
 * gave up at the first silence and then reported every slot as "unknown" for
 * the rest of the session even though polling worked fine seconds later. */
#define BOX_BOOT_WAIT_MS      15000
#define BOX_BOOT_RETRY_MS     400

typedef struct {
    haptic_uart_cfg_t cfg;
    const app_bus_t  *bus;
    volatile bool acquisition_enabled;
    volatile bool calibrate_pending;
    uint32_t tare_count;   /* finished tares; guarded by s_control_mux */
    bool tare_ok;          /* result of the last one                   */
    uint32_t seq[HAPTIC_SENSOR_COUNT];
    uint32_t fail_count[HAPTIC_SENSOR_COUNT];
    int64_t last_fail_log_us[HAPTIC_SENSOR_COUNT];
    uint8_t slot_status[HAPTIC_SENSOR_COUNT]; /* haptic_slot_status_t          */
    uint8_t modules[HAPTIC_SENSOR_COUNT];     /* board slot feeding each panel */
} haptic_ctx_t;

static haptic_ctx_t *s_ctx;
static portMUX_TYPE s_control_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t box_lrc(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum + data[i]);
    }
    return (uint8_t)(~sum + 1u);
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

/* Register block of a module, section 4.6. */
static uint16_t module_force_addr(uint8_t module)
{
    return (uint16_t)(REG_FORCE_BASE + (uint16_t)module * REG_FORCE_BYTES);
}

static uint16_t module_dist_addr(uint8_t module)
{
    return (uint16_t)(REG_DIST_BASE + (uint16_t)module * REG_DIST_STRIDE);
}

/* 0x0010..0x0013 pack the 28 modules as one bit each, lowest module first. */
static bool module_connected(const uint8_t comb[4], uint8_t module)
{
    return (comb[module / 8u] >> (module % 8u)) & 1u;
}

static size_t build_frame(uint8_t *out, uint8_t func, uint16_t reg,
                          const uint8_t *data, uint16_t data_len)
{
    size_t k = 0;
    out[k++] = BOX_REQ_HEAD0;
    out[k++] = BOX_REQ_HEAD1;
    out[k++] = 0x00; /* 预留 */
    out[k++] = func;
    put_u16(&out[k], reg);      k += 2;
    put_u16(&out[k], data_len); k += 2;
    if (data && data_len) {
        memcpy(&out[k], data, data_len);
        k += data_len;
    }
    out[k] = box_lrc(out, k);
    return k + 1;
}

/* Read one complete AA 55 response, resynchronising on the head. The declared
 * length is only trusted once the LRC checks out. */
static int read_response(int port, uint8_t *out, size_t cap, int timeout_ms)
{
    size_t used = 0;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

    while (esp_timer_get_time() < deadline) {
        int n = uart_read_bytes(port, out + used, cap - used, pdMS_TO_TICKS(1));
        if (n > 0) {
            used += (size_t)n;
        }

        size_t start = 0;
        while (start + 1 < used &&
               (out[start] != BOX_RESP_HEAD0 || out[start + 1] != BOX_RESP_HEAD1)) {
            start++;
        }
        if (start > 0) {
            memmove(out, out + start, used - start);
            used -= start;
        }
        if (used >= BOX_FRAME_OVERHEAD - 1u) {
            size_t total = (size_t)get_u16(&out[6]) + BOX_FRAME_OVERHEAD;
            if (total > cap) {
                memmove(out, out + 1, --used);
                continue;
            }
            if (used >= total) {
                return box_lrc(out, total - 1) == out[total - 1] ? (int)total : -2;
            }
        }
        if (used == cap) {
            return -3;
        }
    }
    return 0;
}

static bool transact(haptic_ctx_t *ctx, const uint8_t *tx, size_t tx_len,
                     uint8_t *rx, size_t rx_cap, int timeout_ms, int *rx_len)
{
    uart_flush_input(ctx->cfg.uart_port);
    int written = uart_write_bytes(ctx->cfg.uart_port, tx, tx_len);
    if (written != (int)tx_len ||
        uart_wait_tx_done(ctx->cfg.uart_port, pdMS_TO_TICKS(20)) != ESP_OK) {
        return false;
    }
    int n = read_response(ctx->cfg.uart_port, rx, rx_cap, timeout_ms);
    if (rx_len) {
        *rx_len = n;
    }
    return n > 0;
}

/* Read `len` register bytes into `data`. Returns false unless the board echoed
 * the same function code, register and length back. */
static bool box_read(haptic_ctx_t *ctx, uint16_t reg, uint16_t len,
                     uint8_t *data, int timeout_ms)
{
    uint8_t tx[BOX_FRAME_OVERHEAD];
    uint8_t rx[BOX_RESP_MAX];
    int rx_len = 0;

    if (len > BOX_MAX_PAYLOAD) {
        return false;
    }
    size_t tx_len = build_frame(tx, BOX_FUNC_READ, reg, NULL, len);
    if (!transact(ctx, tx, tx_len, rx, sizeof(rx), timeout_ms, &rx_len)) {
        return false;
    }
    if (rx_len != (int)(BOX_FRAME_OVERHEAD + len) || rx[2] != 0x00 ||
        rx[3] != BOX_FUNC_READ || get_u16(&rx[4]) != reg ||
        get_u16(&rx[6]) != len) {
        return false;
    }
    memcpy(data, &rx[8], len);
    return true;
}

/* Write one register byte. Section 4.4.1: the reply data is the write status,
 * zero meaning success. */
static bool box_write_u8(haptic_ctx_t *ctx, uint8_t func, uint16_t reg,
                         uint8_t value, int timeout_ms)
{
    uint8_t tx[BOX_FRAME_OVERHEAD + 1u];
    uint8_t rx[BOX_RESP_MAX];
    int rx_len = 0;

    size_t tx_len = build_frame(tx, func, reg, &value, 1u);
    if (!transact(ctx, tx, tx_len, rx, sizeof(rx), timeout_ms, &rx_len)) {
        return false;
    }
    return rx_len == (int)(BOX_FRAME_OVERHEAD + 1u) && rx[3] == func &&
           get_u16(&rx[4]) == reg && rx[8] == 0x00;
}

static void queue_sample(haptic_ctx_t *ctx, uint8_t sensor_index, uint8_t module,
                         const uint8_t force[REG_FORCE_BYTES],
                         const uint8_t *points)
{
    haptic_sample_t sample = {
        .seq = ++ctx->seq[sensor_index],
        .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000),
        .sensor_index = sensor_index,
        .device_addr = module,
        .status = ctx->slot_status[sensor_index],
        .point_count = HAPTIC_POINT_COUNT,
    };

    /* The resultant is stored as three int16, but only the low byte carries the
     * value: Fx/Fy signed, Fz unsigned, 0.1 N per LSB. This matches the sensor
     * manual section 5.6.2 ranges and the vendor Hand_UI parser, which reads
     * the same six bytes and uses byte 0, 2 and 4. */
    sample.resultant[0] = (float)(int8_t)force[0] * BOX_FORCE_SCALE_N;
    sample.resultant[1] = (float)(int8_t)force[2] * BOX_FORCE_SCALE_N;
    sample.resultant[2] = (float)force[4] * BOX_FORCE_SCALE_N;

    for (size_t i = 0; i < HAPTIC_POINT_COUNT; i++) {
        sample.fx[i] = (int8_t)points[i * BOX_POINT_BYTES];
        sample.fy[i] = (int8_t)points[i * BOX_POINT_BYTES + 1u];
        sample.fz[i] = points[i * BOX_POINT_BYTES + 2u];
    }

    if (xQueueSend(ctx->bus->haptic_q, &sample, 0) != pdTRUE) {
        haptic_sample_t stale;
        (void)xQueueReceive(ctx->bus->haptic_q, &stale, 0);
        (void)xQueueSend(ctx->bus->haptic_q, &sample, 0);
        ESP_LOGD(TAG, "haptic queue full; dropped oldest sample");
    }
}

static const char *sensor_name(uint8_t sensor_index)
{
    return sensor_index == 0 ? "ring" : "little";
}

static bool poll_sensor(haptic_ctx_t *ctx, uint8_t sensor_index, uint8_t module)
{
    uint8_t force[REG_FORCE_BYTES];
    uint8_t points[BOX_DIST_BYTES];

    bool ok = box_read(ctx, module_force_addr(module), REG_FORCE_BYTES, force,
                       BOX_REPLY_TIMEOUT_MS) &&
              box_read(ctx, module_dist_addr(module), BOX_DIST_BYTES, points,
                       BOX_REPLY_TIMEOUT_MS);
    if (!ok) {
        ctx->fail_count[sensor_index]++;
        int64_t now = esp_timer_get_time();
        /* Once at the first failure, then a reminder a minute: a board that is
         * unplugged on purpose must not fill the console. */
        if (ctx->fail_count[sensor_index] == 1 ||
            now - ctx->last_fail_log_us[sensor_index] >= 60000000) {
            ESP_LOGW(TAG, "%s (module %u, force 0x%04X, dist 0x%04X): no valid "
                          "reply (failures=%u; next reminder in 60 s)",
                     sensor_name(sensor_index), module, module_force_addr(module),
                     module_dist_addr(module), (unsigned)ctx->fail_count[sensor_index]);
            ctx->last_fail_log_us[sensor_index] = now;
        }
        return false;
    }

    if (ctx->fail_count[sensor_index] != 0) {
        ESP_LOGI(TAG, "%s reconnected after %u failed polls",
                 sensor_name(sensor_index), (unsigned)ctx->fail_count[sensor_index]);
        ctx->fail_count[sensor_index] = 0;
    }
    queue_sample(ctx, sensor_index, module, force, points);
    return true;
}

/* Section 4.4.2 note 4 / vendor Hand_UI: function 0x17, register 2, value 1
 * re-zeroes every connected module at once, and takes about a second. */
static bool calibrate_all(haptic_ctx_t *ctx)
{
    return box_write_u8(ctx, BOX_FUNC_CALIB, REG_CALIB, 1u, BOX_CALIB_TIMEOUT_MS);
}

/* ------------------------------------------------------------------------
 * Start-up discovery, section 4.5.1 steps 1-2. This both configures the board
 * and makes a wiring or module-slot mistake obvious in the log instead of
 * showing up as a silent stream of failed polls.
 * ---------------------------------------------------------------------- */
static void discover(haptic_ctx_t *ctx)
{
    uint8_t version[16];
    uint8_t comb[4];

    ESP_LOGW(TAG, "=== integration-board discovery: UART%d TX=GPIO%d RX=GPIO%d @ %d baud ===",
             ctx->cfg.uart_port, ctx->cfg.tx_gpio, ctx->cfg.rx_gpio, ctx->cfg.baud_rate);

    uart_flush_input(ctx->cfg.uart_port);

    /* Wait for the board to finish its own boot before believing a silence. */
    int64_t give_up = esp_timer_get_time() + (int64_t)BOX_BOOT_WAIT_MS * 1000;
    bool answered = false;
    unsigned attempts = 0;
    while (!answered) {
        /* Section 4.5.1 is request/response, so make sure any leftover stream
         * from a previous session or the vendor tool is switched off first.
         * This is the only register this driver writes on its own initiative,
         * and it only ever affects streaming, never module enumeration. */
        (void)box_write_u8(ctx, BOX_FUNC_WRITE, REG_AUTO_POST_EN, 0u,
                           BOX_REPLY_TIMEOUT_MS);
        uart_flush_input(ctx->cfg.uart_port);
        answered = box_read(ctx, REG_VERSION, sizeof(version), version,
                            BOX_REPLY_TIMEOUT_MS);
        attempts++;
        if (answered || esp_timer_get_time() >= give_up) {
            break;
        }
        if (attempts == 1) {
            ESP_LOGW(TAG, "board has not answered yet; waiting up to %d ms for it "
                          "to finish booting", BOX_BOOT_WAIT_MS);
        }
        vTaskDelay(pdMS_TO_TICKS(BOX_BOOT_RETRY_MS));
    }

    if (answered) {
        char text[sizeof(version) + 1];
        size_t k = 0;
        for (size_t i = 0; i < sizeof(version); i++) {
            text[k++] = (version[i] >= 0x20 && version[i] < 0x7F) ? (char)version[i] : ' ';
        }
        text[k] = '\0';
        ESP_LOGI(TAG, "board firmware: \"%s\" (answered on attempt %u)", text, attempts);
    } else {
        ESP_LOGE(TAG, "no reply to the version read (0x0000) after %u attempts "
                      "over %d ms. Check the 5 V supply, the UART header wiring "
                      "(GPIO17 is TX into the board's RX) and the shared ground.",
                 attempts, BOX_BOOT_WAIT_MS);
        return;
    }

    if (!box_read(ctx, REG_SENSOR_COMB, sizeof(comb), comb, BOX_REPLY_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "could not read the module map at 0x0010");
        return;
    }
    ESP_LOGI(TAG, "module map 0x0010..0x0013 = %02X %02X %02X %02X",
             comb[0], comb[1], comb[2], comb[3]);

    /* Section 4.6: the board latches which modules exist ONCE, at its own
     * power-up, so a sensor plugged in afterwards stays invisible. Do NOT try
     * to fix that from here by writing DEVICE_RST (0x0022): measured on
     * ADAPTER_FW02.26, the board does not come back from that soft restart and
     * stays mute until it is physically power-cycled, which is far worse than
     * the missing module it was meant to recover. Report it and let the
     * operator power-cycle with both sensors already connected. */
    uint8_t points_raw[BOX_MODULE_COUNT * 2u];
    if (!box_read(ctx, REG_POINTS_BASE, sizeof(points_raw), points_raw,
                  BOX_REPLY_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "could not read the point-count table at 0x0030");
        return;
    }

    /* 0x0700..0x0737 carries a 16-bit error code per module in the same order
     * as the resultant block. Measured on this board: 0x0000 on the healthy
     * module, 0x0001 on every genuinely empty slot, and a distinct non-0x0001
     * code on a slot that has something attached the board cannot talk to. So
     * this table, not the point count alone, is what separates "empty" from
     * "faulty" - and the difference matters, because one means check the wiring
     * and the other means the board already knows something is wrong. */
    uint8_t errs[BOX_MODULE_COUNT * 2u];
    bool have_errs = box_read(ctx, REG_ERR_BASE, sizeof(errs), errs,
                              BOX_REPLY_TIMEOUT_MS);
    if (!have_errs) {
        ESP_LOGW(TAG, "could not read the per-module error table at 0x0700");
        memset(errs, 0, sizeof(errs));
    }

    static const char *const names[BOX_MODULE_COUNT] = {
        "thumb.prox",  "thumb.mid",  "thumb.tip",  "thumb.nail",
        "index.prox",  "index.mid",  "index.tip",  "index.nail",
        "middle.prox", "middle.mid", "middle.tip", "middle.nail",
        "ring.prox",   "ring.mid",   "ring.tip",   "ring.nail",
        "little.prox", "little.mid", "little.tip", "little.nail",
        "palm1", "palm2", "palm3", "palm4", "palm5", "palm6", "palm7", "palm8",
    };
    uint8_t found[BOX_MODULE_COUNT];
    uint8_t found_n = 0;
    for (uint8_t m = 0; m < BOX_MODULE_COUNT; m++) {
        uint16_t count = get_u16(&points_raw[m * 2u]);
        bool present = module_connected(comb, m);
        if (count == 0 && !present) {
            continue; /* an empty slot is the normal case; do not spam */
        }
        ESP_LOGI(TAG, "  module %2u %-12s map=%d points=%u err=0x%04X "
                      "dist=0x%04X force=0x%04X",
                 m, names[m], present ? 1 : 0, count, get_u16(&errs[m * 2u]),
                 module_dist_addr(m), module_force_addr(m));
        if (count == HAPTIC_POINT_COUNT && found_n < BOX_MODULE_COUNT) {
            found[found_n++] = m;
        }
    }
    ESP_LOGI(TAG, "board reports %u module(s) with %u points", found_n, HAPTIC_POINT_COUNT);

    /* Prefer what the board actually reports over the configured guess. */
    if (found_n == HAPTIC_SENSOR_COUNT) {
        for (uint8_t i = 0; i < HAPTIC_SENSOR_COUNT; i++) {
            if (ctx->modules[i] != found[i]) {
                ESP_LOGW(TAG, "%s: using module %u (%s) instead of the configured %u",
                         sensor_name(i), found[i], names[found[i]], ctx->modules[i]);
                ctx->modules[i] = found[i];
            }
            ctx->slot_status[i] = HAPTIC_SLOT_OK;
        }
    } else {
        for (uint8_t i = 0; i < HAPTIC_SENSOR_COUNT; i++) {
            uint8_t module = ctx->modules[i];
            uint16_t count = get_u16(&points_raw[module * 2u]);
            uint16_t code = get_u16(&errs[module * 2u]);
            if (!module_connected(comb, module) && count == 0) {
                ctx->slot_status[i] = HAPTIC_SLOT_EMPTY;
                if (code == BOX_ERR_ABSENT || !have_errs) {
                    ESP_LOGE(TAG, "%s: module %u (%s) is empty (err 0x%04X, the "
                                  "same code every unpopulated slot reports)",
                             sensor_name(i), module, names[module], code);
                } else {
                    ESP_LOGE(TAG, "%s: module %u (%s) reports error 0x%04X, which "
                                  "is NOT the 0x%04X that empty slots report - "
                                  "something is attached that the board cannot "
                                  "talk to",
                             sensor_name(i), module, names[module], code,
                             BOX_ERR_ABSENT);
                }
            } else if (count != HAPTIC_POINT_COUNT) {
                ctx->slot_status[i] = HAPTIC_SLOT_WRONG_SIZE;
                ESP_LOGE(TAG, "%s: module %u (%s) reports %u points, expected %u",
                         sensor_name(i), module, names[module], count,
                         HAPTIC_POINT_COUNT);
            } else {
                ctx->slot_status[i] = HAPTIC_SLOT_OK;
            }
        }
        ESP_LOGW(TAG, "expected %d modules with %u points but found %u",
                 HAPTIC_SENSOR_COUNT, HAPTIC_POINT_COUNT, found_n);
        ESP_LOGW(TAG, "A slot that reads empty here is usually the 5 V supply: on "
                      "3V3 the board runs, answers, and enumerates the FIRST "
                      "sensor only. Give it 4.5-5 V and power-cycle it - the "
                      "module map is latched once at ITS power-up (section 4.6).");
    }

    uint8_t err = 0;
    if (box_read(ctx, REG_TOTAL_ERR, 1u, &err, BOX_REPLY_TIMEOUT_MS) && err != 0) {
        ESP_LOGW(TAG, "board total error code 0x0023 = 0x%02X", err);
    }
    ESP_LOGW(TAG, "=== discovery done ===");
}

static void haptic_uart_task(void *arg)
{
    haptic_ctx_t *ctx = arg;
    const int poll_hz = ctx->cfg.poll_hz > 0 ? ctx->cfg.poll_hz : 50;
    const TickType_t period = pdMS_TO_TICKS((1000 + poll_hz - 1) / poll_hz);
    TickType_t next_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "polling ring module %u and little module %u at %d Hz each",
             ctx->modules[0], ctx->modules[1], poll_hz);

    discover(ctx);

    for (;;) {
        bool calibrate = false;
        bool acquisition = false;
        portENTER_CRITICAL(&s_control_mux);
        calibrate = ctx->calibrate_pending;
        ctx->calibrate_pending = false;
        acquisition = ctx->acquisition_enabled;
        portEXIT_CRITICAL(&s_control_mux);

        if (calibrate) {
            bool ok = calibrate_all(ctx);
            ESP_LOGI(TAG, "tare: %s", ok ? "OK" : "FAILED");
            portENTER_CRITICAL(&s_control_mux);
            ctx->tare_ok = ok;
            ctx->tare_count++;
            portEXIT_CRITICAL(&s_control_mux);
        }

        if (!acquisition) {
            next_wake = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        (void)poll_sensor(ctx, 0, ctx->modules[0]);
        (void)poll_sensor(ctx, 1, ctx->modules[1]);
        xTaskDelayUntil(&next_wake, period > 0 ? period : 1);
    }
}

bool haptic_uart_set_acquisition(bool enable)
{
    if (!s_ctx) {
        return false;
    }
    portENTER_CRITICAL(&s_control_mux);
    s_ctx->acquisition_enabled = enable;
    portEXIT_CRITICAL(&s_control_mux);
    return true;
}

bool haptic_uart_acquisition_enabled(void)
{
    if (!s_ctx) {
        return false;
    }
    portENTER_CRITICAL(&s_control_mux);
    bool enabled = s_ctx->acquisition_enabled;
    portEXIT_CRITICAL(&s_control_mux);
    return enabled;
}

bool haptic_uart_calibrate(void)
{
    if (!s_ctx) {
        return false;
    }
    portENTER_CRITICAL(&s_control_mux);
    s_ctx->calibrate_pending = true;
    portEXIT_CRITICAL(&s_control_mux);
    return true;
}

uint32_t haptic_uart_tare_count(bool *last_ok)
{
    if (!s_ctx) {
        if (last_ok) *last_ok = false;
        return 0;
    }
    portENTER_CRITICAL(&s_control_mux);
    uint32_t count = s_ctx->tare_count;
    bool ok = s_ctx->tare_ok;
    portEXIT_CRITICAL(&s_control_mux);
    if (last_ok) *last_ok = ok;
    return count;
}

int haptic_uart_start(const haptic_uart_cfg_t *cfg, const app_bus_t *bus)
{
    if (!cfg || !bus || !bus->haptic_q ||
        cfg->ring_module >= BOX_FINGER_MODULES ||
        cfg->little_module >= BOX_FINGER_MODULES ||
        cfg->ring_module == cfg->little_module) {
        return ESP_ERR_INVALID_ARG;
    }

    haptic_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }
    ctx->cfg = *cfg;
    ctx->bus = bus;
    for (size_t i = 0; i < HAPTIC_SENSOR_COUNT; i++) {
        ctx->slot_status[i] = HAPTIC_SLOT_UNKNOWN;
    }
    ctx->modules[0] = cfg->ring_module;
    ctx->modules[1] = cfg->little_module;
    ctx->acquisition_enabled = true; /* board -> ESP on; USB stream opt-in */

    const uart_config_t uart_cfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(cfg->uart_port, cfg->rx_buf_size,
                                        cfg->tx_buf_size, 0, NULL, 0);
    if (err == ESP_OK) err = uart_param_config(cfg->uart_port, &uart_cfg);
    if (err == ESP_OK) {
        err = uart_set_pin(cfg->uart_port, cfg->tx_gpio, cfg->rx_gpio,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        free(ctx);
        return err;
    }

    s_ctx = ctx;
    uint32_t stack = cfg->task_stack > 0 ? (uint32_t)cfg->task_stack : 4096;
    UBaseType_t prio = cfg->task_prio > 0 ? (UBaseType_t)cfg->task_prio : 6;
    if (xTaskCreate(haptic_uart_task, "haptic_uart", stack, ctx, prio, NULL) != pdPASS) {
        s_ctx = NULL;
        uart_driver_delete(cfg->uart_port);
        free(ctx);
        return ESP_FAIL;
    }
    return ESP_OK;
}
