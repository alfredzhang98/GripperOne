#include "imu_uart.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#include "wit_reg.h"   /* WIT frame layout, type bytes, register map + values */

static const char *TAG = "imu_uart";

/* 11-byte output frame: [0]=0x55 [1]=type [2..9]=4x int16 LE [10]=checksum.
 * All frame/type/register names are in wit_reg.h. */
#define UART_READ_CHUNK  128
#define PARSE_BUF_SIZE   256   /* must be >= UART_READ_CHUNK; holds several 11-byte frames */

typedef struct {
    imu_uart_cfg_t   cfg;
    const app_bus_t *bus;
    uint8_t  acc[PARSE_BUF_SIZE];
    size_t   acc_len;
    imu_sample_t cur;   /* accumulates fields across angle/quat frames */
} imu_ctx_t;

/* Single instance; retained so the public config helpers can reach the UART. */
static int s_imu_port = -1;

/* Pending calibration request: set by the caller (supervisor), executed by the
 * IMU task. WIT calibrations need second-scale settle times, so they must not
 * run in the caller's context. */
typedef enum { CAL_NONE = 0, CAL_ACCEL, CAL_ANGLE_REF, CAL_YAWZERO } cal_req_t;
static volatile cal_req_t s_cal_req = CAL_NONE;
/* Pending baud change: 0 = none, else the target BAUD code. Also run by the IMU
 * task (it must switch this UART's own baud mid-sequence). */
static volatile uint8_t s_baud_req = 0;
/* Algorithm mode we last set (default 9-axis = JY901P factory default). */
static volatile bool s_six_axis = false;

/* Register-read handshake: caller sets s_read_addr and waits; the IMU task fills
 * s_read_vals[] from the 0x5F response and sets s_read_done. */
static volatile int32_t  s_read_addr = -1;
static volatile bool     s_read_done = false;
static volatile uint16_t s_read_vals[4];

/* WIT config frame is 5 bytes: 0xFF 0xAA <addr> <dataL> <dataH>. A setting is
 * unlock (KEY = 0xB588) -> write reg -> (optional) SAVE; auto-relocks after ~10s.
 * Register/value names live in wit_reg.h.
 *
 * What we want the unit to stream, and how fast. Orientation only (angle +
 * quaternion) keeps 9600 baud uncongested: 2 frames x 11 B x 20 Hz = 440 B/s. */
#define IMU_RSW_CONTENT  (IMU_RSW_ACC | IMU_RSW_GYRO | IMU_RSW_ANGLE)   /* 0x0E */
#define IMU_OUT_RATE     IMU_RRATE_20HZ                                 /* 20 Hz */

/* Bring-up aid: print the decoded orientation to the USB console at ~2 Hz so the
 * IMU link can be verified without any host tool. Set to 0 to silence. */
#ifndef IMU_DEBUG_LOG
#define IMU_DEBUG_LOG    1
#endif

static void imu_send_cmd(uint8_t addr, uint8_t dl, uint8_t dh)
{
    if (s_imu_port < 0) return;
    const uint8_t f[5] = {0xFF, 0xAA, addr, dl, dh};
    uart_write_bytes(s_imu_port, (const char *)f, sizeof(f));
}

static void imu_unlock(void) { imu_send_cmd(IMU_REG_KEY, 0x88, 0xB5); }
static void imu_save(void)   { imu_send_cmd(IMU_REG_SAVE, 0x00, 0x00); }

/* Low-level register write: unlock -> write -> optional SAVE. Unlock is handled
 * here (the WIT unit auto-relocks after ~10 s idle), so callers never manage it.
 * All the named helpers below are thin wrappers over this. */
void imu_uart_write_reg(uint8_t addr, uint16_t val, bool save)
{
    if (s_imu_port < 0) return;
    imu_unlock();                                            vTaskDelay(pdMS_TO_TICKS(5));
    imu_send_cmd(addr, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8));
    vTaskDelay(pdMS_TO_TICKS(5));
    if (save) { imu_save(); vTaskDelay(pdMS_TO_TICKS(5)); }
}

/* Set which frames the unit outputs and at what rate. Session-only (no SAVE) so
 * we don't wear the IMU's flash every boot; RSW/RRATE take effect immediately. */
static void imu_config_output(uint16_t rsw, uint8_t rrate)
{
    imu_uart_write_reg(IMU_REG_RSW, rsw, false);
    imu_uart_write_reg(IMU_REG_RRATE, rrate, false);
}

/* Run a WIT calibration sequence. Blocks the IMU task (not the caller), per the
 * WIT manual timing: 200 ms after unlock, a mode-specific settle, an optional
 * exit-to-normal, then save. */
static void run_cal(uint8_t mode, uint32_t settle_ms, bool exit_after, bool save)
{
    imu_unlock();                              vTaskDelay(pdMS_TO_TICKS(200));
    imu_send_cmd(IMU_REG_CALSW, mode, 0x00);   vTaskDelay(pdMS_TO_TICKS(settle_ms));
    if (exit_after) {
        imu_send_cmd(IMU_REG_CALSW, IMU_CALSW_NORMAL, 0x00);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (save) { imu_save(); vTaskDelay(pdMS_TO_TICKS(10)); }
}

/* Map a WIT BAUD code to its numeric baud rate; 0 if unknown. */
static int wit_baud_of(uint8_t code)
{
    switch (code) {
    case IMU_BAUD_4800:   return 4800;
    case IMU_BAUD_9600:   return 9600;
    case IMU_BAUD_19200:  return 19200;
    case IMU_BAUD_38400:  return 38400;
    case IMU_BAUD_57600:  return 57600;
    case IMU_BAUD_115200: return 115200;
    case IMU_BAUD_230400: return 230400;
    case IMU_BAUD_460800: return 460800;
    case IMU_BAUD_921600: return 921600;
    default:              return 0;
    }
}

/* --- calibration actions --- */
/* Accel calibration & angle-reference are timed sequences: queue a request here
 * (non-blocking); the IMU task runs it. Accel needs the module face-up/level. */
void imu_uart_cal_accel(void)     { s_cal_req = CAL_ACCEL; }
void imu_uart_set_angle_ref(void) { s_cal_req = CAL_ANGLE_REF; }
/* Yaw / Z-axis zero (CALSW=0x04): timed sequence, run by the IMU task. Only valid
 * under the 6-axis algorithm — refused (with a hint) in 9-axis mode. */
void imu_uart_set_zero(void)
{
    if (!s_six_axis) {
        ESP_LOGW(TAG, "set_zero needs 6-axis: call imu_uart_set_axis(true) first");
        return;
    }
    s_cal_req = CAL_YAWZERO;
}

/* --- algorithm / mounting --- */
/* Switch the fusion algorithm: 6-axis (accel+gyro, relative yaw, immune to
 * magnetic interference) vs 9-axis (adds mag, absolute yaw). Persisted. */
void imu_uart_set_axis(bool six_axis)
{
    s_six_axis = six_axis;
    imu_uart_write_reg(IMU_REG_AXIS6, six_axis ? 1 : 0, true);
}
bool imu_uart_is_six_axis(void) { return s_six_axis; }
void imu_uart_set_orient(bool vertical) { imu_uart_write_reg(IMU_REG_ORIENT, vertical ? 1 : 0, true); }
/* Enter magnetic calibration; rotate through all orientations, then cal_end(). */
void imu_uart_cal_mag(void)       { imu_uart_write_reg(IMU_REG_CALSW, IMU_CALSW_MAG_SPH, false); }
/* Exit calibration mode and persist. */
void imu_uart_cal_end(void)       { imu_uart_write_reg(IMU_REG_CALSW, IMU_CALSW_NORMAL, true);  }

/* --- output / link configuration (persisted) --- */
void imu_uart_set_content(uint16_t rsw) { imu_uart_write_reg(IMU_REG_RSW,   rsw,  true); }
void imu_uart_set_rate(uint8_t code)    { imu_uart_write_reg(IMU_REG_RRATE, code, true); }
/* Baud change is a timed sequence that must switch this UART's own baud in the
 * middle, so it is queued and run by the IMU task (see the task loop). */
void imu_uart_set_baud(uint8_t code)    { s_baud_req = code; }

/* --- system (SAVE register 0x00; unlock+save primitives stay static/internal) --- */
void imu_uart_reboot(void)        { imu_uart_write_reg(IMU_REG_SAVE, 0x00FF, false); }
void imu_uart_factory_reset(void) { imu_uart_write_reg(IMU_REG_SAVE, 0x0001, false); }

/* --- filtering / bandwidth (persisted) --- */
void imu_uart_set_bandwidth(uint8_t code) { imu_uart_write_reg(IMU_REG_BANDWIDTH, code, true); }
void imu_uart_set_filtk(uint16_t k)       { imu_uart_write_reg(IMU_REG_FILTK,   k, true); }
void imu_uart_set_accfilt(uint16_t v)     { imu_uart_write_reg(IMU_REG_ACCFILT,  v, true); }

/* --- power / misc --- */
void imu_uart_set_powon_output(bool on) { imu_uart_write_reg(IMU_REG_POWONSEND, on ? 1 : 0, true); }
void imu_uart_sleep(bool enable)        { imu_uart_write_reg(IMU_REG_SLEEP, enable ? 1 : 0, false); }
void imu_uart_led(bool on)              { imu_uart_write_reg(IMU_REG_LEDOFF, on ? 0 : 1, true); } /* LEDOFF:1=off */

/* --- gyro calibration --- */
/* Auto gyro-zeroing on stillness (0x61: 0=on default, 1=off). */
void imu_uart_gyro_autocal(bool enable) { imu_uart_write_reg(IMU_REG_GYROCALITHR, enable ? 0 : 1, true); }
/* Manual gyro cal (disable auto first): begin acquiring zero bias while still... */
void imu_uart_gyro_cal_begin(void) { imu_uart_write_reg(IMU_REG_CALSW, IMU_CALSW_GYRO_BEG, false); }
/* ...hold still a while, then end (persists) before moving. */
void imu_uart_gyro_cal_end(void)   { imu_uart_write_reg(IMU_REG_CALSW, IMU_CALSW_GYRO_END, true); }

/* --- register read (no unlock needed; call from a task OTHER than the IMU task) ---
 * Read up to 4 consecutive registers from addr: send READADDR (0x27), wait for the
 * 0x5F response. Returns 0 on success, -1 on timeout. Blocks the caller ~<0.2 s. */
int imu_uart_read_reg(uint8_t addr, uint16_t *out, int count)
{
    if (s_imu_port < 0 || !out || count < 1) return -1;
    s_read_done = false;
    s_read_addr = addr;
    imu_send_cmd(IMU_REG_READADDR, addr, 0x00);   /* FF AA 27 addr 00 */
    for (int i = 0; i < 20 && !s_read_done; i++) vTaskDelay(pdMS_TO_TICKS(10));
    int rc = s_read_done ? 0 : -1;
    if (rc == 0) for (int k = 0; k < count && k < 4; k++) out[k] = s_read_vals[k];
    s_read_addr = -1;
    return rc;
}

/* Read the real algorithm mode from the unit (AXIS6) and cache it; true = 6-axis.
 * On read failure returns the last-known cached value. */
bool imu_uart_refresh_axis(void)
{
    uint16_t v[4];
    if (imu_uart_read_reg(IMU_REG_AXIS6, v, 1) == 0) s_six_axis = (v[0] & 1) != 0;
    return s_six_axis;
}

static inline int16_t rd_i16le(const uint8_t *p)
{
    return (int16_t)(p[0] | (p[1] << 8));
}

/* Remove gravity to leave linear (motion/vibration) acceleration. Gravity is the
 * slow/DC part of accel, so we low-pass the raw accel to estimate the gravity
 * vector — this auto-tracks the actual gravity direction in ANY orientation, with
 * no sign/trig/convention assumptions — and subtract it. At rest -> lin ~ 0; only
 * motion/impacts show. A slow tilt is absorbed within ~2 s (that isn't vibration). */
static void imu_remove_gravity(imu_sample_t *s)
{
    static float grav[3];
    static bool  init = false;
    const float  A = 0.98f;   /* low-pass; ~2.5 s time constant at 20 Hz */
    if (!init) { grav[0] = s->accel[0]; grav[1] = s->accel[1]; grav[2] = s->accel[2]; init = true; }
    for (int i = 0; i < 3; i++) {
        grav[i] = A * grav[i] + (1.0f - A) * s->accel[i];
        s->lin_accel[i] = s->accel[i] - grav[i];
    }
}

/* Decode one validated 11-byte frame into ctx->cur. ACC/GYRO update fields;
 * ANGLE is last in the output burst, so publish the coherent sample on it.
 * Returns true only for the frame that should trigger a publish. */
static bool imu_decode(imu_ctx_t *ctx, const uint8_t *f)
{
    switch (f[1]) {
    case IMU_TYPE_ACC:  /* AxL AxH AyL AyH AzL AzH TL TH */
        ctx->cur.accel[0] = rd_i16le(&f[2]) / 32768.0f * 16.0f;
        ctx->cur.accel[1] = rd_i16le(&f[4]) / 32768.0f * 16.0f;
        ctx->cur.accel[2] = rd_i16le(&f[6]) / 32768.0f * 16.0f;
        ctx->cur.temp     = rd_i16le(&f[8]) / 100.0f;
        return false;
    case IMU_TYPE_GYRO: /* WxL WxH WyL WyH WzL WzH VolL VolH */
        ctx->cur.gyro[0] = rd_i16le(&f[2]) / 32768.0f * 2000.0f;
        ctx->cur.gyro[1] = rd_i16le(&f[4]) / 32768.0f * 2000.0f;
        ctx->cur.gyro[2] = rd_i16le(&f[6]) / 32768.0f * 2000.0f;
        return false;
    case IMU_TYPE_ANGLE: /* RollL RollH PitchL PitchH YawL YawH ... (yaw ignored) */
        ctx->cur.roll  = rd_i16le(&f[2]) / 32768.0f * 180.0f;
        ctx->cur.pitch = rd_i16le(&f[4]) / 32768.0f * 180.0f;
        imu_remove_gravity(&ctx->cur);   /* fresh roll/pitch -> update linear accel */
        return true;
    case IMU_TYPE_READ: /* 0x5F register-read response: 4 regs from the query addr */
        if (s_read_addr >= 0 && !s_read_done) {
            for (int k = 0; k < 4; k++) s_read_vals[k] = (uint16_t)rd_i16le(&f[2 + k * 2]);
            s_read_done = true;
        }
        return false;
    default:
        return false;
    }
}

static void imu_consume(imu_ctx_t *ctx, const uint8_t *data, size_t len)
{
    /* Never copy more than the buffer can hold. */
    if (len > PARSE_BUF_SIZE) {
        data += len - PARSE_BUF_SIZE;
        len = PARSE_BUF_SIZE;
    }
    /* Drop just enough oldest bytes to fit the new data (bounded, no overflow). */
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

    ESP_LOGI(TAG, "imu_uart_task on UART%d @ %d baud", ctx->cfg.uart_port,
             ctx->cfg.baud_rate);

    /* The JY901P defaults to accel+gyro+angle+mag (RSW 0x1E) at 10 Hz. Configure
     * it for what we use: accel (0x51) + gyro (0x52) + angle (0x53), at our rate.
     * Session-only, so the unit's saved config is left untouched. */
    vTaskDelay(pdMS_TO_TICKS(100));   /* let the unit settle after power-up */
    imu_config_output(IMU_RSW_CONTENT, IMU_OUT_RATE);
    ESP_LOGI(TAG, "configured: RSW=0x%04X rate-code=0x%02X", IMU_RSW_CONTENT, IMU_OUT_RATE);

#if IMU_DEBUG_LOG
    TickType_t last_log = xTaskGetTickCount();
#endif
    for (;;) {
        int n = uart_read_bytes(ctx->cfg.uart_port, buf, UART_READ_CHUNK,
                                pdMS_TO_TICKS(50));
        if (n > 0) {
            imu_consume(ctx, buf, (size_t)n);
        }

        /* Run a queued calibration (blocks this task for a few seconds, which is
         * fine — calibration wants the unit held still anyway). */
        if (s_cal_req != CAL_NONE) {
            if (s_cal_req == CAL_ACCEL) {
                ESP_LOGI(TAG, "accel cal: keep the module face-up & still (~4.3 s)...");
                run_cal(IMU_CALSW_ACC, 4000, true, true);
            } else if (s_cal_req == CAL_ANGLE_REF) {
                ESP_LOGI(TAG, "angle-ref: hold still (~3.2 s)...");
                run_cal(IMU_CALSW_ANGLEREF, 3000, false, true);
            } else { /* CAL_YAWZERO (6-axis only) */
                ESP_LOGI(TAG, "yaw zero: hold still (~3.2 s)...");
                run_cal(IMU_CALSW_YAWZERO, 3000, false, true);
            }
            ESP_LOGI(TAG, "calibration done");
            s_cal_req = CAL_NONE;
        }

        /* Run a queued baud change: write BAUD at the old rate (the unit switches
         * immediately), then switch THIS UART to match, then re-unlock + save at
         * the new rate. */
        if (s_baud_req != 0) {
            uint8_t code = s_baud_req;
            int newbaud = wit_baud_of(code);
            if (newbaud > 0) {
                ESP_LOGI(TAG, "baud change -> 0x%02X (%d baud)...", code, newbaud);
                imu_unlock();                                    vTaskDelay(pdMS_TO_TICKS(200));
                imu_send_cmd(IMU_REG_BAUD, code, 0x00);          vTaskDelay(pdMS_TO_TICKS(50));
                uart_set_baudrate(ctx->cfg.uart_port, newbaud);
                uart_flush_input(ctx->cfg.uart_port);            vTaskDelay(pdMS_TO_TICKS(50));
                imu_unlock();                                    vTaskDelay(pdMS_TO_TICKS(200));
                imu_save();                                      vTaskDelay(pdMS_TO_TICKS(10));
                ctx->cfg.baud_rate = newbaud;
                ESP_LOGI(TAG, "baud change done @ %d", newbaud);
            } else {
                ESP_LOGW(TAG, "baud change: unknown code 0x%02X, ignored", code);
            }
            s_baud_req = 0;
        }
#if IMU_DEBUG_LOG
        if (xTaskGetTickCount() - last_log >= pdMS_TO_TICKS(500)) {
            last_log = xTaskGetTickCount();
            ESP_LOGI(TAG, "roll=%.1f pitch=%.1f | a=[%.2f %.2f %.2f]g lin=[%.2f %.2f %.2f]g | w=[%.0f %.0f %.0f]",
                     ctx->cur.roll, ctx->cur.pitch,
                     ctx->cur.accel[0], ctx->cur.accel[1], ctx->cur.accel[2],
                     ctx->cur.lin_accel[0], ctx->cur.lin_accel[1], ctx->cur.lin_accel[2],
                     ctx->cur.gyro[0], ctx->cur.gyro[1], ctx->cur.gyro[2]);
        }
#endif
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
    s_imu_port = cfg->uart_port;

    uint32_t   stack = cfg->task_stack > 0 ? (uint32_t)cfg->task_stack : 4096;
    UBaseType_t prio = cfg->task_prio  > 0 ? (UBaseType_t)cfg->task_prio : 6;
    if (xTaskCreate(imu_uart_task, "imu_uart", stack, ctx, prio, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create imu_uart_task");
        free(ctx);
        return ESP_FAIL;
    }
    return ESP_OK;
}
