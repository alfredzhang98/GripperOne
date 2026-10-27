#include "imu_uart.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#include "wit_reg.h"   /* WIT frame layout, type bytes, register map + values */

static const char *TAG = "imu_uart";

/* 11-byte output frame: [0]=0x55 [1]=type [2..9]=4x int16 LE [10]=checksum.
 * All frame/type/register names are in wit_reg.h. */
#define UART_READ_CHUNK  128
#define PARSE_BUF_SIZE   256   /* must be >= UART_READ_CHUNK; holds several 11-byte frames */

#define IMU_FRESH_ACC    (1U << 0)
#define IMU_FRESH_GYRO   (1U << 1)
#define IMU_FRESH_ANGLE  (1U << 2)
#define IMU_FRESH_ALL    (IMU_FRESH_ACC | IMU_FRESH_GYRO | IMU_FRESH_ANGLE)

typedef struct {
    imu_uart_cfg_t   cfg;
    const app_bus_t *bus;
    uint8_t  acc[PARSE_BUF_SIZE];
    size_t   acc_len;
    imu_sample_t cur;   /* accumulates fields across angle/quat frames */
    uint8_t  fresh;     /* vendor-SDK-style independent frame update bits */
    float    gravity[3];
    bool     gravity_valid;
    int      rate_hz;
    int      current_baud;
    TickType_t last_valid_tick;
} imu_ctx_t;

/* Single instance; retained so the public config helpers can reach the UART. */
static int s_imu_port = -1;
/* Baud the link actually settled on, so a run-time rate change can re-check
 * the stream against it without reaching into the task's context. */
static int s_baud_now = 0;

/* Every sensor write is serialized through the IMU task. This matters because a
 * WIT transaction is a multi-frame sequence (unlock -> write -> optional save):
 * letting the supervisor and IMU tasks emit those frames independently can
 * splice two valid transactions into one invalid sequence. */
typedef enum {
    IMU_REQ_WRITE_REG = 0,
    IMU_REQ_CAL_ACCEL,
    IMU_REQ_CAL_ANGLE_REF,
    IMU_REQ_CAL_YAWZERO,
    IMU_REQ_SET_RATE,
    IMU_REQ_SET_BAUD,
    IMU_REQ_SET_ACQUISITION,
} imu_req_kind_t;

typedef struct {
    imu_req_kind_t kind;
    uint8_t        addr;
    uint16_t       value;
    bool           save;
} imu_req_t;

#define IMU_REQ_QUEUE_LEN  12
static QueueHandle_t     s_req_q = NULL;
static SemaphoreHandle_t s_tx_mutex = NULL;
static TaskHandle_t      s_imu_task = NULL;
static portMUX_TYPE      s_state_mux = portMUX_INITIALIZER_UNLOCKED;
static unsigned          s_cal_pending = 0;
/* Algorithm mode we last set (default 9-axis = JY901P factory default). */
static volatile bool s_six_axis = false;
static volatile bool s_acquisition_enabled = true;

static void imu_finish_calibration_request(void);

/* Register-read handshake: caller sets s_read_addr and waits; the IMU task fills
 * s_read_vals[] from the 0x5F response and sets s_read_done. */
static volatile int32_t  s_read_addr = -1;
static volatile bool     s_read_done = false;
static volatile uint16_t s_read_vals[4];

/* WIT config frame is 5 bytes: 0xFF 0xAA <addr> <dataL> <dataH>. A setting is
 * unlock (KEY = 0xB588) -> write reg -> (optional) SAVE; auto-relocks after ~10s.
 * Register/value names live in wit_reg.h.
 *
 * What we want the unit to stream, and how fast: acceleration, angular rate and
 * angle -> three 11-byte frames per output period. At 100 Hz that is 3300 B/s,
 * or about 29% of what the 115200 8N1 run link carries. Detection at the saved
 * 9600 baud happens before this rate is applied. */
#define IMU_RSW_CONTENT  (IMU_RSW_ACC | IMU_RSW_GYRO | IMU_RSW_ANGLE)   /* 0x0E */
#define IMU_OUT_RATE     IMU_RRATE_100HZ                                /* fallback */

/* Bring-up aid: print the decoded orientation to the USB console at ~2 Hz so the
 * IMU link can be verified without any host tool. Off by default, because every
 * sample is also published to bus->imu_q and whoever consumes that queue prints
 * it: leaving this on puts the same numbers on the console twice, at two
 * unrelated rates. Set to 1 to exercise the decoder with no consumer attached. */
#ifndef IMU_DEBUG_LOG
#define IMU_DEBUG_LOG    0
#endif

static void imu_send_cmd(uint8_t addr, uint8_t dl, uint8_t dh)
{
    if (s_imu_port < 0) return;
    const uint8_t f[5] = {0xFF, 0xAA, addr, dl, dh};
    uart_write_bytes(s_imu_port, (const char *)f, sizeof(f));
}

static void imu_unlock(void) { imu_send_cmd(IMU_REG_KEY, 0x88, 0xB5); }
static void imu_save(void)   { imu_send_cmd(IMU_REG_SAVE, 0x00, 0x00); }

static bool imu_queue_request(const imu_req_t *req)
{
    if (!req || !s_req_q) {
        ESP_LOGW(TAG, "IMU command queue unavailable/full");
        return false;
    }

    const bool is_cal = req->kind == IMU_REQ_CAL_ACCEL ||
                        req->kind == IMU_REQ_CAL_ANGLE_REF ||
                        req->kind == IMU_REQ_CAL_YAWZERO;
    /* Mark calibration busy before publishing the request. The IMU task has a
     * higher priority than the supervisor and may consume it immediately. */
    if (is_cal) {
        portENTER_CRITICAL(&s_state_mux);
        s_cal_pending++;
        portEXIT_CRITICAL(&s_state_mux);
    }

    if (xQueueSend(s_req_q, req, 0) != pdTRUE) {
        if (is_cal) imu_finish_calibration_request();
        ESP_LOGW(TAG, "IMU command queue unavailable/full");
        return false;
    }
    return true;
}

bool imu_uart_is_calibrating(void)
{
    portENTER_CRITICAL(&s_state_mux);
    bool busy = s_cal_pending != 0;
    portEXIT_CRITICAL(&s_state_mux);
    return busy;
}

/* Called only by imu_uart_task while it owns s_tx_mutex. The official WIT
 * normal-protocol SDK leaves 10 ms after unlock and between register writes. */
static void imu_write_reg_now(uint8_t addr, uint16_t val, bool save)
{
    imu_unlock();
    vTaskDelay(pdMS_TO_TICKS(10));
    imu_send_cmd(addr, (uint8_t)(val & 0xFF), (uint8_t)(val >> 8));
    vTaskDelay(pdMS_TO_TICKS(10));
    if (save) {
        imu_save();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Low-level register write: unlock -> write -> optional SAVE. Unlock is handled
 * by the IMU task (the WIT unit auto-relocks after ~10 s idle). Public callers
 * enqueue rather than writing the UART directly, so transactions cannot cross. */
void imu_uart_write_reg(uint8_t addr, uint16_t val, bool save)
{
    const imu_req_t req = {
        .kind = IMU_REQ_WRITE_REG,
        .addr = addr,
        .value = val,
        .save = save,
    };
    (void)imu_queue_request(&req);
}

/* Set which frames the unit outputs and at what rate. Session-only (no SAVE) so
 * we don't wear the IMU's flash every boot; RSW/RRATE take effect immediately. */
static void imu_config_output(uint16_t rsw, uint8_t rrate)
{
    /* Called only after a checksum-valid frame proves the baud is right.
     * Follow the vendor SDK's minimal sequence and touch only the stream
     * content/rate registers. In older JY901 firmware SLEEP (0x22) behaves as
     * a toggle command rather than a dependable absolute-state register, so
     * writing SLEEP=0 here can stop an otherwise healthy stream and make the
     * module restart at its saved 9600-baud setting. Any serial command already
     * wakes a sleeping unit; POWONSEND only controls behaviour after power-on
     * and is likewise unnecessary during a live session. */
    imu_write_reg_now(IMU_REG_RSW, rsw, false);
    imu_write_reg_now(IMU_REG_RRATE, rrate, false);
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

/* Reverse mapping used only for the configured boot -> run transition. This is
 * not a scan: the caller supplies one explicit target baud. */
static uint8_t wit_code_of_baud(int baud)
{
    switch (baud) {
    case 4800:   return IMU_BAUD_4800;
    case 9600:   return IMU_BAUD_9600;
    case 19200:  return IMU_BAUD_19200;
    case 38400:  return IMU_BAUD_38400;
    case 57600:  return IMU_BAUD_57600;
    case 115200: return IMU_BAUD_115200;
    case 230400: return IMU_BAUD_230400;
    case 460800: return IMU_BAUD_460800;
    case 921600: return IMU_BAUD_921600;
    default:     return 0;
    }
}

/* Map a desired output rate in Hz to its RRATE code; 0 if the unit has no such
 * rate. Only the rates worth using are listed — the sub-10 Hz codes exist but
 * nothing here wants them. */
static uint8_t wit_rate_code_of(int hz)
{
    switch (hz) {
    case 10:  return IMU_RRATE_10HZ;
    case 20:  return IMU_RRATE_20HZ;
    case 50:  return IMU_RRATE_50HZ;
    case 100: return IMU_RRATE_100HZ;
    case 200: return IMU_RRATE_200HZ;
    default:  return 0;
    }
}

/* Map a WIT RRATE code to the output rate in centi-hertz (Hz x 100, so the
 * sub-1 Hz codes survive integer math); 0 if the code is not a periodic mode. */
static int wit_rate_chz(uint8_t code)
{
    switch (code) {
    case IMU_RRATE_0_2HZ: return 20;
    case IMU_RRATE_0_5HZ: return 50;
    case IMU_RRATE_1HZ:   return 100;
    case IMU_RRATE_2HZ:   return 200;
    case IMU_RRATE_5HZ:   return 500;
    case IMU_RRATE_10HZ:  return 1000;
    case IMU_RRATE_20HZ:  return 2000;
    case IMU_RRATE_50HZ:  return 5000;
    case IMU_RRATE_100HZ: return 10000;
    case IMU_RRATE_200HZ: return 20000;
    default:              return 0;   /* ONCE / NONE / unknown */
    }
}

/* Report what the configured stream costs, and warn if it does not comfortably
 * fit the link. Each enabled RSW bit costs one 11-byte frame per output period
 * (exact for the ACC/GYRO/ANGLE bits we use; the GPS bits can cost more), and at
 * 8N1 the link carries baud/10 bytes per second. Worth saying out loud because
 * an over-subscribed link drops frames silently, which on the bench is
 * indistinguishable from a loose wire or a failing sensor. */
static void imu_check_bandwidth(int baud, uint16_t rsw, uint8_t rate_code)
{
    int chz = wit_rate_chz(rate_code);
    int cap = baud / 10;
    if (chz <= 0 || cap <= 0) {
        return;
    }
    int frames = __builtin_popcount(rsw);
    int need   = frames * IMU_FRAME_LEN * chz / 100;   /* bytes per second */

    ESP_LOGI(TAG, "stream: %d frame(s) x %d B x %d.%02d Hz = %d B/s; link %d B/s (%d%%)",
             frames, IMU_FRAME_LEN, chz / 100, chz % 100, need, cap, need * 100 / cap);
    if (need * 10 > cap * 8) {
        ESP_LOGW(TAG, "stream needs >80%% of the link: expect dropped frames. "
                      "Raise the baud rate (imu_uart_set_baud) or lower RRATE.");
    }
}

/* --- calibration actions --- */
/* Timed sequences are queued (non-blocking); the IMU task runs them in order. */
bool imu_uart_cal_accel(void)
{
    const imu_req_t req = { .kind = IMU_REQ_CAL_ACCEL };
    return imu_queue_request(&req);
}

bool imu_uart_set_angle_ref(void)
{
    const imu_req_t req = { .kind = IMU_REQ_CAL_ANGLE_REF };
    return imu_queue_request(&req);
}
/* Yaw / Z-axis zero (CALSW=0x04): timed sequence, run by the IMU task. Only valid
 * under the 6-axis algorithm — refused (with a hint) in 9-axis mode. */
bool imu_uart_set_zero(void)
{
    if (!s_six_axis) {
        ESP_LOGW(TAG, "set_zero needs 6-axis: call imu_uart_set_axis(true) first");
        return false;
    }
    const imu_req_t req = { .kind = IMU_REQ_CAL_YAWZERO };
    return imu_queue_request(&req);
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
/* Change the output rate while running. Session-only (save=false) on purpose:
 * imu_uart_cfg_t.rate_hz stays the source of truth across a power cycle, so a
 * live experiment can never quietly become the permanent setting. Re-checks the
 * stream against the link, because raising the rate is exactly how you
 * over-subscribe it. Returns false and changes nothing for an unsupported rate. */
bool imu_uart_set_rate_hz(int hz)
{
    uint8_t code = wit_rate_code_of(hz);
    if (code == 0) {
        ESP_LOGW(TAG, "rate %d Hz not supported (10/20/50/100/200)", hz);
        return false;
    }
    const imu_req_t req = {
        .kind = IMU_REQ_SET_RATE,
        .value = (uint16_t)hz,
    };
    return imu_queue_request(&req);
}

bool imu_uart_set_acquisition(bool enable)
{
    const imu_req_t req = {
        .kind = IMU_REQ_SET_ACQUISITION,
        .value = enable ? 1 : 0,
    };
    return imu_queue_request(&req);
}

bool imu_uart_acquisition_enabled(void)
{
    return s_acquisition_enabled;
}
/* Baud change is a timed sequence that must switch this UART's own baud in the
 * middle, so it is queued and run by the IMU task (see the task loop). */
void imu_uart_set_baud(uint8_t code)
{
    const imu_req_t req = { .kind = IMU_REQ_SET_BAUD, .value = code };
    (void)imu_queue_request(&req);
}

/* --- system (SAVE register 0x00; unlock+save primitives stay static/internal) --- */
void imu_uart_reboot(void)        { imu_uart_write_reg(IMU_REG_SAVE, 0x00FF, false); }
void imu_uart_factory_reset(void) { imu_uart_write_reg(IMU_REG_SAVE, 0x0001, false); }

/* --- filtering / bandwidth (persisted) --- */
void imu_uart_set_bandwidth(uint8_t code) { imu_uart_write_reg(IMU_REG_BANDWIDTH, code, true); }
void imu_uart_set_filtk(uint16_t k)       { imu_uart_write_reg(IMU_REG_FILTK,   k, true); }
void imu_uart_set_accfilt(uint16_t v)     { imu_uart_write_reg(IMU_REG_ACCFILT,  v, true); }

/* --- power / misc --- */
bool imu_uart_set_powon_output(bool on)
{
    const imu_req_t req = {
        .kind = IMU_REQ_WRITE_REG,
        .addr = IMU_REG_POWONSEND,
        .value = on ? 1 : 0,
        .save = true,
    };
    return imu_queue_request(&req);
}

bool imu_uart_ensure_powon_output(bool on)
{
    uint16_t stored = 0;
    if (imu_uart_read_reg(IMU_REG_POWONSEND, &stored, 1) != 0) {
        ESP_LOGW(TAG, "cannot read POWONSEND; will retry startup policy later");
        return false;
    }
    bool current = (stored & 1U) != 0;
    if (current == on) {
        ESP_LOGI(TAG, "POWONSEND already %s; no flash write", on ? "on" : "off");
        return true;
    }
    ESP_LOGI(TAG, "POWONSEND %s -> %s (saving once)",
             current ? "on" : "off", on ? "on" : "off");
    return imu_uart_set_powon_output(on);
}

void imu_uart_sleep(bool enable)        { imu_uart_write_reg(IMU_REG_SLEEP, enable ? 1 : 0, false); }
void imu_uart_led(bool on)              { imu_uart_write_reg(IMU_REG_LEDOFF, on ? 0 : 1, true); } /* LEDOFF:1=off */

/* --- gyro calibration --- */
/* The supplied JY901P protocol page defines register 0x61 as the auto-cal
 * switch for this model: 0 = enabled (factory default), 1 = disabled. */
void imu_uart_gyro_autocal(bool enable)
{
    imu_uart_write_reg(IMU_REG_GYROCALITHR, enable ? 0 : 1, true);
}
/* Manual gyro cal (disable auto first): begin acquiring zero bias while still... */
void imu_uart_gyro_cal_begin(void) { imu_uart_write_reg(IMU_REG_CALSW, IMU_CALSW_GYRO_BEG, false); }
/* ...hold still a while, then end (persists) before moving. */
void imu_uart_gyro_cal_end(void)   { imu_uart_write_reg(IMU_REG_CALSW, IMU_CALSW_GYRO_END, true); }

/* --- register read (no unlock needed; call from a task OTHER than the IMU task) ---
 * Read up to 4 consecutive registers from addr: send READADDR (0x27), wait for the
 * 0x5F response. Returns 0 on success, -1 on timeout. Blocks the caller ~<0.2 s. */
int imu_uart_read_reg(uint8_t addr, uint16_t *out, int count)
{
    if (s_imu_port < 0 || !out || count < 1 || count > 4 || !s_tx_mutex) return -1;
    if (xTaskGetCurrentTaskHandle() == s_imu_task) {
        ESP_LOGE(TAG, "imu_uart_read_reg cannot block the IMU receive task");
        return -1;
    }
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(6000)) != pdTRUE) {
        ESP_LOGW(TAG, "register read timed out waiting for IMU command channel");
        return -1;
    }
    s_read_done = false;
    s_read_addr = addr;
    /* A "read" is really a write to READADDR, so it needs the same unlock as
     * any other register write. The unit auto-relocks after ~10 s idle, so a
     * read issued outside that window is silently dropped and looks exactly
     * like a dead sensor - which is what made the startup POWONSEND check fail
     * on every retry while telemetry kept streaming perfectly. */
    imu_unlock();
    vTaskDelay(pdMS_TO_TICKS(10));
    imu_send_cmd(IMU_REG_READADDR, addr, 0x00);   /* FF AA 27 addr 00 */
    for (int i = 0; i < 20 && !s_read_done; i++) vTaskDelay(pdMS_TO_TICKS(10));
    int rc = s_read_done ? 0 : -1;
    if (rc == 0) for (int k = 0; k < count; k++) out[k] = s_read_vals[k];
    s_read_addr = -1;
    xSemaphoreGive(s_tx_mutex);
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

/* High-pass acceleration for motion/vibration sensing. This estimates the slow
 * gravity/DC component in SENSOR coordinates and subtracts it; it is not a
 * world-frame acceleration transform. The 2.5 s time constant stays fixed when
 * RRATE changes. */
#define GRAVITY_LP_TAU_S  2.5f
static void imu_remove_gravity(imu_ctx_t *ctx, imu_sample_t *s)
{
    int hz = ctx->rate_hz > 0 ? ctx->rate_hz : 20;
    const float a = expf(-1.0f / ((float)hz * GRAVITY_LP_TAU_S));
    if (!ctx->gravity_valid) {
        for (int i = 0; i < 3; i++) ctx->gravity[i] = s->accel[i];
        ctx->gravity_valid = true;
    }
    for (int i = 0; i < 3; i++) {
        ctx->gravity[i] = a * ctx->gravity[i] + (1.0f - a) * s->accel[i];
        s->lin_accel[i] = s->accel[i] - ctx->gravity[i];
    }
}

/* Decode one validated 11-byte frame into ctx->cur. Like the vendor ESP32 SDK,
 * track ACC/GYRO/ANGLE as independent update bits: requiring a rigid frame order
 * can suppress every console sample on a valid stream that starts mid-cycle or
 * contains an additional enabled frame. Returns true once all three are fresh. */
static bool imu_decode(imu_ctx_t *ctx, const uint8_t *f)
{
    switch (f[1]) {
    case IMU_TYPE_ACC:  /* AxL AxH AyL AyH AzL AzH TL TH */
        ctx->cur.accel[0] = rd_i16le(&f[2]) / 32768.0f * 16.0f;
        ctx->cur.accel[1] = rd_i16le(&f[4]) / 32768.0f * 16.0f;
        ctx->cur.accel[2] = rd_i16le(&f[6]) / 32768.0f * 16.0f;
        ctx->cur.temp     = rd_i16le(&f[8]) / 100.0f;
        ctx->fresh |= IMU_FRESH_ACC;
        break;
    case IMU_TYPE_GYRO: /* WxL WxH WyL WyH WzL WzH VolL VolH */
        ctx->cur.gyro[0] = rd_i16le(&f[2]) / 32768.0f * 2000.0f;
        ctx->cur.gyro[1] = rd_i16le(&f[4]) / 32768.0f * 2000.0f;
        ctx->cur.gyro[2] = rd_i16le(&f[6]) / 32768.0f * 2000.0f;
        ctx->fresh |= IMU_FRESH_GYRO;
        break;
    case IMU_TYPE_ANGLE: /* RollL RollH PitchL PitchH YawL YawH ... (yaw ignored) */
        ctx->cur.roll  = rd_i16le(&f[2]) / 32768.0f * 180.0f;
        ctx->cur.pitch = rd_i16le(&f[4]) / 32768.0f * 180.0f;
        ctx->fresh |= IMU_FRESH_ANGLE;
        break;
    case IMU_TYPE_READ: /* 0x5F register-read response: 4 regs from the query addr */
        if (s_read_addr >= 0 && !s_read_done) {
            for (int k = 0; k < 4; k++) s_read_vals[k] = (uint16_t)rd_i16le(&f[2 + k * 2]);
            s_read_done = true;
        }
        return false;
    default:
        return false;
    }

    if ((ctx->fresh & IMU_FRESH_ALL) != IMU_FRESH_ALL) {
        return false;
    }
    ctx->fresh = 0;
    imu_remove_gravity(ctx, &ctx->cur);
    return true;
}

static bool imu_consume(imu_ctx_t *ctx, const uint8_t *data, size_t len)
{
    bool any_valid = false;

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
        any_valid = true;
        ctx->last_valid_tick = xTaskGetTickCount();
        if (imu_decode(ctx, f)) {
            static uint32_t seq = 0;
            ctx->cur.seq = ++seq;
            ctx->cur.timestamp_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
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
    return any_valid;
}

/* Move the unit AND this UART to the baud named by @p code: write BAUD at the
 * old rate and follow it over. Runtime commands may persist the setting; the
 * automatic boot migration deliberately does not, so a full power cycle starts
 * from the documented 9600 default again. Returns the new baud, 0 on failure. */
static int imu_apply_baud(imu_ctx_t *ctx, uint8_t code, bool persist)
{
    int newbaud = wit_baud_of(code);
    if (newbaud <= 0) {
        ESP_LOGW(TAG, "baud change: unknown code 0x%02X, ignored", code);
        return 0;
    }
    /* Refuse the rates this part does not have. A code the unit does not
     * understand can leave the live link unreachable. */
    if (code == IMU_BAUD_460800 || code == IMU_BAUD_921600) {
        ESP_LOGE(TAG, "%d baud is WT931/JY931/HWT606/HWT906 only; refusing to "
                      "write it to a JY901P. Use 230400 or below.", newbaud);
        return 0;
    }
    ESP_LOGI(TAG, "baud change -> 0x%02X (%d baud)...", code, newbaud);
    imu_unlock();                                    vTaskDelay(pdMS_TO_TICKS(10));
    imu_send_cmd(IMU_REG_BAUD, code, 0x00);          vTaskDelay(pdMS_TO_TICKS(500));
    uart_set_baudrate(ctx->cfg.uart_port, newbaud);
    uart_flush_input(ctx->cfg.uart_port);            vTaskDelay(pdMS_TO_TICKS(50));
    if (persist) {
        imu_unlock();                                vTaskDelay(pdMS_TO_TICKS(10));
        imu_save();                                  vTaskDelay(pdMS_TO_TICKS(10));
    }
    ctx->current_baud = newbaud;
    s_baud_now = newbaud;
    ESP_LOGI(TAG, "baud change done @ %d (%s)", newbaud,
             persist ? "saved" : "session only");
    return newbaud;
}

/* What a probe saw, so a failure can say something useful instead of "nothing
 * worked": the raw byte count separates "nothing is transmitting" (power,
 * wiring, wrong RX pin) from "something is, but we cannot sync to it" (wrong
 * baud, wrong protocol, noise), and the captured head bytes let a human read
 * the stream by eye. */
#define IMU_PROBE_KEEP  44
#define IMU_STREAM_STALE_MS  500

typedef struct {
    int     bytes;                      /* raw bytes received during the probe */
    uint8_t first[IMU_PROBE_KEEP];      /* the first of them, verbatim         */
    int     first_len;
} imu_probe_result_t;

static bool imu_probe_type_valid(uint8_t type)
{
    return (type >= IMU_TYPE_TIME && type <= IMU_TYPE_GSA) ||
           type == IMU_TYPE_READ;
}

/* Ask for the read-only VERSION register, then listen for up to @p ms and report
 * whether any checksum-valid frame arrives. The official SDK uses the same kind
 * of read to identify a sensor; it also wakes a unit whose SLEEP bit was set,
 * without unlocking, writing, or saving configuration.
 * Kept independent of imu_consume/ctx on purpose: this runs before the link is
 * trusted, so it must not disturb parser state or publish half-decoded samples.
 *
 * Resyncs one byte at a time, exactly like imu_consume(). An earlier version
 * restarted from the next 0x55 after a bad checksum, discarding the ten bytes
 * it had buffered - which in a short probe window can swallow the very frame
 * head it is looking for, since 0x55 also occurs inside payloads. */
static bool imu_probe(int port, int ms, imu_probe_result_t *res)
{
    uint8_t    acc[256];
    size_t     acc_len  = 0;
    uint8_t    b[64];
    bool       found    = false;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(ms);

    if (res) {
        res->bytes = 0;
        res->first_len = 0;
    }
    uart_flush_input(port);
    imu_send_cmd(IMU_REG_READADDR, IMU_REG_VERSION, 0x00);

    while (!found && (int)(xTaskGetTickCount() - deadline) < 0) {
        int n = uart_read_bytes(port, b, sizeof(b), pdMS_TO_TICKS(20));
        if (n <= 0) {
            continue;
        }
        if (res) {
            res->bytes += n;
            for (int i = 0; i < n && res->first_len < IMU_PROBE_KEEP; i++) {
                res->first[res->first_len++] = b[i];
            }
        }

        /* Append, dropping the oldest bytes if the window would overflow. */
        int copy = n;
        const uint8_t *src = b;
        if (copy > (int)sizeof(acc)) {
            src  = b + (copy - (int)sizeof(acc));
            copy = (int)sizeof(acc);
        }
        if (acc_len + (size_t)copy > sizeof(acc)) {
            size_t drop = acc_len + (size_t)copy - sizeof(acc);
            memmove(acc, acc + drop, acc_len - drop);
            acc_len -= drop;
        }
        memcpy(acc + acc_len, src, (size_t)copy);
        acc_len += (size_t)copy;

        for (size_t i = 0; i + IMU_FRAME_LEN <= acc_len; i++) {
            if (acc[i] != IMU_FRAME_HEAD) {
                continue;
            }
            uint8_t sum = 0;
            for (int k = 0; k < 10; k++) {
                sum += acc[i + k];
            }
            if (imu_probe_type_valid(acc[i + 1]) && sum == acc[i + 10]) {
                found = true;
                break;
            }
        }
    }
    return found;
}

/* Print what a failed probe actually received. Reading these bytes by eye is
 * usually faster than any further guessing: WIT frames start 0x55 0x5x, a baud
 * mismatch shows as 0x00/0xFF runs or repeating garbage, and an idle-low line
 * shows as a stream of 0x00. */
static void imu_log_probe_bytes(int baud, const imu_probe_result_t *res)
{
    char line[IMU_PROBE_KEEP * 3 + 1];
    int  k = 0;
    for (int i = 0; i < res->first_len && k + 4 < (int)sizeof(line); i++) {
        k += snprintf(line + k, sizeof(line) - (size_t)k, "%02X ", res->first[i]);
    }
    line[k] = '\0';
    ESP_LOGW(TAG, "  %6d baud: %3d bytes, head: %s", baud, res->bytes, line);
}

/* Check only the two configured states: factory/boot baud and desired run baud.
 * If the module is found at the boot baud, move it to the run baud without SAVE,
 * then verify a valid frame there. This also handles an ESP reset while the IMU
 * stays powered and therefore is already at the run baud. */
static bool imu_link_up(imu_ctx_t *ctx)
{
    const int boot_baud = ctx->cfg.baud_rate;
    const int run_baud = ctx->cfg.run_baud > 0 ? ctx->cfg.run_baud : boot_baud;
    imu_probe_result_t boot_res = {0};
    imu_probe_result_t run_res = {0};

    esp_err_t err = uart_set_baudrate(ctx->cfg.uart_port, boot_baud);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cannot set IMU boot baud %d: %s", boot_baud,
                 esp_err_to_name(err));
        return false;
    }
    ctx->current_baud = boot_baud;
    s_baud_now = boot_baud;

    /* 1.2 s also covers a sensor still returning at only 1 Hz. */
    if (imu_probe(ctx->cfg.uart_port, 1200, &boot_res)) {
        ESP_LOGI(TAG, "valid WIT frame at boot baud %d", boot_baud);
        if (run_baud == boot_baud) {
            ctx->last_valid_tick = xTaskGetTickCount();
            return true;
        }

        uint8_t code = wit_code_of_baud(run_baud);
        if (code == 0) {
            ESP_LOGE(TAG, "configured run baud %d is unsupported", run_baud);
            return false;
        }
        ESP_LOGI(TAG, "identified IMU at %d; switching session to %d",
                 boot_baud, run_baud);
        if (imu_apply_baud(ctx, code, false) <= 0) {
            return false;
        }
        if (imu_probe(ctx->cfg.uart_port, 1200, &run_res)) {
            ctx->last_valid_tick = xTaskGetTickCount();
            ESP_LOGI(TAG, "valid WIT frame after migration to %d baud", run_baud);
            return true;
        }
        ESP_LOGE(TAG, "IMU RX worked at %d, but no valid frame after the %d "
                      "baud command; check ESP TX GPIO%d -> IMU RX",
                 boot_baud, run_baud, ctx->cfg.tx_gpio);
        imu_log_probe_bytes(run_baud, &run_res);
        return false;
    }

    if (run_baud != boot_baud) {
        err = uart_set_baudrate(ctx->cfg.uart_port, run_baud);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "cannot set IMU run baud %d: %s", run_baud,
                     esp_err_to_name(err));
            return false;
        }
        ctx->current_baud = run_baud;
        s_baud_now = run_baud;
        if (imu_probe(ctx->cfg.uart_port, 1200, &run_res)) {
            ctx->last_valid_tick = xTaskGetTickCount();
            ESP_LOGI(TAG, "valid WIT frame: IMU already at run baud %d", run_baud);
            return true;
        }
    }

    if (boot_res.bytes == 0 && run_res.bytes == 0) {
        ESP_LOGE(TAG, "no IMU bytes at %d or %d baud on RX GPIO%d; check IMU TX "
                      "-> GPIO%d and common GND",
                 boot_baud, run_baud, ctx->cfg.rx_gpio, ctx->cfg.rx_gpio);
        return false;
    }

    ESP_LOGE(TAG, "bytes arrived, but no valid 11-byte WIT frame at the only "
                  "two configured baud rates");
    imu_log_probe_bytes(boot_baud, &boot_res);
    if (run_baud != boot_baud) imu_log_probe_bytes(run_baud, &run_res);
    ESP_LOGE(TAG, "expected 55 5x + 8 data bytes + checksum; verify GPIO%d is "
                  "connected to the IMU's TTL TX (idle HIGH)", ctx->cfg.rx_gpio);
    return false;
}

static void imu_finish_calibration_request(void)
{
    portENTER_CRITICAL(&s_state_mux);
    if (s_cal_pending > 0) s_cal_pending--;
    portEXIT_CRITICAL(&s_state_mux);
}

static void imu_process_request(imu_ctx_t *ctx, const imu_req_t *req)
{
    const bool is_cal = req->kind == IMU_REQ_CAL_ACCEL ||
                        req->kind == IMU_REQ_CAL_ANGLE_REF ||
                        req->kind == IMU_REQ_CAL_YAWZERO;

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    switch (req->kind) {
    case IMU_REQ_WRITE_REG:
        imu_write_reg_now(req->addr, req->value, req->save);
        break;

    case IMU_REQ_CAL_ACCEL:
        ESP_LOGI(TAG, "accel cal: keep the module face-up & still (>5 s)...");
        run_cal(IMU_CALSW_ACC, 5200, true, true);
        break;

    case IMU_REQ_CAL_ANGLE_REF:
        ESP_LOGI(TAG, "angle-ref: hold still (~3.3 s)...");
        run_cal(IMU_CALSW_ANGLEREF, 3000, false, true);
        break;

    case IMU_REQ_CAL_YAWZERO:
        ESP_LOGI(TAG, "yaw zero: hold still (~3.3 s)...");
        run_cal(IMU_CALSW_YAWZERO, 3000, false, true);
        break;

    case IMU_REQ_SET_RATE: {
        int hz = (int)req->value;
        uint8_t code = wit_rate_code_of(hz);
        if (code != 0) {
            imu_write_reg_now(IMU_REG_RRATE, code, false);
            ctx->rate_hz = hz;
            imu_check_bandwidth(s_baud_now, IMU_RSW_CONTENT, code);
        }
        break;
    }

    case IMU_REQ_SET_BAUD:
        if (imu_apply_baud(ctx, (uint8_t)req->value, true) > 0) {
            uint8_t code = wit_rate_code_of(ctx->rate_hz);
            if (code != 0) imu_check_bandwidth(s_baud_now, IMU_RSW_CONTENT, code);
        }
        break;

    case IMU_REQ_SET_ACQUISITION: {
        bool enable = req->value != 0;
        if (enable) {
            uint8_t code = wit_rate_code_of(ctx->rate_hz);
            imu_config_output(IMU_RSW_CONTENT, code ? code : IMU_OUT_RATE);
            ctx->last_valid_tick = xTaskGetTickCount();
            ESP_LOGI(TAG, "IMU -> ESP32 acquisition resumed at %d Hz", ctx->rate_hz);
        } else {
            imu_write_reg_now(IMU_REG_RRATE, IMU_RRATE_NONE, false);
            imu_write_reg_now(IMU_REG_RSW, 0, false);
            uart_flush_input(ctx->cfg.uart_port);
            ctx->acc_len = 0;
            ctx->fresh = 0;
            ctx->last_valid_tick = 0;
            ESP_LOGI(TAG, "IMU -> ESP32 acquisition stopped (session only)");
        }
        s_acquisition_enabled = enable;
        break;
    }
    }
    if (is_cal) {
        /* Discard frames accumulated while the task was deliberately sleeping;
         * the next sample must start with fresh ACC/GYRO/ANGLE frames. */
        uart_flush_input(ctx->cfg.uart_port);
        ctx->acc_len = 0;
        ctx->fresh = 0;
        if (req->kind == IMU_REQ_CAL_ACCEL) ctx->gravity_valid = false;
        imu_finish_calibration_request();
        ESP_LOGI(TAG, "calibration done");
    }
    xSemaphoreGive(s_tx_mutex);
}

static void imu_uart_task(void *arg)
{
    imu_ctx_t *ctx = (imu_ctx_t *)arg;
    uint8_t buf[UART_READ_CHUNK];

    ESP_LOGI(TAG, "imu_uart_task UART%d: TX GPIO%d -> IMU RX, RX GPIO%d <- IMU TX",
             ctx->cfg.uart_port, ctx->cfg.tx_gpio, ctx->cfg.rx_gpio);
    ESP_LOGI(TAG, "IMU baud path: boot %d -> run %d (only these two are checked)",
             ctx->cfg.baud_rate,
             ctx->cfg.run_baud > 0 ? ctx->cfg.run_baud : ctx->cfg.baud_rate);

    uint8_t rate = wit_rate_code_of(ctx->cfg.rate_hz);
    if (rate == 0) {
        if (ctx->cfg.rate_hz != 0) {
            ESP_LOGW(TAG, "rate_hz=%d is not a WIT rate (10/20/50/100/200); using 100 Hz",
                     ctx->cfg.rate_hz);
        }
        rate = IMU_OUT_RATE;
    }
    ctx->rate_hz = wit_rate_chz(rate) / 100;
    s_acquisition_enabled = true;

    vTaskDelay(pdMS_TO_TICKS(100));   /* let the unit settle after power-up */
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    bool configured = imu_link_up(ctx);   /* must precede any config write */

    /* The JY901P defaults to accel+gyro+angle+mag (RSW 0x1E) at 10 Hz. Configure
     * it for what we use: accel (0x51) + gyro (0x52) + angle (0x53), at our rate.
     * Session-only, so the unit's saved config is left untouched. */
    if (configured) {
        imu_config_output(IMU_RSW_CONTENT, rate);
        ESP_LOGI(TAG, "configured: RSW=0x%04X rate-code=0x%02X", IMU_RSW_CONTENT, rate);
        imu_check_bandwidth(ctx->current_baud, IMU_RSW_CONTENT, rate);
    }
    xSemaphoreGive(s_tx_mutex);
    TickType_t next_probe = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    bool wake_attempted = false;

#if IMU_DEBUG_LOG
    TickType_t last_log = xTaskGetTickCount();
#endif
    for (;;) {
        int n = uart_read_bytes(ctx->cfg.uart_port, buf, UART_READ_CHUNK,
                                pdMS_TO_TICKS(50));
        if (n > 0) {
            if (imu_consume(ctx, buf, (size_t)n)) {
                wake_attempted = false;
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (s_acquisition_enabled && configured && ctx->last_valid_tick != 0 &&
            now - ctx->last_valid_tick >= pdMS_TO_TICKS(IMU_STREAM_STALE_MS)) {
            if (!wake_attempted) {
                ESP_LOGW(TAG, "valid IMU stream stopped; waking and restoring "
                              "continuous output at %d baud", ctx->current_baud);
                xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
                /* A read is harmless and also wakes a sleeping unit. Restore
                 * only content/rate; do not write the model-dependent SLEEP
                 * command while recovering a healthy UART session. */
                imu_send_cmd(IMU_REG_READADDR, IMU_REG_VERSION, 0x00);
                vTaskDelay(pdMS_TO_TICKS(100));
                uint8_t active_rate = wit_rate_code_of(ctx->rate_hz);
                imu_config_output(IMU_RSW_CONTENT, active_rate ? active_rate : rate);
                xSemaphoreGive(s_tx_mutex);
                wake_attempted = true;
                ctx->last_valid_tick = xTaskGetTickCount(); /* recovery grace */
            } else {
                ESP_LOGW(TAG, "IMU wake did not restore frames; re-detecting at "
                              "%d and %d baud",
                         ctx->cfg.baud_rate,
                         ctx->cfg.run_baud > 0 ? ctx->cfg.run_baud : ctx->cfg.baud_rate);
                configured = false;
                wake_attempted = false;
                next_probe = now;
            }
        }

        if (s_acquisition_enabled && !configured &&
            (int)(xTaskGetTickCount() - next_probe) >= 0) {
            xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
            configured = imu_link_up(ctx);
            if (configured) {
                uint8_t active_rate = wit_rate_code_of(ctx->rate_hz);
                imu_config_output(IMU_RSW_CONTENT, active_rate ? active_rate : rate);
                ESP_LOGI(TAG, "IMU reconnected and configured");
                wake_attempted = false;
            }
            xSemaphoreGive(s_tx_mutex);
            next_probe = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
        }

        imu_req_t req;
        while (xQueueReceive(s_req_q, &req, 0) == pdTRUE) {
            imu_process_request(ctx, &req);
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
    if (!cfg || !bus || !bus->imu_q || cfg->rx_buf_size <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_req_q || s_tx_mutex || s_imu_task) {
        return ESP_ERR_INVALID_STATE;
    }

    imu_ctx_t *ctx = calloc(1, sizeof(imu_ctx_t));
    if (!ctx) {
        return ESP_ERR_NO_MEM;
    }
    ctx->cfg = *cfg;
    ctx->bus = bus;

    s_req_q = xQueueCreate(IMU_REQ_QUEUE_LEN, sizeof(imu_req_t));
    s_tx_mutex = xSemaphoreCreateMutex();
    if (!s_req_q || !s_tx_mutex) {
        if (s_req_q) vQueueDelete(s_req_q);
        if (s_tx_mutex) vSemaphoreDelete(s_tx_mutex);
        s_req_q = NULL;
        s_tx_mutex = NULL;
        free(ctx);
        return ESP_ERR_NO_MEM;
    }

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
    /* UART idles HIGH. Keep a disconnected/loose RX from floating into the
     * repeated 00/FF garbage that looks deceptively like received data. */
    ESP_ERROR_CHECK(gpio_set_pull_mode(cfg->rx_gpio, GPIO_PULLUP_ONLY));
    s_imu_port = cfg->uart_port;
    s_baud_now = cfg->baud_rate;

    uint32_t   stack = cfg->task_stack > 0 ? (uint32_t)cfg->task_stack : 4096;
    UBaseType_t prio = cfg->task_prio  > 0 ? (UBaseType_t)cfg->task_prio : 6;
    if (xTaskCreate(imu_uart_task, "imu_uart", stack, ctx, prio, &s_imu_task) != pdPASS) {
        ESP_LOGE(TAG, "failed to create imu_uart_task");
        uart_driver_delete(cfg->uart_port);
        vQueueDelete(s_req_q);
        vSemaphoreDelete(s_tx_mutex);
        s_req_q = NULL;
        s_tx_mutex = NULL;
        s_imu_port = -1;
        s_baud_now = 0;
        free(ctx);
        return ESP_FAIL;
    }
    return ESP_OK;
}
