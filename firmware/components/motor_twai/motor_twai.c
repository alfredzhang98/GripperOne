#include "motor_twai.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "motor_twai";

/* Damiao DM-J4310-2EC CAN framing (CAN 2.0A standard frame, 11-bit ID).
 *
 * ESC_ID (reg 0x08) = motor's receive ID, factory default 0x01.
 * MST_ID (reg 0x07) = feedback ID the motor transmits on, factory default 0x00.
 * Force-position command frames go to 0x300 + ESC_ID; enable / disable /
 * clear-error go to ESC_ID (the drive matches only the low 8 ID bits, p.12).
 * Register read/write frames go to 0x7FF and are answered on MST_ID. */
#define MOTOR_ESC_ID_DEFAULT    0x001
#define MOTOR_MST_ID_DEFAULT    0x000
#define DM_ID_OFFSET_FORCE_POS  0x300
#define DM_PARAM_FRAME_ID       0x7FF
#define DM_PARAM_CMD_READ       0x33
#define DM_PARAM_CMD_WRITE      0x55
#define DM_PARAM_CMD_SAVE       0xAA

/* Registers used by the probe (manual p.17-18). */
#define REG_ACC         0x04   /* float, trapezoid acceleration, rad/s^2     */
#define REG_DEC         0x05   /* float, trapezoid deceleration, < 0         */
#define REG_KP_APR      0x1B   /* float, position-loop gain                  */
#define REG_TIMEOUT     0x09   /* uint32, 50 us per tick, only while enabled */
#define REG_CTRL_MODE   0x0A   /* uint32, 1..4                               */
#define REG_NPP         0x10   /* uint32, pole pairs                         */
#define REG_FLUX        0x13   /* float, rotor flux                          */
#define REG_GR          0x14   /* float, gear ratio                          */
#define REG_PMAX        0x15   /* float, rad                                 */
#define REG_VMAX        0x16   /* float, rad/s                               */
#define REG_TMAX        0x17   /* float, N*m                                 */
#define REG_GREF        0x1E   /* float, gearbox torque efficiency           */
#define REG_IMAX        0x3B   /* float, A, driver maximum current           */

static char s_loop_regs[256];          /* probe diagnostics, logged at first enable */
static bool s_loop_regs_logged = false;
#define DM_PARAM_CACHE_LEN 0x52

#define CTRL_MODE_FORCE_POS 4u

/* Force-position frame scaling (manual p.14): v_des x100 and i_des x10000,
 * both unsigned 16-bit and clamped to 0..10000 by the drive. */
#define DM_FP_VEL_SCALE    100.0f
#define DM_FP_VEL_MAX_RAW  10000u    /* -> 100 rad/s */
#define DM_FP_CUR_SCALE    10000.0f
#define DM_FP_CUR_MAX_RAW  10000u    /* -> 1.0 pu   */

/* Timing. */
#define CMD_PERIOD_MS          10    /* enabled: one command frame per period      */
#define POLL_PERIOD_MS         50    /* disabled: one status poll per period       */
#define FEEDBACK_LOST_MS       300   /* no feedback this long -> not ready, re-probe */
#define ENABLE_FRESH_MS        150   /* enable() needs feedback at least this new  */
#define ENABLE_GRACE_MS        100   /* after enable, feedback must say "enabled"  */
#define SETPOINT_STALE_MS      150   /* enabled but no new setpoint: disable       */
#define PROBE_RETRY_MS         1000
#define PROBE_ALIVE_WAIT_MS    100
#define PARAM_REPLY_TIMEOUT_MS 50
#define PARAM_ATTEMPTS         3
#define TX_IDLE_WAIT_MS        5     /* one 8-byte frame is ~0.13 ms at 1 Mbit/s  */
#define TIMEOUT_MS_DEFAULT     300
#define DM_TIMEOUT_TICK_US     50

/* Sanity window for the ranges read from the motor. The drive initialises its
 * position in (-pi, pi] at power-up and this gripper's stroke is 6.9 rad
 * (bench 2026-10-22; seeks may travel up to 8 rad), so PMAX >= pi + 8 (11.5)
 * keeps the 16-bit position feedback from wrapping during use. A wrap would
 * still be caught by the gripper's position-jump check (motor off). PMAX <= 25
 * keeps one position step (2*PMAX/65535) at or below 0.76 mrad, which the
 * gripper's motion windows assume. */
#define PMAX_MIN_RAD   11.5f
#define PMAX_MAX_RAD   25.0f
#define RANGE_MAX      1000.0f

/* Damiao well-known 8-byte control frames (sent to CAN ID = ESC_ID). */
static const uint8_t DM_FRAME_ENABLE[8]    = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const uint8_t DM_FRAME_DISABLE[8]   = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const uint8_t DM_FRAME_CLEAR_ERR[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};

static float uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return (float)x * span / (float)((1u << bits) - 1) + x_min;
}

static twai_node_handle_t s_node = NULL;
static twai_onchip_node_config_t s_node_cfg;   /* kept to rebuild the node   */
static const app_bus_t   *s_bus  = NULL;
static volatile bool      s_save_ack = false;  /* 0xAA store reply seen      */

/* Every transmit holds this mutex, so frames leave in the order they were
 * decided and the enable/poll decision cannot interleave with a disable. */
static SemaphoreHandle_t s_tx_mutex = NULL;
/* Guards the setpoint triple. */
static portMUX_TYPE s_sp_lock = portMUX_INITIALIZER_UNLOCKED;

static uint32_t s_esc_id = MOTOR_ESC_ID_DEFAULT;
static uint32_t s_mst_id = MOTOR_MST_ID_DEFAULT;
static int      s_bitrate_kbps = 0;
static uint32_t s_timeout_ticks = 0;

/* Feedback decode ranges. Compiled-in guesses until the probe replaces them
 * with the motor's own registers; nothing acts on feedback before that. */
static volatile float s_pmax = 12.5f;
static volatile float s_vmax = 30.0f;
static volatile float s_tmax = 10.0f;

static volatile bool     s_ready   = false;
static volatile bool     s_enabled = false;
static volatile uint32_t s_epoch   = 0;
static motor_twai_info_t s_info;
static volatile bool     s_have_info = false;

/* Force-position setpoint. s_sp_valid is cleared by disable() so a later
 * enable() cannot replay a setpoint from an earlier session. */
static float s_sp_pos = 0.0f;
static float s_sp_vel = 0.0f;
static float s_sp_cur = 0.0f;
static bool  s_sp_valid = false;

static volatile TickType_t s_last_fb_tick = 0;
static volatile uint32_t   s_fb_count = 0;
static volatile uint8_t    s_last_fb_err = 0;    /* ERR nibble of the newest feedback */
static volatile TickType_t s_enable_tick = 0;    /* when the last enable was sent    */
static volatile TickType_t s_sp_tick = 0;        /* when the setpoint was last set    */

/* Bring-up diagnostics: frames that are not the motor's feedback (another
 * ID, or MST_ID with another motor's ID nibble). The most common first-power-up
 * fault is a motor whose CAN_ID / MST_ID differ from the configured ones. */
static volatile uint32_t s_foreign_n = 0;
static volatile uint32_t s_foreign_id = 0;
static volatile uint8_t  s_foreign_dlc = 0;
static volatile uint8_t  s_foreign_d[8];
/* Bus errors by type since the last probe report (on_error callback). */
static volatile uint32_t s_err_ack = 0, s_err_bit = 0, s_err_form = 0, s_err_stuff = 0, s_err_arb = 0;

static volatile uint32_t s_param_raw[DM_PARAM_CACHE_LEN];
static volatile bool     s_param_valid[DM_PARAM_CACHE_LEN];

static bool param_write_u32_verified(uint8_t rid, uint32_t value);
static void node_rebuild(void);

/* ---- low-level TX -------------------------------------------------------- */

/* Frame storage that outlives motor_tx(). See the comment in motor_tx(). */
#define TX_SLOTS 8
static twai_frame_t s_tx_frame[TX_SLOTS];
static uint8_t      s_tx_buf[TX_SLOTS][8];
static unsigned     s_tx_slot = 0;

/* Caller must hold s_tx_mutex. */
static esp_err_t motor_tx(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    if (!s_node || len > 8) {
        return ESP_ERR_INVALID_STATE;
    }

    /* The ESP-IDF v6 TWAI driver copies a frame into the controller only when
     * the controller is idle. When it is busy, it queues a POINTER to the
     * caller's twai_frame_t and reads it later from the ISR, so a frame built
     * on this function's stack would be read after it is gone. Two measures:
     *  1. wait until the controller is idle, so the transmit below takes the
     *     immediate-copy path (every transmit holds s_tx_mutex, so no other
     *     frame can start in between);
     *  2. build the frame in static storage that stays valid for the next
     *     TX_SLOTS - 1 transmits anyway.
     * If the controller does not go idle (no node ACKs: motor unpowered,
     * transceiver off, wrong bit rate), the frame is not sent at all. */
    /* In bus-off the driver rejects every call and logs an error of its own
     * each time; check the state first and only start a recovery below. */
    twai_node_status_t nst;
    esp_err_t err;
    if (twai_node_get_info(s_node, &nst, NULL) == ESP_OK && nst.state == TWAI_ERROR_BUS_OFF) {
        err = ESP_ERR_INVALID_STATE;
    } else {
        err = twai_node_transmit_wait_all_done(s_node, TX_IDLE_WAIT_MS);
    }
    if (err == ESP_OK) {
        unsigned k = s_tx_slot;
        s_tx_slot = (s_tx_slot + 1u) % TX_SLOTS;
        memcpy(s_tx_buf[k], data, len);
        s_tx_frame[k] = (twai_frame_t){
            .header = { .id = can_id, .dlc = len },
            .buffer = s_tx_buf[k],
            .buffer_len = len,
        };
        err = twai_node_transmit(s_node, &s_tx_frame[k], 0);
    }
    if (err == ESP_OK) {
        return ESP_OK;
    }

    /* A CAN frame must be ACKed by another node. Start a bus-off recovery at
     * most once a second; log at most once every 10 s so a dead bus cannot
     * flood the console (the probe prints the detailed diagnosis). */
    static TickType_t next_recover = 0;
    static TickType_t next_log = 0;
    TickType_t now = xTaskGetTickCount();
    if (err == ESP_ERR_INVALID_STATE && (int32_t)(now - next_recover) >= 0) {   /* wrap-safe */
        next_recover = now + pdMS_TO_TICKS(1000);
        (void)twai_node_recover(s_node);
    }
    if ((int32_t)(now - next_log) < 0) {
        return err;
    }
    next_log = now + pdMS_TO_TICKS(10000);

    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "CAN bus-off (our own frames fail on the bus) - recovering; next note in 10 s");
    } else if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "CAN controller still busy (no ACK on the bus?) - frame 0x%03X not sent",
                 (unsigned)can_id);
    } else {
        ESP_LOGW(TAG, "transmit (id=0x%03X) failed: %s", (unsigned)can_id, esp_err_to_name(err));
    }
    return err;
}

/* Force-position frame, 0x300 + ESC_ID (manual p.14):
 *   D0..D3 = p_des (float LE), D4..D5 = v_des x100 (uint16 LE),
 *   D6..D7 = i_des x10000 (uint16 LE). Caller must hold s_tx_mutex. */
static void send_force_pos(float p, float vel_limit, float cur_limit)
{
    if (vel_limit < 0.0f) vel_limit = 0.0f;
    if (cur_limit < 0.0f) cur_limit = 0.0f;
    /* Truncate (never above the request), but not below it by float error:
     * 3 x 0.0001 x 10000 can come out as 2.9999998. */
    uint32_t vraw = (uint32_t)(vel_limit * DM_FP_VEL_SCALE + 1e-3f);
    uint32_t iraw = (uint32_t)(cur_limit * DM_FP_CUR_SCALE + 1e-3f);
    if (vraw > DM_FP_VEL_MAX_RAW) vraw = DM_FP_VEL_MAX_RAW;
    if (iraw > DM_FP_CUR_MAX_RAW) iraw = DM_FP_CUR_MAX_RAW;

    uint8_t d[8];
    memcpy(&d[0], &p, 4);
    d[4] = vraw & 0xFF;
    d[5] = (vraw >> 8) & 0xFF;
    d[6] = iraw & 0xFF;
    d[7] = (iraw >> 8) & 0xFF;
    (void)motor_tx(DM_ID_OFFSET_FORCE_POS + s_esc_id, d, 8);
}

static bool setpoint_get(float *p, float *v, float *i)
{
    taskENTER_CRITICAL(&s_sp_lock);
    bool valid = s_sp_valid;
    *p = s_sp_pos;
    *v = s_sp_vel;
    *i = s_sp_cur;
    taskEXIT_CRITICAL(&s_sp_lock);
    return valid;
}

static void setpoint_invalidate(void)
{
    taskENTER_CRITICAL(&s_sp_lock);
    s_sp_valid = false;
    s_sp_cur = 0.0f;
    taskEXIT_CRITICAL(&s_sp_lock);
}

/* ---- public control ----------------------------------------------------- */

bool motor_twai_is_ready(void) { return s_ready; }
uint32_t motor_twai_epoch(void) { return s_epoch; }

uint32_t motor_twai_feedback_age_ms(void)
{
    if (s_fb_count == 0) {
        return UINT32_MAX;
    }
    TickType_t now = xTaskGetTickCount();
    return (uint32_t)((now - s_last_fb_tick) * portTICK_PERIOD_MS);
}

bool motor_twai_get_info(motor_twai_info_t *out)
{
    if (!s_have_info || !out) {
        return false;
    }
    *out = s_info;
    return true;
}

void motor_twai_set_force_pos(float pos, float vel_limit, float cur_limit)
{
    if (!isfinite(pos) || !isfinite(vel_limit) || !isfinite(cur_limit)) {
        /* Never forward a NaN/Inf to the drive: keep the old target and drop
         * the current limit to zero instead. */
        ESP_LOGE(TAG, "non-finite setpoint (%f, %f, %f) - current limit forced to 0",
                 pos, vel_limit, cur_limit);
        taskENTER_CRITICAL(&s_sp_lock);
        s_sp_cur = 0.0f;
        taskEXIT_CRITICAL(&s_sp_lock);
        return;
    }
    if (vel_limit < 0.0f) vel_limit = 0.0f;
    if (vel_limit > 100.0f) vel_limit = 100.0f;
    if (cur_limit < 0.0f) cur_limit = 0.0f;
    if (cur_limit > 1.0f) cur_limit = 1.0f;

    taskENTER_CRITICAL(&s_sp_lock);
    s_sp_pos = pos;
    s_sp_vel = vel_limit;
    s_sp_cur = cur_limit;
    s_sp_valid = true;
    s_sp_tick = xTaskGetTickCount();
    taskEXIT_CRITICAL(&s_sp_lock);
}

bool motor_twai_persist_config(void)
{
    if (!s_node || !s_tx_mutex || !s_ready || s_enabled) {
        ESP_LOGW(TAG, "persist refused: motor must be probed and disabled");
        return false;
    }
    /* The probe already set both; write again so the store saves exactly these. */
    if (!param_write_u32_verified(REG_CTRL_MODE, CTRL_MODE_FORCE_POS) ||
        !param_write_u32_verified(REG_TIMEOUT, s_timeout_ticks)) {
        ESP_LOGE(TAG, "persist: register write not confirmed");
        return false;
    }
    s_save_ack = false;
    uint8_t d[4] = { s_esc_id & 0xFF, (s_esc_id >> 8) & 0xFF, DM_PARAM_CMD_SAVE, 0x01 };
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    esp_err_t err = motor_tx(DM_PARAM_FRAME_ID, d, 4);
    xSemaphoreGive(s_tx_mutex);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "persist: store frame not sent");
        return false;
    }
    /* Manual p.16: a store takes up to 30 ms and only works while disabled. */
    TickType_t t0 = xTaskGetTickCount();
    while (!s_save_ack && (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(200)) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (!s_save_ack) {
        ESP_LOGE(TAG, "persist: no reply to the store command");
        return false;
    }
    ESP_LOGW(TAG, "CTRL_MODE %u and TIMEOUT %u ticks saved in the motor's flash "
                  "(about 10000 write cycles in its life - do not repeat routinely)",
             (unsigned)CTRL_MODE_FORCE_POS, (unsigned)s_timeout_ticks);
    return true;
}

void motor_twai_disable(void)
{
    if (!s_tx_mutex) {
        return;
    }
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    bool was_enabled = s_enabled;
    s_enabled = false;
    setpoint_invalidate();
    (void)motor_tx(s_esc_id, DM_FRAME_DISABLE, 8);
    xSemaphoreGive(s_tx_mutex);
    if (was_enabled) {
        ESP_LOGI(TAG, "disabled");
    }
}

void motor_twai_clear_error(void)
{
    if (!s_node || !s_tx_mutex) {
        return;
    }
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    s_enabled = false;            /* the drive is disabled after a fault */
    setpoint_invalidate();
    (void)motor_tx(s_esc_id, DM_FRAME_CLEAR_ERR, 8);
    xSemaphoreGive(s_tx_mutex);
    ESP_LOGI(TAG, "clear-error");
}

/* ---- register access (probe only, motor disabled) ------------------------ */

/* Register frame: D0..D1 = motor CAN_ID (LE), D2 = command, D3 = RID,
 * D4..D7 = 32-bit value (LE). The reply arrives on MST_ID and is cached by the
 * RX callback. */
static bool param_transact(uint8_t cmd, uint8_t rid, uint32_t value, uint32_t *out, int attempts)
{
    if (rid >= DM_PARAM_CACHE_LEN) {
        return false;
    }
    for (int attempt = 0; attempt < attempts; attempt++) {
        s_param_valid[rid] = false;
        uint8_t d[8] = { s_esc_id & 0xFF, (s_esc_id >> 8) & 0xFF, cmd, rid, 0, 0, 0, 0 };
        uint8_t len = 4;
        if (cmd == DM_PARAM_CMD_WRITE) {
            memcpy(&d[4], &value, 4);
            len = 8;
        }
        xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
        esp_err_t err = motor_tx(DM_PARAM_FRAME_ID, d, len);
        xSemaphoreGive(s_tx_mutex);
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(PARAM_REPLY_TIMEOUT_MS));
            continue;
        }
        TickType_t t0 = xTaskGetTickCount();
        while (!s_param_valid[rid] &&
               (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(PARAM_REPLY_TIMEOUT_MS)) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        if (s_param_valid[rid]) {
            uint32_t raw = s_param_raw[rid];
            if (cmd == DM_PARAM_CMD_WRITE && raw != value) {
                continue;   /* echo differs from what we wrote: try again */
            }
            if (out) *out = raw;
            return true;
        }
    }
    return false;
}

static bool param_read_u32(uint8_t rid, uint32_t *out)
{
    return param_transact(DM_PARAM_CMD_READ, rid, 0, out, PARAM_ATTEMPTS);
}

static bool param_read_f32(uint8_t rid, float *out)
{
    uint32_t raw;
    if (!param_transact(DM_PARAM_CMD_READ, rid, 0, &raw, PARAM_ATTEMPTS)) {
        return false;
    }
    memcpy(out, &raw, 4);
    return true;
}

static bool param_write_u32_verified(uint8_t rid, uint32_t value)
{
    uint32_t back = 0;
    return param_transact(DM_PARAM_CMD_WRITE, rid, value, NULL, PARAM_ATTEMPTS) &&
           param_read_u32(rid, &back) && back == value;
}

static bool param_write_f32_verified(uint8_t rid, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    float back = 0.0f;
    return param_transact(DM_PARAM_CMD_WRITE, rid, bits, NULL, PARAM_ATTEMPTS) &&
           param_read_f32(rid, &back) && fabsf(back - value) <= 1e-3f * fabsf(value);
}

/* Before every enable, read back the two registers the probe set. CAN writes
 * are not stored in the drive's flash (manual p.12/p.16): a drive that
 * restarted, even too briefly for feedback to be missed, is back in its flash
 * mode, and a force-position frame would then be decoded with another mode's
 * layout. A mismatch marks the driver "not ready", so the task re-probes and
 * the epoch increments (the gripper then discards its calibration). The reads
 * run before the TX mutex is taken (param_transact takes it per frame).
 * After motor_twai_persist_config() the flash holds the same values, so this
 * check no longer reveals a restart; it then only guards against another mode
 * being written. A restart is still caught by "reports disabled while
 * enabled" and by the gripper's position-jump check. */
static bool registers_still_set(void)
{
    uint32_t mode = 0, timeout = 0;
    if (!param_transact(DM_PARAM_CMD_READ, REG_CTRL_MODE, 0, &mode, 1) ||
        !param_transact(DM_PARAM_CMD_READ, REG_TIMEOUT, 0, &timeout, 1)) {
        ESP_LOGE(TAG, "enable: registers did not answer - re-probing");
        s_ready = false;
        return false;
    }
    if (mode != CTRL_MODE_FORCE_POS || timeout != s_timeout_ticks) {
        ESP_LOGE(TAG, "enable: CTRL_MODE %u (want %u), TIMEOUT %u (want %u) - the drive has "
                      "restarted or been reconfigured; re-probing",
                 (unsigned)mode, (unsigned)CTRL_MODE_FORCE_POS, (unsigned)timeout,
                 (unsigned)s_timeout_ticks);
        s_ready = false;
        return false;
    }
    return true;
}

bool motor_twai_enable(void)
{
    if (!s_node || !s_tx_mutex) {
        return false;
    }
    bool ok = false;
    bool was_enabled = s_enabled;
    const char *why = NULL;

    if (!s_ready) {
        ESP_LOGW(TAG, "enable refused: probe has not succeeded");
        return false;
    }
    if (!was_enabled && !registers_still_set()) {
        return false;
    }

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    float p, v, i;
    if (!s_ready) {
        why = "probe has not succeeded";
    } else if (motor_twai_feedback_age_ms() > ENABLE_FRESH_MS) {
        why = "feedback is stale";
    } else if (!setpoint_get(&p, &v, &i)) {
        why = "no setpoint since the last disable";
    } else {
        /* Setpoint first, so the drive never holds an older command when the
         * enable lands; then enable; then the setpoint again. */
        send_force_pos(p, v, i);
        if (motor_tx(s_esc_id, DM_FRAME_ENABLE, 8) != ESP_OK) {
            why = "enable frame not sent";
        } else {
            s_enabled = true;
            s_enable_tick = xTaskGetTickCount();
            send_force_pos(p, v, i);
            ok = true;
        }
    }
    xSemaphoreGive(s_tx_mutex);

    if (!ok) {
        ESP_LOGW(TAG, "enable refused: %s", why);
    } else if (!was_enabled) {
        ESP_LOGI(TAG, "enabled");
        if (!s_loop_regs_logged && s_loop_regs[0]) {
            /* again here: the boot log is gone before the USB console reconnects */
            s_loop_regs_logged = true;
            ESP_LOGI(TAG, "drive loop registers:%s", s_loop_regs);
        }
    }
    return ok;
}

static bool range_ok(float v, float lo)
{
    return isfinite(v) && v >= lo && v <= RANGE_MAX;
}

/* Why the probe got no answer: the controller's error counters tell whether
 * any node acknowledges our frames, and the foreign-frame record tells whether
 * the motor answers on other IDs. */
static void probe_diagnose(void)
{
    static const char *const state_name[] = { "error-active", "error-warning", "error-passive", "bus-off" };
    twai_node_status_t st = {0};
    twai_node_record_t rec = {0};
    bool have = false;
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    if (s_node) {
        have = twai_node_get_info(s_node, &st, &rec) == ESP_OK;
    }
    xSemaphoreGive(s_tx_mutex);
    const uint32_t e_ack = s_err_ack, e_bit = s_err_bit, e_form = s_err_form, e_stuff = s_err_stuff,
                   e_arb = s_err_arb;
    s_err_ack = s_err_bit = s_err_form = s_err_stuff = s_err_arb = 0;
    if (have) {
        unsigned si = (unsigned)st.state < 4u ? (unsigned)st.state : 3u;
        ESP_LOGW(TAG, "  CAN controller %s, TX error count %u, RX error count %u; errors since the last "
                      "report: ACK %u, bit %u, form %u, stuff %u, arbitration lost %u",
                 state_name[si], (unsigned)st.tx_error_count, (unsigned)st.rx_error_count,
                 (unsigned)e_ack, (unsigned)e_bit, (unsigned)e_form, (unsigned)e_stuff, (unsigned)e_arb);
        if (e_ack > 0 && e_bit == 0 && e_form == 0 && e_stuff == 0) {
            ESP_LOGW(TAG, "  -> only ACK errors: our frames go onto the bus intact (the transceiver reads "
                          "them back) and no node answers, and no node sends anything. Look at the motor "
                          "side: motor supply on (its CAN transceiver is powered from it), CANH/CANL "
                          "actually reach the motor connector, CANH/CANL not swapped, termination "
                          "(about 60 R between CANH and CANL with the power off)");
        } else if (e_bit > 0 && e_form == 0 && e_stuff == 0) {
            ESP_LOGW(TAG, "  -> bit errors: the controller does not read back its own bits. Look at our "
                          "side: transceiver supply, STB/S pin at GND, GPIO%d -> TXD and GPIO%d <- RXD "
                          "(not swapped), CANH/CANL not shorted",
                     s_node_cfg.io_cfg.tx, s_node_cfg.io_cfg.rx);
        } else if (e_form > 0 || e_stuff > 0) {
            ESP_LOGW(TAG, "  -> stuff/form errors: another node is active but its frames cannot be "
                          "decoded: most likely the motor's bit rate is not %d kbps (check it in the "
                          "Damiao tool), or CANH/CANL are swapped", s_bitrate_kbps);
        } else if (st.state == TWAI_ERROR_BUS_OFF) {
            /* A node alone on a working bus stops at error-passive (a missing
             * ACK does not raise the count further); bus-off needs bit errors:
             * the controller does not read back what it sends. */
            ESP_LOGW(TAG, "  -> bus-off: the controller does not read back its own bits. Likely: "
                          "transceiver unpowered or STB/S pin not at GND; GPIO%d/GPIO%d not wired to the "
                          "transceiver's TXD/RXD (or swapped); CANH and CANL shorted; or another node "
                          "on the bus at a bit rate other than %d kbps",
                     s_node_cfg.io_cfg.tx, s_node_cfg.io_cfg.rx, s_bitrate_kbps);
        } else if (st.tx_error_count >= 128) {
            ESP_LOGW(TAG, "  -> frames reach the bus but no node acknowledges them: motor (CAN side) "
                          "unpowered or not connected, CANH/CANL swapped or open, or the motor's bit "
                          "rate is not %d kbps", s_bitrate_kbps);
        } else if (st.tx_error_count == 0 && s_foreign_n == 0) {
            ESP_LOGW(TAG, "  -> our frames are acknowledged (some CAN node is on the bus at %d kbps), but "
                          "nothing answers ESC_ID 0x%03X: check the motor's CAN_ID in the Damiao tool. "
                          "A USB-CAN adapter left on the bus also acknowledges frames.",
                     s_bitrate_kbps, (unsigned)s_esc_id);
        }
    }
    uint32_t n = s_foreign_n;
    if (n) {
        uint32_t id = s_foreign_id;
        ESP_LOGW(TAG, "  %u frame(s) not decoded as feedback; last: %s ID 0x%03X, %u bytes, "
                      "%02X %02X %02X %02X %02X %02X %02X %02X",
                 (unsigned)n, (id & 0x80000000u) ? "extended" : "standard", (unsigned)(id & 0x1FFFFFFFu),
                 (unsigned)s_foreign_dlc, s_foreign_d[0], s_foreign_d[1], s_foreign_d[2], s_foreign_d[3],
                 s_foreign_d[4], s_foreign_d[5], s_foreign_d[6], s_foreign_d[7]);
        if (!(id & 0x80000000u) && (id & 0x7FFu) != s_mst_id) {
            ESP_LOGW(TAG, "  -> if this is the motor, its Master ID (MST_ID) is 0x%03X, not 0x%03X: "
                          "set MOTOR_TWAI_MST_ID in main/app_config.h. The low 4 bits of the first byte "
                          "(0x%X) are the motor's CAN_ID.",
                     (unsigned)(id & 0x7FFu), (unsigned)s_mst_id, (unsigned)(s_foreign_d[0] & 0x0F));
        } else if (!(id & 0x80000000u)) {
            ESP_LOGW(TAG, "  -> MST_ID matches, but the first byte's low 4 bits (0x%X) are not ESC_ID "
                          "0x%03X: set MOTOR_TWAI_ESC_ID in main/app_config.h to the motor's CAN_ID.",
                     (unsigned)(s_foreign_d[0] & 0x0F), (unsigned)s_esc_id);
        }
        s_foreign_n = 0;
    } else {
        ESP_LOGW(TAG, "  no CAN frame of any ID received");
    }
}

/*
 * Probe the motor with it disabled. Returns true and sets s_ready only when
 * every required step succeeded; anything else leaves the motor disabled and
 * not ready, and the task retries a second later.
 */
static bool motor_probe(void)
{
    static uint32_t s_attempt = 0;
    s_attempt++;
    const bool verbose = (s_attempt <= 3) || (s_attempt % 10 == 0);

    /* 1. Liveness: the drive answers any frame on its ID with feedback. */
    uint32_t fb0 = s_fb_count;
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    if (!s_node) {
        node_rebuild();
    }
    s_enabled = false;
    setpoint_invalidate();
    (void)motor_tx(s_esc_id, DM_FRAME_DISABLE, 8);
    xSemaphoreGive(s_tx_mutex);
    TickType_t t0 = xTaskGetTickCount();
    while (s_fb_count == fb0 &&
           (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(PROBE_ALIVE_WAIT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    if (s_fb_count == fb0) {
        if (verbose) {
            ESP_LOGW(TAG, "probe %u: no feedback from ESC_ID 0x%03X on MST_ID 0x%03X. "
                          "Motor powered? IDs and %d kbps right? Retrying every %d ms.",
                     (unsigned)s_attempt, (unsigned)s_esc_id, (unsigned)s_mst_id,
                     s_bitrate_kbps, PROBE_RETRY_MS);
            probe_diagnose();
        }
        return false;
    }

    /* 2. Map ranges and current settings. */
    float pmax = 0, vmax = 0, tmax = 0;
    uint32_t mode = 0, timeout = 0;
    if (!param_read_f32(REG_PMAX, &pmax) || !param_read_f32(REG_VMAX, &vmax) ||
        !param_read_f32(REG_TMAX, &tmax) || !param_read_u32(REG_CTRL_MODE, &mode) ||
        !param_read_u32(REG_TIMEOUT, &timeout)) {
        ESP_LOGE(TAG, "probe %u: motor answers feedback but not register reads",
                 (unsigned)s_attempt);
        return false;
    }
    if (!range_ok(pmax, PMAX_MIN_RAD) || pmax > PMAX_MAX_RAD ||
        !range_ok(vmax, 0.1f) || !range_ok(tmax, 0.1f)) {
        ESP_LOGE(TAG, "probe %u: implausible ranges PMAX=%g VMAX=%g TMAX=%g. PMAX must be "
                      "%.0f..%.0f rad (no wrap within a seek, fine enough steps); set it with "
                      "the Damiao debug tool. Motion stays disabled.",
                 (unsigned)s_attempt, pmax, vmax, tmax, PMAX_MIN_RAD, PMAX_MAX_RAD);
        return false;
    }

    /* 3. Control mode 4. Not saved to flash; the drive reverts on power-up,
     *    which a re-probe after any feedback loss rewrites. */
    if (mode != CTRL_MODE_FORCE_POS) {
        ESP_LOGW(TAG, "CTRL_MODE is %u, writing %u (force-position)",
                 (unsigned)mode, (unsigned)CTRL_MODE_FORCE_POS);
        if (!param_write_u32_verified(REG_CTRL_MODE, CTRL_MODE_FORCE_POS)) {
            ESP_LOGE(TAG, "probe %u: could not set CTRL_MODE", (unsigned)s_attempt);
            return false;
        }
        mode = CTRL_MODE_FORCE_POS;
    }

    /* 4. Comms-loss timeout: the drive disables itself if no command arrives
     *    for this long while enabled (manual p.25). */
    if (timeout != s_timeout_ticks) {
        if (!param_write_u32_verified(REG_TIMEOUT, s_timeout_ticks)) {
            ESP_LOGE(TAG, "probe %u: could not set TIMEOUT", (unsigned)s_attempt);
            return false;
        }
        timeout = s_timeout_ticks;
    }

    /* 5. Optional diagnostics: Kt * Imax from the manual's formula
     *    Kt = 1.5 * Npp * flux * GR * GREF (p.7). Not used for control. */
    float imax = 0, flux = 0, gr = 0, gref = 0;
    uint32_t npp = 0;
    float tau_per_pu = 0.0f;
    bool have_imax = param_read_f32(REG_IMAX, &imax) && isfinite(imax) && imax > 0.0f;
    if (have_imax && param_read_u32(REG_NPP, &npp) && param_read_f32(REG_FLUX, &flux) &&
        param_read_f32(REG_GR, &gr) && param_read_f32(REG_GREF, &gref) &&
        isfinite(flux) && isfinite(gr) && isfinite(gref) && npp > 0) {
        tau_per_pu = 1.5f * (float)npp * flux * gr * gref * imax;
    }
    if (!have_imax) {
        imax = 0.0f;
    }

    /* 6. The drive's trapezoid profile (RAM only, rewritten at every probe).
     *    Its flash default ACC 2 / DEC -2 rad/s^2 capped every move: a target
     *    0.2 rad ahead allows sqrt(2 x 2 x 0.2) = 0.89 rad/s (bench 2026-10-23:
     *    0.90 rad/s for a commanded 2). Not fatal: moves are only slower. */
    if (!param_write_f32_verified(REG_ACC, MOTOR_TWAI_ACC_RAD_S2) ||
        !param_write_f32_verified(REG_DEC, -MOTOR_TWAI_ACC_RAD_S2)) {
        ESP_LOGW(TAG, "probe %u: could not set ACC/DEC to %.0f rad/s^2; moves keep the drive's own limits",
                 (unsigned)s_attempt, MOTOR_TWAI_ACC_RAD_S2);
    }
    if (MOTOR_TWAI_KP_APR > 0.0f && !param_write_f32_verified(REG_KP_APR, MOTOR_TWAI_KP_APR)) {
        ESP_LOGW(TAG, "probe %u: could not set KP_APR to %.0f", (unsigned)s_attempt, MOTOR_TWAI_KP_APR);
    }

    /* 7. Diagnostics for motion tuning: the drive's own trapezoid limits and
     *    loop gains (manual p.17-18). Logged only. Bench 2026-10-23: the jaw
     *    moved at about 4.6 x (target - position) rad/s, whatever v_des. */
    static const struct { uint8_t rid; const char *name; } loop_regs[] = {
        {0x04, "ACC"}, {0x05, "DEC"}, {0x06, "MAX_SPD"}, {0x19, "KP_ASR"}, {0x1A, "KI_ASR"},
        {0x1B, "KP_APR"}, {0x1C, "KI_APR"}, {0x1F, "Deta"}, {0x20, "V_BW"}, {0x0B, "Damp"},
        {0x0C, "Inertia"},
    };
    char *regs = s_loop_regs;
    int nr = 0;
    for (size_t k = 0; k < sizeof(loop_regs) / sizeof(loop_regs[0]) && nr < (int)sizeof(s_loop_regs) - 24; k++) {
        float v = 0.0f;
        if (param_read_f32(loop_regs[k].rid, &v)) {
            nr += snprintf(regs + nr, sizeof(s_loop_regs) - (size_t)nr, " %s=%.4g", loop_regs[k].name, v);
        } else {
            nr += snprintf(regs + nr, sizeof(s_loop_regs) - (size_t)nr, " %s=?", loop_regs[k].name);
        }
    }
    ESP_LOGI(TAG, "drive loop registers:%s", regs);

    /* Commit: ranges before ready, so nothing reads stale ranges as valid. */
    s_pmax = pmax;
    s_vmax = vmax;
    s_tmax = tmax;
    s_info = (motor_twai_info_t){
        .pmax = pmax, .vmax = vmax, .tmax = tmax, .imax = imax,
        .tau_per_pu_est = tau_per_pu, .timeout_ticks = timeout, .ctrl_mode = mode,
    };
    s_have_info = true;
    s_epoch++;
    s_ready = true;
    ESP_LOGI(TAG, "probe OK (epoch %u): PMAX=%.3f rad VMAX=%.2f rad/s TMAX=%.2f N*m "
                  "CTRL_MODE=%u TIMEOUT=%u ticks (%u ms) Imax=%.2f A "
                  "Kt*Imax=%.3f N*m/pu (register estimate)",
             (unsigned)s_epoch, pmax, vmax, tmax, (unsigned)mode, (unsigned)timeout,
             (unsigned)(timeout * DM_TIMEOUT_TICK_US / 1000u), imax, tau_per_pu);
    s_attempt = 0;
    return true;
}

/* ---- RX path: on_rx_done runs in ISR context ---------------------------- */

static bool motor_on_rx_done(twai_node_handle_t handle,
                             const twai_rx_done_event_data_t *edata,
                             void *user_ctx)
{
    (void)edata; (void)user_ctx;

    uint8_t d[8] = {0};
    twai_frame_t rx = { .buffer = d, .buffer_len = sizeof(d) };
    if (twai_node_receive_from_isr(handle, &rx) != ESP_OK) {
        return false;
    }
    const uint8_t dlc = (uint8_t)rx.header.dlc;
    if (rx.header.ide || rx.header.id != s_mst_id) {
        s_foreign_id = rx.header.id | (rx.header.ide ? 0x80000000u : 0u);
        s_foreign_dlc = dlc;
        for (int k = 0; k < 8; k++) s_foreign_d[k] = d[k];
        s_foreign_n++;
        return false;
    }

    /* Register replies share MST_ID with feedback. They echo the motor CAN_ID
     * in D0/D1 and the command (0x33 read, 0x55 write, 0xAA save) in D2. */
    uint16_t echo_id = (uint16_t)(d[0] | (d[1] << 8));
    if (dlc >= 4 && echo_id == (uint16_t)s_esc_id &&
        (d[2] == DM_PARAM_CMD_READ || d[2] == DM_PARAM_CMD_WRITE || d[2] == DM_PARAM_CMD_SAVE)) {
        uint8_t rid = d[3];
        if (d[2] == DM_PARAM_CMD_SAVE && rid == 0x01) {   /* reply: ... 0xAA 0x01 (p.16) */
            s_save_ack = true;
        }
        if (dlc == 8 && d[2] != DM_PARAM_CMD_SAVE && rid < DM_PARAM_CACHE_LEN) {
            uint32_t raw;
            memcpy(&raw, &d[4], 4);
            s_param_raw[rid] = raw;
            s_param_valid[rid] = true;
        }
        return false;
    }

    /* Feedback frame (8 bytes, same in every mode, manual p.13):
     *   D0 = id[3:0] | err << 4
     *   D1 = pos[15:8]   D2 = pos[7:0]
     *   D3 = vel[11:4]   D4 = vel[3:0]<<4 | tq[11:8]   D5 = tq[7:0]
     *   D6 = T_MOS (degC)  D7 = T_Rotor (degC) */
    if (dlc != 8 || (d[0] & 0x0F) != (s_esc_id & 0x0F)) {
        s_foreign_id = rx.header.id;
        s_foreign_dlc = dlc;
        for (int k = 0; k < 8; k++) s_foreign_d[k] = d[k];
        s_foreign_n++;
        return false;
    }
    uint8_t  err = d[0] >> 4;
    uint16_t p_int = (uint16_t)((d[1] << 8) | d[2]);
    uint16_t v_int = (uint16_t)((d[3] << 4) | (d[4] >> 4));
    uint16_t t_int = (uint16_t)(((d[4] & 0xF) << 8) | d[5]);
    const float pmax = s_pmax, vmax = s_vmax, tmax = s_tmax;

    static uint32_t seq = 0;
    const TickType_t now = xTaskGetTickCountFromISR();
    motor_status_t st = {
        .seq        = ++seq,
        .t_ms       = (uint32_t)(now * portTICK_PERIOD_MS),
        .enabled    = (err == 1),
        .position   = uint_to_float(p_int, -pmax, pmax, 16),
        .velocity   = uint_to_float(v_int, -vmax, vmax, 12),
        .torque     = uint_to_float(t_int, -tmax, tmax, 12),
        .t_mos      = (float)d[6],
        .t_rotor    = (float)d[7],
        .error_code = (err <= 1) ? 0 : err,   /* 0/1 are healthy, >=8 are faults */
    };
    s_last_fb_err = err;
    s_last_fb_tick = now;
    s_fb_count++;

    BaseType_t hp_task_woken = pdFALSE;
    if (s_bus && s_bus->motor_status_q) {
        xQueueSendFromISR(s_bus->motor_status_q, &st, &hp_task_woken);
    }
    return hp_task_woken == pdTRUE;
}

/* Bus error interrupt: count the error types for the probe report. An ACK
 * error alone means our frame went onto the bus intact and nobody answered;
 * bit errors mean the controller did not read back what it sent; stuff and
 * form errors mean another node is sending something this node cannot decode. */
static bool motor_on_error(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    (void)handle; (void)user_ctx;
    if (edata->err_flags.ack_err) s_err_ack++;
    if (edata->err_flags.bit_err) s_err_bit++;
    if (edata->err_flags.form_err) s_err_form++;
    if (edata->err_flags.stuff_err) s_err_stuff++;
    if (edata->err_flags.arb_lost) s_err_arb++;
    return false;
}

/* ---- node lifecycle ------------------------------------------------------ */

static esp_err_t node_create(void)
{
    esp_err_t err = twai_new_node_onchip(&s_node_cfg, &s_node);
    if (err != ESP_OK) {
        s_node = NULL;
        return err;
    }
    const twai_event_callbacks_t cbs = {
        .on_rx_done = motor_on_rx_done,
        .on_error = motor_on_error,
    };
    /* Accept every standard frame; motor_on_rx_done() decodes only MST_ID and
     * records anything else, so a motor configured with other IDs shows up in
     * the probe log instead of being dropped silently by the hardware filter.
     * The bus carries only this motor, so the extra interrupts are few. */
    const twai_mask_filter_config_t filter = { .id = 0, .mask = 0, .is_ext = false };
    err = twai_node_register_event_callbacks(s_node, &cbs, NULL);
    if (err == ESP_OK) err = twai_node_config_mask_filter(s_node, 0, &filter);
    if (err == ESP_OK) err = twai_node_enable(s_node);
    if (err != ESP_OK) {
        (void)twai_node_delete(s_node);
        s_node = NULL;
    }
    return err;
}

/* Drop whatever the controller is still trying to send. A frame that no node
 * ACKs (motor unpowered) is retransmitted for ever, and would be delivered the
 * moment the motor powers up again - an enable or command frame from before
 * the power loss. Disabling and re-enabling the node resumes that frame, so
 * the node is deleted and rebuilt instead. Caller holds s_tx_mutex. */
static void node_rebuild(void)
{
    if (s_node) {
        (void)twai_node_disable(s_node);
        (void)twai_node_delete(s_node);
        s_node = NULL;
    }
    esp_err_t err = node_create();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI node rebuild failed: %s (retried at the next probe)", esp_err_to_name(err));
    }
}

/* ---- task ---------------------------------------------------------------- */

static void motor_twai_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "motor_twai_task running");

    TickType_t next_wake = xTaskGetTickCount();
    uint32_t poll_ms = 0;
    for (;;) {
        if (!s_ready) {
            if (!motor_probe()) {
                vTaskDelay(pdMS_TO_TICKS(PROBE_RETRY_MS));
            }
            next_wake = xTaskGetTickCount();
            poll_ms = 0;
            continue;
        }

        xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
        if (s_enabled) {
            float p, v, i;
            if (setpoint_get(&p, &v, &i)) {
                send_force_pos(p, v, i);
            } else {
                /* Cannot happen (disable clears both together); fail safe. */
                s_enabled = false;
                (void)motor_tx(s_esc_id, DM_FRAME_DISABLE, 8);
            }
            poll_ms = 0;
        } else {
            poll_ms += CMD_PERIOD_MS;
            if (poll_ms >= POLL_PERIOD_MS) {
                poll_ms = 0;
                (void)motor_tx(s_esc_id, DM_FRAME_DISABLE, 8);   /* status poll */
            }
        }
        xSemaphoreGive(s_tx_mutex);

        /* Enabled, but the newest feedback (received well after the enable)
         * says "disabled" with no fault code: the drive restarted or ignored
         * the enable. Faults (err >= 8) are reported through the feedback and
         * handled by the supervisor. */
        TickType_t now = xTaskGetTickCount();
        if (s_enabled && s_last_fb_err == 0 &&
            (int32_t)(s_last_fb_tick - s_enable_tick) > (int32_t)pdMS_TO_TICKS(ENABLE_GRACE_MS)) {
            xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
            s_enabled = false;
            s_ready = false;
            setpoint_invalidate();
            (void)motor_tx(s_esc_id, DM_FRAME_DISABLE, 8);
            xSemaphoreGive(s_tx_mutex);
            ESP_LOGE(TAG, "drive reports disabled while it should be enabled (restart?) - re-probing");
        }
        /* The setpoint owner (the gripper) refreshes every 10 ms. If it stops,
         * stop driving rather than replay an ageing command. If this task was
         * itself held up (a flash write stalls every task), give the owner one
         * period to catch up before judging. */
        static TickType_t s_last_loop = 0;
        static TickType_t s_grace_until = 0;
        if (s_last_loop != 0 && (int32_t)(now - s_last_loop) > (int32_t)pdMS_TO_TICKS(5 * CMD_PERIOD_MS)) {
            s_grace_until = now + pdMS_TO_TICKS(3 * CMD_PERIOD_MS);
        }
        s_last_loop = now;
        if (s_enabled && (int32_t)(now - s_grace_until) >= 0 &&
            (int32_t)(now - s_sp_tick) > (int32_t)pdMS_TO_TICKS(SETPOINT_STALE_MS)) {
            xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
            s_enabled = false;
            setpoint_invalidate();
            (void)motor_tx(s_esc_id, DM_FRAME_DISABLE, 8);
            xSemaphoreGive(s_tx_mutex);
            ESP_LOGE(TAG, "setpoint not refreshed for %u ms - motor disabled",
                     (unsigned)((now - s_sp_tick) * portTICK_PERIOD_MS));
        }

        uint32_t age = motor_twai_feedback_age_ms();
        if (age > FEEDBACK_LOST_MS) {
            xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
            bool was_enabled = s_enabled;
            s_enabled = false;
            s_ready = false;
            setpoint_invalidate();
            node_rebuild();          /* nobody is answering: drop any pending frame */
            xSemaphoreGive(s_tx_mutex);
            ESP_LOGE(TAG, "no feedback for %u ms%s - motor not ready, probing again",
                     (unsigned)age, was_enabled ? " while enabled" : "");
        }

        xTaskDelayUntil(&next_wake, pdMS_TO_TICKS(CMD_PERIOD_MS));
    }
}

int motor_twai_start(const motor_twai_cfg_t *cfg, const app_bus_t *bus)
{
    if (!cfg || !bus) {
        return ESP_ERR_INVALID_ARG;
    }
    s_bus = bus;
    s_esc_id = cfg->esc_id >= 0 ? (uint32_t)cfg->esc_id : MOTOR_ESC_ID_DEFAULT;
    s_mst_id = cfg->mst_id >= 0 ? (uint32_t)cfg->mst_id : MOTOR_MST_ID_DEFAULT;
    s_bitrate_kbps = cfg->bitrate_kbps;
    int timeout_ms = cfg->timeout_ms > 0 ? cfg->timeout_ms : TIMEOUT_MS_DEFAULT;
    s_timeout_ticks = (uint32_t)timeout_ms * 1000u / DM_TIMEOUT_TICK_US;

    s_tx_mutex = xSemaphoreCreateMutex();
    if (!s_tx_mutex) {
        return ESP_ERR_NO_MEM;
    }

    s_node_cfg = (twai_onchip_node_config_t){
        .io_cfg = {
            .tx = cfg->tx_gpio,
            .rx = cfg->rx_gpio,
            .quanta_clk_out = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing = {
            .bitrate = (uint32_t)cfg->bitrate_kbps * 1000,
        },
        .tx_queue_depth = 8,
    };
    ESP_ERROR_CHECK(node_create());

    ESP_LOGI(TAG, "TWAI up: TX=GPIO%d RX=GPIO%d @ %d kbps, ESC_ID 0x%03X MST_ID 0x%03X, "
                  "comms-loss timeout %d ms (external transceiver required)",
             cfg->tx_gpio, cfg->rx_gpio, cfg->bitrate_kbps, (unsigned)s_esc_id,
             (unsigned)s_mst_id, timeout_ms);

    uint32_t   stack = cfg->task_stack > 0 ? (uint32_t)cfg->task_stack : 4096;
    UBaseType_t prio = cfg->task_prio  > 0 ? (UBaseType_t)cfg->task_prio : 7;
    if (xTaskCreate(motor_twai_task, "motor_twai", stack, NULL, prio, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create motor_twai_task");
        return ESP_FAIL;
    }
    return ESP_OK;
}
