/**
 * @file app_supervisor.c
 * @brief Command dispatch, IMU/haptic telemetry, and the gripper control loop.
 *
 * Every 10 ms this task drains the IMU, tactile and motor queues, executes
 * queued host commands, assembles a gripper_input_t and calls gripper_tick().
 * The gripper module owns all motion decisions; this file only decides
 * whether motion is permitted at all:
 *
 *   - SAFE_STOP (host command, or a pad above GRIPPER_OVERFORCE_N while the
 *     motor is enabled) disables the motor and blocks motion until
 *     MOTOR_CLEAR_ERR;
 *   - a motor fault code in the feedback does the same;
 *   - motor and gripper actuation needs an unlocked session.
 *
 * IMU silence is reported as APP_STATE_ERROR but never blocks the gripper:
 * the IMU plays no part in gripping.
 */
#include "app_supervisor.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs.h"

#include "app_proto.h"
#include "imu_uart.h"
#include "haptic_uart.h"
#include "motor_twai.h"
#include "gripper.h"
#include "wireless_wifi.h"

static const char *TAG = "supervisor";

#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 2

#define LOOP_PERIOD_MS      10
#define IMU_TIMEOUT_MS      1500  /* allows one in-driver wake/reconfigure attempt */
#define IMU_DATA_FRESH_MS    500  /* stream/get require a recently received sample */
#define HAPTIC_DATA_FRESH_MS 500
/* The gripper needs much newer data than telemetry does: at 80 Hz polling a
 * new sample arrives every 12.5 ms, so 60 ms means about four missed polls. */
#define GRIP_TACTILE_FRESH_MS 60
/* Motor feedback: 100 Hz while enabled, 20 Hz status polls while disabled. */
#define GRIP_MOTOR_FRESH_MS   120
#define OVERFORCE_N_DEFAULT   24.0f
#define OVERFORCE_SAMPLES     2      /* in a row: a single 25.5 N (0xFF) frame
                                      * right after a tare tripped it once with
                                      * the pads 7 rad apart (bench 2026-10-23) */
#define TARE_LOADED_N         0.5f   /* refuse a tare while a pad reads more */
#define TARE_FRESH_MS         200    /* ... and unless both pads are this fresh */

/* Console echo of the decoded sample: print one in every N, 0 = silent. */
#ifndef IMU_PRINT_EVERY_N
#define IMU_PRINT_EVERY_N   10
#endif

static const app_bus_t     *s_bus = NULL;
static volatile app_state_t s_state = APP_STATE_INIT;
static float                s_overforce_n = OVERFORCE_N_DEFAULT;

/* motion permission */
static uint8_t s_unlock_mask = 0;     /* APP_TR_BIT per unlocked transport */
static uint8_t s_reply_tr = APP_TR_USB;  /* transport of the command being handled */
static bool s_safe_stop   = false;   /* latched until MOTOR_CLEAR_ERR          */
static bool s_motor_fault = false;   /* drive reported a fault; same latch     */
#define MOTOR_ERR_COMM_LOST   0xD    /* DM drive: commands stopped while enabled */
#define BOOT_CLEAR_GRACE_MS   1000   /* after the start-up clear, 0xD frames
                                      * still in flight are ignored this long;
                                      * a 0xD after that latches as usual      */
static bool       s_boot_fault_checked = false;  /* first motor status seen      */
static TickType_t s_boot_clear_until = 0;
static uint8_t    s_over_n[HAPTIC_SENSOR_COUNT];   /* see OVERFORCE_SAMPLES */

/* streaming state (the "user layer" runtime) */
static uint8_t s_stream_imu = 0;      /* APP_TR_BIT per transport that enabled it */
static uint8_t s_stream_haptic = 0;
static bool s_powon_output = false;
static TickType_t s_last_imu_tick = 0;
static TickType_t s_last_haptic_tick[HAPTIC_SENSOR_COUNT];

#define IMU_NVS_NS         "imu_cfg"
#define IMU_NVS_POWON_KEY  "powon_send"

static bool load_powon_preference(void)
{
    nvs_handle_t handle = 0;
    uint8_t value = 0;
    esp_err_t err = nvs_open(IMU_NVS_NS, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        err = nvs_get_u8(handle, IMU_NVS_POWON_KEY, &value);
        nvs_close(handle);
    }
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "no saved POWONSEND preference; defaulting to off");
        return false;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot load POWONSEND preference: %s; defaulting to off",
                 esp_err_to_name(err));
        return false;
    }
    return value != 0;
}

static bool save_powon_preference(bool on)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(IMU_NVS_NS, NVS_READWRITE, &handle);
    if (err == ESP_OK) err = nvs_set_u8(handle, IMU_NVS_POWON_KEY, on ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(handle);
    if (handle) nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "cannot save POWONSEND preference: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

/* latest snapshot, for telemetry and OP_IMU_GET */
static imu_sample_t s_last_imu;
static bool         s_have_imu = false;
static haptic_sample_t s_last_haptic[HAPTIC_SENSOR_COUNT];
static bool             s_have_haptic[HAPTIC_SENSOR_COUNT];
static float            s_point_max[HAPTIC_SENSOR_COUNT];   /* N, per sample */

/* latest motor feedback */
static motor_status_t s_last_motor;
static bool           s_have_motor = false;

static bool imu_data_fresh(void)
{
    TickType_t now = xTaskGetTickCount();
    return s_have_imu && s_last_imu_tick != 0 &&
           now - s_last_imu_tick <= pdMS_TO_TICKS(IMU_DATA_FRESH_MS);
}

static bool haptic_fresh_within(uint8_t sensor_index, uint32_t max_age_ms)
{
    if (sensor_index >= HAPTIC_SENSOR_COUNT) return false;
    TickType_t now = xTaskGetTickCount();
    return s_have_haptic[sensor_index] && s_last_haptic_tick[sensor_index] != 0 &&
           now - s_last_haptic_tick[sensor_index] <= pdMS_TO_TICKS(max_age_ms);
}

static bool haptic_data_fresh(uint8_t sensor_index)
{
    return haptic_fresh_within(sensor_index, HAPTIC_DATA_FRESH_MS);
}

static bool both_haptic_fresh(void)
{
    return haptic_data_fresh(0) && haptic_data_fresh(1);
}

/* Age of a tactile sample from its own receive stamp (esp_timer ms), so a
 * supervisor loop that ran late cannot make old data look new. */
static uint32_t haptic_sample_age_ms(uint8_t i)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    return now - s_last_haptic[i].timestamp_ms;
}

/* A tare makes the current reading the new zero. Allow it only when both pads
 * are known to be unloaded right now: fresh data on both, both below
 * TARE_LOADED_N. Old or missing data is not "unloaded". */
static bool pads_known_unloaded(void)
{
    for (uint8_t i = 0; i < HAPTIC_SENSOR_COUNT; i++) {
        if (!s_have_haptic[i] || haptic_sample_age_ms(i) > TARE_FRESH_MS ||
            s_last_haptic[i].resultant[2] > TARE_LOADED_N) {
            return false;
        }
    }
    return true;
}

static bool motion_allowed(void)
{
    return !s_safe_stop && !s_motor_fault;
}

const volatile app_state_t *app_supervisor_state_ptr(void) { return &s_state; }

static void enter_state(app_state_t next)
{
    if (next != s_state) {
        ESP_LOGI(TAG, "state %s -> %s", app_state_name(s_state), app_state_name(next));
        s_state = next;
    }
}

static void safe_stop(const char *why)
{
    ESP_LOGE(TAG, "SAFE_STOP: %s", why);
    gripper_abort();
    motor_twai_disable();
    s_safe_stop = true;
    enter_state(APP_STATE_SAFE_STOP);
}

/* ---- reply helpers ------------------------------------------------------ */
static void reply_ack(uint8_t seq, uint8_t op, uint8_t status)
{
    uint8_t f[APP_PROTO_HDR_LEN + 2 + 2];
    size_t n = app_proto_ack(f, sizeof(f), seq, op, status);
    if (n) app_proto_emit_to(s_reply_tr, f, n);
}

static void emit_imu(void)
{
    uint8_t f[APP_PROTO_MAX_FRAME];
    size_t n = app_proto_tele_imu(f, sizeof(f), &s_last_imu);
    if (n) app_proto_emit_to(s_reply_tr, f, n);
}

static void emit_imu_batch(const imu_sample_t *samples, size_t count)
{
    uint8_t f[APP_PROTO_MAX_FRAME];
    size_t n = app_proto_tele_imu_batch(f, sizeof(f), samples, count);
    if (n) app_proto_emit_mask(s_stream_imu, f, n);
}

static void emit_haptic(uint8_t sensor_index, uint8_t mask)
{
    uint8_t f[APP_PROTO_MAX_FRAME];
    size_t n = app_proto_tele_haptic(f, sizeof(f), &s_last_haptic[sensor_index]);
    if (n) app_proto_emit_mask(mask, f, n);
}

static void emit_motor(void)
{
    uint8_t f[APP_PROTO_HDR_LEN + 32 + 2];
    size_t n = app_proto_tele_motor(f, sizeof(f), &s_last_motor);
    if (n) app_proto_emit_to(s_reply_tr, f, n);
}

/* GRIP_GET_CALIB response, little-endian:
 *   op u8, valid u8, gripper state u8, last fault u8,
 *   pos_closed, pos_open, pos_touch (rad), k_force (N/pu), i_offset (pu),
 *   i_move (pu), default_force (N), position (rad), opening (0..1, -1 = n/a),
 *   force target (N) - 10 x f32. */
static void emit_grip_calib(uint8_t seq)
{
    gripper_calib_t cal;
    gripper_status_t st;
    gripper_get_calib(&cal);
    gripper_get_status(&st);
    const float v[10] = {
        cal.pos_closed, cal.pos_open, cal.pos_touch, cal.k_force, cal.i_offset,
        cal.i_move, cal.default_force, st.pos, st.opening, st.force_target,
    };
    uint8_t pl[4 + sizeof(v)];
    pl[0] = OP_GRIP_GET_CALIB;
    pl[1] = cal.valid ? 1 : 0;
    pl[2] = (uint8_t)st.state;
    pl[3] = (uint8_t)st.fault;
    memcpy(&pl[4], v, sizeof(v));      /* ESP32-C6 is little-endian */
    uint8_t f[APP_PROTO_HDR_LEN + sizeof(pl) + 2];
    size_t n = app_proto_build(f, sizeof(f), APP_PROTO_T_RESPONSE, seq, pl, (uint16_t)sizeof(pl));
    if (n) app_proto_emit_to(s_reply_tr, f, n);
}

/* OP_WIFI_GET_INFO response, little-endian:
 *   op u8, started u8, station state u8 (0 none, 1 connecting, 2 connected),
 *   client connected u8, AP ip u32, station ip u32, client ip u32,
 *   AP ssid (u8 length + bytes), station ssid (u8 length + bytes). */
static void emit_wifi_info(uint8_t seq)
{
    wireless_wifi_info_t wi;
    memset(&wi, 0, sizeof(wi));
    wireless_wifi_get_info(&wi);
    uint8_t pl[4 + 12 + 1 + 32 + 1 + 32];
    size_t k = 0;
    pl[k++] = OP_WIFI_GET_INFO;
    pl[k++] = wi.started ? 1 : 0;
    pl[k++] = wi.sta_state;
    pl[k++] = wi.client ? 1 : 0;
    memcpy(&pl[k], &wi.ap_ip, 4); k += 4;
    memcpy(&pl[k], &wi.sta_ip, 4); k += 4;
    memcpy(&pl[k], &wi.client_ip, 4); k += 4;
    size_t a = strnlen(wi.ap_ssid, 32);
    pl[k++] = (uint8_t)a; memcpy(&pl[k], wi.ap_ssid, a); k += a;
    size_t b = strnlen(wi.sta_ssid, 32);
    pl[k++] = (uint8_t)b; memcpy(&pl[k], wi.sta_ssid, b); k += b;
    uint8_t f[APP_PROTO_HDR_LEN + sizeof(pl) + 2];
    size_t n = app_proto_build(f, sizeof(f), APP_PROTO_T_RESPONSE, seq, pl, (uint16_t)k);
    if (n) app_proto_emit_to(s_reply_tr, f, n);
}

static bool is_motion_op(uint8_t op)
{
    switch (op) {
    case OP_SELF_CALIBRATE:
    case OP_GRIP_CALIBRATE:
    case OP_GRIP_HOME:
    case OP_GRIP_SET_CLOSE:
    case OP_GRIP_GRIP:
    case OP_GRIP_MOVE_DEG:
        return true;
    default:
        return false;
    }
}

/* Ops that change the motor or the gripper and therefore need an unlocked
 * session. Stopping (SAFE_STOP, MOTOR_DISABLE, GRIP_STOP) and reads are
 * always allowed. */
static bool needs_unlock(uint8_t op)
{
    return is_motion_op(op) || op == OP_MOTOR_CLEAR_ERR || op == OP_GRIP_SET_FORCE ||
           op == OP_MOTOR_SAVE_CONFIG || op == OP_WIFI_SET_STA;
}

/* Execute one decoded user command.
 *
 * The telemetry stream is off until STREAM_EN asks for it and stays off after
 * STREAM_DIS. There is deliberately no "any IMU command also starts the
 * stream" shortcut: it made STREAM_EN unnecessary (so nobody sent it), and it
 * meant a calibration request silently opened a high-rate feed on a link the
 * console is already sharing. */
static void handle_cmd(const app_cmd_t *c)
{
    s_reply_tr = c->transport < APP_TR_COUNT ? c->transport : APP_TR_USB;
    if (c->op == OP_INTERNAL_TRANSPORT_CLOSED) {
        /* The peer of this transport left (BLE disconnect, TCP close). Relock
         * it and stop its streams. A move it started stops where it is; a
         * grip keeps holding the object (releasing it could drop it). */
        s_unlock_mask &= (uint8_t)~APP_TR_BIT(s_reply_tr);
        s_stream_imu &= (uint8_t)~APP_TR_BIT(s_reply_tr);
        s_stream_haptic &= (uint8_t)~APP_TR_BIT(s_reply_tr);
        gripper_status_t st;
        gripper_get_status(&st);
        if (st.state == GRIPPER_MOVING) {
            gripper_stop();
        }
        ESP_LOGI(TAG, "transport %u closed: session locked%s", (unsigned)s_reply_tr,
                 st.state == GRIPPER_MOVING ? ", move stopped" : "");
        return;
    }
    if (needs_unlock(c->op) && !(s_unlock_mask & APP_TR_BIT(s_reply_tr))) {
        ESP_LOGW(TAG, "op 0x%02x rejected: session locked", c->op);
        reply_ack(c->seq, c->op, APP_PROTO_ERR_LOCKED);
        return;
    }
    if (is_motion_op(c->op) && !motion_allowed()) {
        ESP_LOGW(TAG, "op 0x%02x rejected: %s", c->op,
                 s_safe_stop ? "SAFE_STOP latched" : "motor fault latched");
        reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        return;
    }

    switch (c->op) {
    /* --- system --- */
    case OP_PING:
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_GET_VERSION: {
        uint8_t pl[4] = { OP_GET_VERSION, APP_PROTO_VERSION, FW_VERSION_MAJOR, FW_VERSION_MINOR };
        uint8_t f[APP_PROTO_HDR_LEN + sizeof(pl) + 2];
        size_t n = app_proto_build(f, sizeof(f), APP_PROTO_T_RESPONSE, c->seq, pl, sizeof(pl));
        if (n) app_proto_emit_to(s_reply_tr, f, n);
        break;
    }
    case OP_GET_STATE: {
        uint8_t f[APP_PROTO_HDR_LEN + 2 + 2];
        size_t n = app_proto_resp_state(f, sizeof(f), c->seq, (uint8_t)s_state);
        if (n) app_proto_emit_to(s_reply_tr, f, n);
        break;
    }
    case OP_SAFE_STOP:
        safe_stop("host command");
        reply_ack(c->seq, c->op, APP_PROTO_OK);   /* always allowed */
        break;
    case OP_SESSION_UNLOCK:
        /* per transport: unlocking USB does not let a BLE or Wi-Fi peer move the jaw */
        if (c->u0 == APP_PROTO_SESSION_KEY) {
            s_unlock_mask |= APP_TR_BIT(s_reply_tr);
            reply_ack(c->seq, c->op, APP_PROTO_OK);
        } else {
            s_unlock_mask &= (uint8_t)~APP_TR_BIT(s_reply_tr);
            reply_ack(c->seq, c->op, APP_PROTO_ERR_ARG);
        }
        break;
    case OP_SESSION_LOCK:
        s_unlock_mask &= (uint8_t)~APP_TR_BIT(s_reply_tr);
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_WIFI_SET_STA: {
        /* data = u8 ssid_len, ssid, u8 pass_len, pass */
        char ssid[33] = {0}, pass[65] = {0};
        size_t sl = c->data_len >= 1 ? c->data[0] : 0;
        size_t pl_off = 1 + sl;
        size_t pw = (c->data_len > pl_off) ? c->data[pl_off] : 0;
        if (sl > 32 || pw > 64 || 1 + sl + 1 + pw > c->data_len) {
            reply_ack(c->seq, c->op, APP_PROTO_ERR_ARG);
            break;
        }
        memcpy(ssid, &c->data[1], sl);
        memcpy(pass, &c->data[pl_off + 1], pw);
        reply_ack(c->seq, c->op, wireless_wifi_set_sta(ssid, pass) == ESP_OK ? APP_PROTO_OK
                                                                           : APP_PROTO_ERR_STATE);
        break;
    }
    case OP_WIFI_GET_INFO:
        emit_wifi_info(c->seq);
        break;
    case OP_SELF_CALIBRATE:
        /* Gripper calibration, which includes the tactile tare. The IMU
         * calibration needs the unit still and level, which a moving jaw
         * cannot guarantee; it stays a separate command (0x23). */
        reply_ack(c->seq, c->op, gripper_calibrate() ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;

    /* --- motor ---
     * MOTOR_ENABLE, SET_TARGET, SET_GAINS and SAVE_ZERO are retired: the
     * gripper owns the motor in force-position mode. ENABLE bypassed its
     * limits, SET_TARGET/SET_GAINS were MIT-mode commands, and SAVE_ZERO moves
     * the drive's position frame under the calibration. */
    case OP_MOTOR_DISABLE:
        gripper_abort();
        motor_twai_disable();
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_MOTOR_CLEAR_ERR:
        /* Clearing while the gripper drives the motor would disable it under
         * the gripper (and drop a held object); clearing a SAFE_STOP while a
         * pad is still at the trip force would re-arm motion into it. */
        if (gripper_motor_active()) {
            ESP_LOGW(TAG, "clear refused: the gripper is driving the motor (send MOTOR_DISABLE first)");
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
            break;
        }
        if (s_safe_stop) {
            bool loaded = false;
            for (uint8_t i = 0; i < HAPTIC_SENSOR_COUNT; i++) {
                if (s_have_haptic[i] && s_last_haptic[i].resultant[2] >= s_overforce_n) loaded = true;
            }
            if (loaded) {
                ESP_LOGW(TAG, "clear refused: a pad still reads >= %.1f N", s_overforce_n);
                reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
                break;
            }
        }
        motor_twai_clear_error();
        s_motor_fault = false;
        s_safe_stop = false;
        gripper_clear_fault();
        if (s_state == APP_STATE_ERROR || s_state == APP_STATE_SAFE_STOP) {
            enter_state(APP_STATE_IDLE);
        }
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_MOTOR_GET_STATUS:
        if (s_have_motor) emit_motor();
        else reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        break;
    case OP_MOTOR_SAVE_CONFIG:
        /* Writes the motor's own flash. Runs here, in the supervisor task, so
         * no gripper_tick() can enable the motor during the register writes;
         * blocks this task for up to about a second. */
        if (gripper_motor_active() || gripper_is_busy()) {
            ESP_LOGW(TAG, "save config refused: the gripper is driving the motor (send MOTOR_DISABLE first)");
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
            break;
        }
        reply_ack(c->seq, c->op, motor_twai_persist_config() ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;

    /* --- IMU --- */
    case OP_IMU_STREAM_EN:
        if (!imu_data_fresh()) {
            ESP_LOGW(TAG, "cannot enable USB telemetry: no fresh IMU data");
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        } else {
            s_stream_imu |= APP_TR_BIT(s_reply_tr);
            ESP_LOGI(TAG, "IMU telemetry enabled on transport %u", (unsigned)s_reply_tr);
            reply_ack(c->seq, c->op, APP_PROTO_OK);
        }
        break;
    case OP_IMU_STREAM_DIS:
        s_stream_imu &= (uint8_t)~APP_TR_BIT(s_reply_tr);
        ESP_LOGI(TAG, "IMU telemetry stopped on transport %u (sensor acquisition remains active)",
                 (unsigned)s_reply_tr);
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_IMU_GET:
        if (imu_data_fresh()) emit_imu();
        else reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        break;
    case OP_IMU_CALIBRATE:
        reply_ack(c->seq, c->op,
                  imu_uart_cal_accel() ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;
    case OP_IMU_SET_REF:
        reply_ack(c->seq, c->op,
                  imu_uart_set_angle_ref() ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;
    case OP_IMU_POWON_SET:
        if (c->u0 > 1) {
            reply_ack(c->seq, c->op, APP_PROTO_ERR_ARG);
        } else {
            bool on = c->u0 != 0;
            bool accepted = imu_uart_set_powon_output(on);
            if (accepted) {
                s_powon_output = on;
                (void)save_powon_preference(on);
            }
            reply_ack(c->seq, c->op, accepted ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        }
        break;
    case OP_IMU_SET_RATE:
        reply_ack(c->seq, c->op,
                  imu_uart_set_rate_hz((int)c->u0) ? APP_PROTO_OK : APP_PROTO_ERR_ARG);
        break;
    case OP_IMU_ACQ_EN: {
        bool accepted = imu_uart_set_acquisition(true);
        ESP_LOGI(TAG, "IMU acquisition resume %s", accepted ? "requested" : "rejected");
        reply_ack(c->seq, c->op, accepted ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;
    }
    case OP_IMU_ACQ_DIS: {
        bool accepted = imu_uart_set_acquisition(false);
        if (accepted) {
            s_have_imu = false;
        }
        ESP_LOGI(TAG, "IMU acquisition stop %s", accepted ? "requested" : "rejected");
        reply_ack(c->seq, c->op, accepted ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;
    }

    /* --- Haptic: sensor polling and USB reporting are independent. --- */
    case OP_HAPTIC_STREAM_EN:
        if (!both_haptic_fresh()) {
            ESP_LOGW(TAG, "cannot enable haptic USB telemetry: both sensors are not fresh");
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        } else {
            s_stream_haptic |= APP_TR_BIT(s_reply_tr);
            ESP_LOGI(TAG, "haptic telemetry enabled on transport %u", (unsigned)s_reply_tr);
            reply_ack(c->seq, c->op, APP_PROTO_OK);
        }
        break;
    case OP_HAPTIC_STREAM_DIS:
        s_stream_haptic &= (uint8_t)~APP_TR_BIT(s_reply_tr);
        ESP_LOGI(TAG, "haptic telemetry stopped on transport %u (sensor polling unchanged)",
                 (unsigned)s_reply_tr);
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_HAPTIC_GET:
        if (both_haptic_fresh()) {
            emit_haptic(0, APP_TR_BIT(s_reply_tr));
            emit_haptic(1, APP_TR_BIT(s_reply_tr));
        } else {
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        }
        break;
    case OP_HAPTIC_CALIBRATE:
        /* A tare pauses polling for about a second and redefines zero: never
         * while the gripper drives the motor, and only with both pads known
         * to be unloaded (the PaXini manual requires an unloaded sensor). */
        if (gripper_is_busy() || gripper_motor_active() || !pads_known_unloaded()) {
            ESP_LOGW(TAG, "tactile tare refused: %s",
                     (gripper_is_busy() || gripper_motor_active()) ? "the gripper is driving the motor"
                                                                   : "pads not fresh and unloaded");
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        } else {
            reply_ack(c->seq, c->op,
                      haptic_uart_calibrate() ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        }
        break;
    case OP_HAPTIC_ACQ_EN:
        reply_ack(c->seq, c->op,
                  haptic_uart_set_acquisition(true) ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;
    case OP_HAPTIC_ACQ_DIS:
        if (gripper_is_busy() || gripper_motor_active()) {
            ESP_LOGW(TAG, "tactile polling stop refused: the gripper is driving the motor");
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        } else if (haptic_uart_set_acquisition(false)) {
            memset(s_have_haptic, 0, sizeof(s_have_haptic));
            reply_ack(c->seq, c->op, APP_PROTO_OK);
        } else {
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        }
        break;

    /* --- gripper (motion ops already unlock- and state-gated above) --- */
    case OP_GRIP_CALIBRATE:  /* full calibration, overwrites the stored one */
        reply_ack(c->seq, c->op, gripper_calibrate() ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;
    case OP_GRIP_HOME:       /* open-stop check + stored calibration (full one if none) */
        reply_ack(c->seq, c->op, gripper_home() ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;
    case OP_GRIP_SET_CLOSE:
        reply_ack(c->seq, c->op, gripper_set_closing(c->f0, c->f1) ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;
    case OP_GRIP_GRIP:
        reply_ack(c->seq, c->op, gripper_grip(c->f0, c->f1) ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;
    case OP_GRIP_SET_FORCE:
        reply_ack(c->seq, c->op, gripper_set_default_force(c->f0) ? APP_PROTO_OK : APP_PROTO_ERR_ARG);
        break;
    case OP_GRIP_STOP:
        gripper_stop();
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_GRIP_GET_CALIB:
        emit_grip_calib(c->seq);
        break;
    case OP_GRIP_MOVE_DEG:
        reply_ack(c->seq, c->op, gripper_move_deg(c->f0, c->f1) ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;

    default:
        /* Includes the retired motor opcodes 0x10 / 0x12 / 0x13 / 0x14. */
        reply_ack(c->seq, c->op, APP_PROTO_ERR_OPCODE);
        break;
    }
}

/* Assemble everything the gripper needs from the latest snapshots. */
static void build_gripper_input(gripper_input_t *in, uint32_t dt_ms)
{
    memset(in, 0, sizeof(*in));
    TickType_t now = xTaskGetTickCount();
    in->dt_ms = dt_ms;
    in->allow_motion = motion_allowed();

    in->motor_ready = motor_twai_is_ready();
    in->motor_epoch = motor_twai_epoch();
    motor_twai_info_t mi;
    if (motor_twai_get_info(&mi)) {
        in->motor_tau_per_pu_est = mi.tau_per_pu_est;
    }
    /* Freshness from the frame's own receive stamp, not from when this loop
     * dequeued it: a loop that ran late must not make old data look new. */
    uint32_t now_ms = (uint32_t)(now * portTICK_PERIOD_MS);
    in->motor_fresh = s_have_motor && now_ms - s_last_motor.t_ms <= GRIP_MOTOR_FRESH_MS;
    in->motor_seq = s_last_motor.seq;
    in->motor_enabled = s_last_motor.enabled;
    in->motor_error = s_last_motor.error_code;
    in->pos = s_last_motor.position;
    in->vel = s_last_motor.velocity;
    in->torque = s_last_motor.torque;

    for (uint8_t i = 0; i < HAPTIC_SENSOR_COUNT && i < GRIPPER_TACTILE_COUNT; i++) {
        in->tac_ok[i] = s_have_haptic[i] && haptic_sample_age_ms(i) <= GRIP_TACTILE_FRESH_MS &&
                        s_last_haptic[i].status == HAPTIC_SLOT_OK;
        in->tac_seq[i] = s_last_haptic[i].seq;
        in->fz[i] = s_last_haptic[i].resultant[2];
        in->fz_point_max[i] = s_point_max[i];
    }
    bool tare_ok = false;
    in->tare_count = haptic_uart_tare_count(&tare_ok);
    in->tare_ok = tare_ok;
}

static void supervisor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "supervisor_task running (IMU + haptic + gripper)");

    app_cmd_t cmd;
    int  imu_silence_ms = 0;
    unsigned print_n = 0;
    TickType_t next_wake = xTaskGetTickCount();
    TickType_t last_tick = next_wake;
    /* Only watchdog the IMU once it has actually produced data, so a board
     * with no IMU attached reports nothing rather than faulting forever. */
    bool imu_seen = false;
    bool powon_policy_done = false;
    TickType_t next_powon_policy_check = xTaskGetTickCount();

#if IMU_PRINT_EVERY_N > 0
    ESP_LOGI(TAG, "     ax      ay      az (g) |       gx       gy       gz (deg/s) |    roll   pitch (deg)");
#endif

    for (;;) {
        /* Apply stream enable/disable before draining the next IMU batch. This
         * makes a received STOP command take effect before another USB packet
         * is emitted, instead of one supervisor period later. */
        while (xQueueReceive(s_bus->ctrl_q, &cmd, 0) == pdTRUE) {
            handle_cmd(&cmd);
        }

        bool got_imu = false;

        /* Sole consumer of imu_q. Accumulate every 100 Hz sample received during
         * this supervisor period and emit them together. */
        imu_sample_t imu_batch[APP_PROTO_IMU_BATCH_MAX];
        size_t imu_batch_count = 0;
        while (xQueueReceive(s_bus->imu_q, &s_last_imu, 0) == pdTRUE) {
            got_imu = true;
            s_have_imu = true;
            s_last_imu_tick = xTaskGetTickCount();
            imu_batch[imu_batch_count++] = s_last_imu;
            if (imu_batch_count == APP_PROTO_IMU_BATCH_MAX) {
                if (s_stream_imu) emit_imu_batch(imu_batch, imu_batch_count);
                imu_batch_count = 0;
            }
#if IMU_PRINT_EVERY_N > 0
            if (s_stream_imu && ++print_n >= IMU_PRINT_EVERY_N) {
                print_n = 0;
                ESP_LOGI(TAG, "%7.3f %7.3f %7.3f | %8.2f %8.2f %8.2f | %7.2f %7.2f",
                         s_last_imu.accel[0], s_last_imu.accel[1], s_last_imu.accel[2],
                         s_last_imu.gyro[0],  s_last_imu.gyro[1],  s_last_imu.gyro[2],
                         s_last_imu.roll,     s_last_imu.pitch);
            }
#endif
        }
        if (s_stream_imu && imu_batch_count > 0) {
            emit_imu_batch(imu_batch, imu_batch_count);
        }

        /* Each tactile app_proto frame carries one complete 77-point module. */
        haptic_sample_t haptic;
        while (xQueueReceive(s_bus->haptic_q, &haptic, 0) == pdTRUE) {
            if (haptic.sensor_index >= HAPTIC_SENSOR_COUNT) continue;
            uint8_t index = haptic.sensor_index;
            s_last_haptic[index] = haptic;
            s_have_haptic[index] = true;
            /* consecutive samples at or above the last-resort limit */
            s_over_n[index] = haptic.resultant[2] >= s_overforce_n
                              ? (uint8_t)(s_over_n[index] < 255 ? s_over_n[index] + 1 : 255) : 0;
            s_last_haptic_tick[index] = xTaskGetTickCount();
            uint8_t peak = 0;
            for (size_t p = 0; p < HAPTIC_POINT_COUNT; p++) {
                if (haptic.fz[p] > peak) peak = haptic.fz[p];
            }
            s_point_max[index] = (float)peak * 0.1f;   /* 0.1 N per LSB */
            if (s_stream_haptic) emit_haptic(index, s_stream_haptic);
        }

        /* Motor feedback (100 Hz while enabled, 20 Hz polls while disabled). */
        motor_status_t ms;
        while (xQueueReceive(s_bus->motor_status_q, &ms, 0) == pdTRUE) {
            s_last_motor = ms;
            s_have_motor = true;
            if (!s_boot_fault_checked) {
                s_boot_fault_checked = true;
                if (ms.error_code == MOTOR_ERR_COMM_LOST) {
                    /* This board restarted (flash, reset) while the motor was
                     * enabled: the drive stopped on the missing commands and
                     * is already off. Not a drive fault; clear it once. Bench
                     * 2026-10-23: "fault code 0xD" 1.2 s after every flash. */
                    ESP_LOGW(TAG, "motor reports 0xD (CAN commands lost) at start-up: this board restarted "
                                  "while the motor was enabled. Cleared; the motor stays off");
                    motor_twai_clear_error();
                    s_boot_clear_until = xTaskGetTickCount() + pdMS_TO_TICKS(BOOT_CLEAR_GRACE_MS);
                    continue;
                }
            }
            if (ms.error_code == MOTOR_ERR_COMM_LOST && s_boot_clear_until != 0 &&
                (int32_t)(xTaskGetTickCount() - s_boot_clear_until) < 0) {
                continue;   /* frames sent before the clear took effect */
            }
            if (ms.error_code != 0 && !s_motor_fault) {
                ESP_LOGE(TAG, "motor reports fault code 0x%X - motor off until MOTOR_CLEAR_ERR",
                         ms.error_code);
                gripper_abort();
                motor_twai_disable();
                s_motor_fault = true;
                enter_state(APP_STATE_ERROR);
            }
        }

        /* Last-resort force limit, independent of the gripper logic: the
         * gripper backs off at a lower force; if a pad still exceeds this
         * while the motor is enabled, switch the motor off. */
        if (!s_safe_stop && s_have_motor && s_last_motor.enabled) {
            for (uint8_t i = 0; i < HAPTIC_SENSOR_COUNT; i++) {
                if (s_have_haptic[i] && haptic_sample_age_ms(i) <= GRIP_TACTILE_FRESH_MS &&
                    s_over_n[i] >= OVERFORCE_SAMPLES) {
                    ESP_LOGE(TAG, "pad %u at %.1f N >= %.1f N", i,
                             s_last_haptic[i].resultant[2], s_overforce_n);
                    safe_stop("over-force");
                    break;
                }
            }
        }

        /* Gripper control step. */
        TickType_t now_tick = xTaskGetTickCount();
        uint32_t dt_ms = (uint32_t)((now_tick - last_tick) * portTICK_PERIOD_MS);
        last_tick = now_tick;
        gripper_input_t gin;
        build_gripper_input(&gin, dt_ms);
        gripper_tick(&gin);

        /* Liveness: fault only on an IMU that was seen alive and then went
         * quiet — never on one that has never streamed. Reported only. */
        bool calibrating = imu_uart_is_calibrating();
        bool acquisition_enabled = imu_uart_acquisition_enabled();
        if (got_imu) imu_seen = true;
        imu_silence_ms = (!acquisition_enabled || got_imu || calibrating)
                             ? 0 : imu_silence_ms + LOOP_PERIOD_MS;
        if (acquisition_enabled && imu_seen && imu_silence_ms > IMU_TIMEOUT_MS &&
            s_state != APP_STATE_ERROR && s_state != APP_STATE_SAFE_STOP) {
            ESP_LOGE(TAG, "IMU silent for %d ms", imu_silence_ms);
            s_have_imu = false;
            enter_state(APP_STATE_ERROR);
        } else if (got_imu && !s_motor_fault && !s_safe_stop &&
                   (s_state == APP_STATE_ERROR || s_state == APP_STATE_INIT)) {
            if (s_state == APP_STATE_INIT) ESP_LOGI(TAG, "first complete IMU sample");
            else ESP_LOGI(TAG, "IMU frames resumed");
            enter_state(APP_STATE_IDLE);
        }

        /* ACTIVE while the gripper is moving the jaw; reporting only. */
        if (s_state == APP_STATE_IDLE && gripper_is_busy()) {
            enter_state(APP_STATE_ACTIVE);
        } else if (s_state == APP_STATE_ACTIVE && !gripper_is_busy()) {
            enter_state(APP_STATE_IDLE);
        }

        /* Apply the saved product preference (default off on first boot). */
        TickType_t now = xTaskGetTickCount();
        if (got_imu && !powon_policy_done &&
            (int)(now - next_powon_policy_check) >= 0) {
            powon_policy_done = imu_uart_ensure_powon_output(s_powon_output);
            if (!powon_policy_done) {
                next_powon_policy_check = now + pdMS_TO_TICKS(5000);
            }
        }

        /* Fixed-period scheduling keeps the cadence at 1/LOOP_PERIOD_MS rather
         * than adding this pass's processing time to every delay. */
        xTaskDelayUntil(&next_wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

int app_supervisor_start(const app_bus_t *bus, const app_supervisor_cfg_t *cfg)
{
    if (!bus || !bus->imu_q || !bus->haptic_q || !bus->ctrl_q || !bus->motor_status_q) {
        return ESP_ERR_INVALID_ARG;
    }
    s_bus = bus;
    s_stream_imu = 0;      /* telemetry is opt-in after every ESP32 boot */
    s_stream_haptic = 0;
    s_unlock_mask = 0;
    s_have_imu = false;
    memset(s_have_haptic, 0, sizeof(s_have_haptic));
    memset(s_last_haptic_tick, 0, sizeof(s_last_haptic_tick));
    memset(s_point_max, 0, sizeof(s_point_max));
    s_last_imu_tick = 0;
    s_powon_output = load_powon_preference();
    if (cfg && cfg->overforce_n > 0.0f) s_overforce_n = cfg->overforce_n;

    int task_stack = cfg ? cfg->task_stack : 0;
    int task_prio  = cfg ? cfg->task_prio  : 0;

    uint32_t   stack = task_stack > 0 ? (uint32_t)task_stack : 6144;
    UBaseType_t prio = task_prio  > 0 ? (UBaseType_t)task_prio : 5;
    if (xTaskCreate(supervisor_task, "supervisor", stack, NULL, prio, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create supervisor_task");
        return ESP_FAIL;
    }
    return ESP_OK;
}
