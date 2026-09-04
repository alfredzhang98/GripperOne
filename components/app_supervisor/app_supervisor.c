/**
 * @file app_supervisor.c
 * @brief Command dispatch + IMU/haptic telemetry — sensor bring-up build.
 *
 * Scoped down to the IMU and two tactile modules. What remains out for now:
 * the motor/gripper command cases, the over-force
 * SAFE_STOP, gripper_tick(), and the session-unlock gate (it existed only to
 * guard motor actuation — with no motor reachable from here there is nothing
 * for it to guard, and a half-wired gate is worse than none).
 *
 * Restoring a subsystem means restoring three things together: its cases in
 * handle_cmd(), its queue drain in supervisor_task(), and its entry in this
 * component's CMakeLists REQUIRES.
 */
#include "app_supervisor.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"

#include "app_proto.h"
#include "imu_uart.h"
#include "haptic_uart.h"

static const char *TAG = "supervisor";

#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 1

#define LOOP_PERIOD_MS      20
#define IMU_TIMEOUT_MS      1500  /* allows one in-driver wake/reconfigure attempt */
#define IMU_DATA_FRESH_MS    500  /* stream/get require a recently received sample */
#define HAPTIC_DATA_FRESH_MS 500

/* Console echo of the decoded sample: print one in every N, 0 = silent.
 *
 * The active transport shares USB-Serial/JTAG with ESP_LOG. This bring-up build
 * keeps console output far below the 100 Hz sensor stream. The binary USB
 * telemetry still contains every sample; this affects human-readable logs only. */
#ifndef IMU_PRINT_EVERY_N
#define IMU_PRINT_EVERY_N   10
#endif

static const app_bus_t     *s_bus = NULL;
static volatile app_state_t s_state = APP_STATE_INIT;

/* streaming state (the "user layer" runtime) */
static bool s_stream_imu = false;
static bool s_stream_haptic = false;
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

static bool imu_data_fresh(void)
{
    TickType_t now = xTaskGetTickCount();
    return s_have_imu && s_last_imu_tick != 0 &&
           now - s_last_imu_tick <= pdMS_TO_TICKS(IMU_DATA_FRESH_MS);
}

static bool haptic_data_fresh(uint8_t sensor_index)
{
    if (sensor_index >= HAPTIC_SENSOR_COUNT) return false;
    TickType_t now = xTaskGetTickCount();
    return s_have_haptic[sensor_index] && s_last_haptic_tick[sensor_index] != 0 &&
           now - s_last_haptic_tick[sensor_index] <= pdMS_TO_TICKS(HAPTIC_DATA_FRESH_MS);
}

static bool both_haptic_fresh(void)
{
    return haptic_data_fresh(0) && haptic_data_fresh(1);
}

const volatile app_state_t *app_supervisor_state_ptr(void) { return &s_state; }

static void enter_state(app_state_t next)
{
    if (next != s_state) {
        ESP_LOGI(TAG, "state %s -> %s", app_state_name(s_state), app_state_name(next));
        s_state = next;
    }
}

/* ---- reply helpers ------------------------------------------------------ */
static void reply_ack(uint8_t seq, uint8_t op, uint8_t status)
{
    uint8_t f[APP_PROTO_HDR_LEN + 2 + 2];
    size_t n = app_proto_ack(f, sizeof(f), seq, op, status);
    if (n) app_proto_emit(f, n);
}

static void emit_imu(void)
{
    uint8_t f[APP_PROTO_MAX_FRAME];
    size_t n = app_proto_tele_imu(f, sizeof(f), &s_last_imu);
    if (n) app_proto_emit(f, n);
}

static void emit_imu_batch(const imu_sample_t *samples, size_t count)
{
    uint8_t f[APP_PROTO_MAX_FRAME];
    size_t n = app_proto_tele_imu_batch(f, sizeof(f), samples, count);
    if (n) app_proto_emit(f, n);
}

static void emit_haptic(uint8_t sensor_index)
{
    uint8_t f[APP_PROTO_MAX_FRAME];
    size_t n = app_proto_tele_haptic(f, sizeof(f), &s_last_haptic[sensor_index]);
    if (n) app_proto_emit(f, n);
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
    switch (c->op) {
    /* --- system --- */
    case OP_PING:
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_GET_VERSION: {
        uint8_t pl[4] = { OP_GET_VERSION, APP_PROTO_VERSION, FW_VERSION_MAJOR, FW_VERSION_MINOR };
        uint8_t f[APP_PROTO_HDR_LEN + sizeof(pl) + 2];
        size_t n = app_proto_build(f, sizeof(f), APP_PROTO_T_RESPONSE, c->seq, pl, sizeof(pl));
        if (n) app_proto_emit(f, n);
        break;
    }
    case OP_GET_STATE: {
        uint8_t f[APP_PROTO_HDR_LEN + 2 + 2];
        size_t n = app_proto_resp_state(f, sizeof(f), c->seq, (uint8_t)s_state);
        if (n) app_proto_emit(f, n);
        break;
    }

    /* --- IMU --- */
    case OP_IMU_STREAM_EN:
        if (!imu_data_fresh()) {
            ESP_LOGW(TAG, "cannot enable USB telemetry: no fresh IMU data");
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        } else {
            s_stream_imu = true;
            ESP_LOGI(TAG, "USB IMU telemetry enabled");
            reply_ack(c->seq, c->op, APP_PROTO_OK);
        }
        break;
    case OP_IMU_STREAM_DIS:
        s_stream_imu = false;
        ESP_LOGI(TAG, "USB IMU telemetry stopped (sensor acquisition remains active)");
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
            s_stream_haptic = true;
            ESP_LOGI(TAG, "USB haptic telemetry enabled");
            reply_ack(c->seq, c->op, APP_PROTO_OK);
        }
        break;
    case OP_HAPTIC_STREAM_DIS:
        s_stream_haptic = false;
        ESP_LOGI(TAG, "USB haptic telemetry stopped (sensor polling unchanged)");
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_HAPTIC_GET:
        if (both_haptic_fresh()) {
            emit_haptic(0);
            emit_haptic(1);
        } else {
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        }
        break;
    case OP_HAPTIC_CALIBRATE:
        reply_ack(c->seq, c->op,
                  haptic_uart_calibrate() ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;
    case OP_HAPTIC_ACQ_EN:
        reply_ack(c->seq, c->op,
                  haptic_uart_set_acquisition(true) ? APP_PROTO_OK : APP_PROTO_ERR_STATE);
        break;
    case OP_HAPTIC_ACQ_DIS:
        if (haptic_uart_set_acquisition(false)) {
            memset(s_have_haptic, 0, sizeof(s_have_haptic));
            reply_ack(c->seq, c->op, APP_PROTO_OK);
        } else {
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        }
        break;

    default:
        /* Includes every motor/gripper opcode: they are still defined in
         * app_protocol.h, but nothing here can execute them in this build. */
        reply_ack(c->seq, c->op, APP_PROTO_ERR_OPCODE);
        break;
    }
}

static void supervisor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "supervisor_task running (IMU + haptic)");

    app_cmd_t cmd;
    int  imu_silence_ms = 0;
    unsigned print_n = 0;
    TickType_t next_wake = xTaskGetTickCount();
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
         * this 20 ms supervisor period and emit them together. Normal traffic is
         * therefore 50 USB packets/s with two samples/packet, while the host
         * still reconstructs the complete 100 Hz stream. */
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
            /* These are USB console lines, not protocol telemetry. Gate them
             * with the same switch so STOP makes the host link visibly quiet. */
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
            s_last_haptic_tick[index] = xTaskGetTickCount();
            if (s_stream_haptic) emit_haptic(index);
        }

        /* Liveness: fault only on an IMU that was seen alive and then went
         * quiet — never on one that has never streamed. With no actuator to
         * stop, ERROR is purely a report, and recovers on its own when frames
         * come back. */
        bool calibrating = imu_uart_is_calibrating();
        bool acquisition_enabled = imu_uart_acquisition_enabled();
        if (got_imu) imu_seen = true;
        imu_silence_ms = (!acquisition_enabled || got_imu || calibrating)
                             ? 0 : imu_silence_ms + LOOP_PERIOD_MS;
        if (acquisition_enabled && imu_seen && imu_silence_ms > IMU_TIMEOUT_MS &&
            s_state != APP_STATE_ERROR) {
            ESP_LOGE(TAG, "IMU silent for %d ms", imu_silence_ms);
            s_have_imu = false;
            enter_state(APP_STATE_ERROR);
        } else if (got_imu && (s_state == APP_STATE_ERROR || s_state == APP_STATE_INIT)) {
            if (s_state == APP_STATE_INIT) ESP_LOGI(TAG, "first complete IMU sample");
            else ESP_LOGI(TAG, "IMU frames resumed");
            enter_state(APP_STATE_IDLE);
        }

        /* Apply the saved product preference (default off on first boot). Once
         * this firmware has identified the unit, RSW/RRATE explicitly start the
         * required IMU -> ESP32 session. Read before saving so normal boots do
         * not wear the IMU flash. OP_IMU_POWON_SET updates this preference. */
        TickType_t now = xTaskGetTickCount();
        if (got_imu && !powon_policy_done &&
            (int)(now - next_powon_policy_check) >= 0) {
            powon_policy_done = imu_uart_ensure_powon_output(s_powon_output);
            if (!powon_policy_done) {
                next_powon_policy_check = now + pdMS_TO_TICKS(5000);
            }
        }

        /* Fixed-period scheduling keeps the USB report cadence at 50 Hz rather
         * than adding processing time to every 20 ms delay. */
        xTaskDelayUntil(&next_wake, pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

int app_supervisor_start(const app_bus_t *bus, const app_supervisor_cfg_t *cfg)
{
    if (!bus || !bus->imu_q || !bus->haptic_q || !bus->ctrl_q) {
        return ESP_ERR_INVALID_ARG;
    }
    s_bus = bus;
    s_stream_imu = false;  /* USB telemetry is opt-in after every ESP32 boot. */
    s_stream_haptic = false;
    s_have_imu = false;
    memset(s_have_haptic, 0, sizeof(s_have_haptic));
    memset(s_last_haptic_tick, 0, sizeof(s_last_haptic_tick));
    s_last_imu_tick = 0;
    s_powon_output = load_powon_preference();

    int task_stack = cfg ? cfg->task_stack : 0;
    int task_prio  = cfg ? cfg->task_prio  : 0;

    uint32_t   stack = task_stack > 0 ? (uint32_t)task_stack : 4096;
    UBaseType_t prio = task_prio  > 0 ? (UBaseType_t)task_prio : 5;
    if (xTaskCreate(supervisor_task, "supervisor", stack, NULL, prio, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create supervisor_task");
        return ESP_FAIL;
    }
    return ESP_OK;
}
