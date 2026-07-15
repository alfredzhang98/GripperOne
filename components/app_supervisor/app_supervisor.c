#include "app_supervisor.h"

#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "app_proto.h"
#include "motor_twai.h"
#include "imu_uart.h"
#include "haptic_uart.h"
#include "gripper.h"

static const char *TAG = "supervisor";

#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 1

#define LOOP_PERIOD_MS      20
#define HAPTIC_TIMEOUT_MS   500
#define IMU_TIMEOUT_MS      500
#define DEFAULT_TELEM_HZ    20
/* Built-in default for the per-module SAFE_STOP force limit; overridable via
 * app_supervisor_cfg_t.overforce_n (set in main from app_config.h). */
#define OVERFORCE_N_DEFAULT 40.0f

static const app_bus_t     *s_bus = NULL;
static volatile app_state_t s_state = APP_STATE_INIT;
static float                s_overforce_n = OVERFORCE_N_DEFAULT;

/* session + streaming state (the "user layer" runtime) */
static bool s_unlocked      = false;
static bool s_stream_imu    = false;
static bool s_stream_haptic = false;
static bool s_stream_motor  = false;
static int  s_telem_hz      = DEFAULT_TELEM_HZ;

/* latest snapshots for telemetry / GET */
static imu_sample_t    s_last_imu;
static haptic_sample_t s_last_haptic;
static motor_status_t  s_last_motor;

const volatile app_state_t *app_supervisor_state_ptr(void) { return &s_state; }

/* Largest per-module normal force magnitude in a sample, Newtons. A single
 * hard contact is the crush hazard, so watch the peak module rather than the
 * (possibly self-cancelling) sum. */
static float haptic_peak_force(const haptic_sample_t *h)
{
    float peak = 0.0f;
    uint16_t n = h->module_count > HAPTIC_MAX_MODULES ? HAPTIC_MAX_MODULES : h->module_count;
    for (uint16_t i = 0; i < n; i++) {
        float a = fabsf(h->force[i].fz);
        if (a > peak) peak = a;
    }
    return peak;
}

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

static void emit_imu(void)    { uint8_t f[APP_PROTO_MAX_FRAME]; size_t n = app_proto_tele_imu(f, sizeof(f), &s_last_imu);       if (n) app_proto_emit(f, n); }
static void emit_haptic(void) { uint8_t f[APP_PROTO_MAX_FRAME]; size_t n = app_proto_tele_haptic(f, sizeof(f), &s_last_haptic); if (n) app_proto_emit(f, n); }
static void emit_motor(void)  { uint8_t f[APP_PROTO_MAX_FRAME]; size_t n = app_proto_tele_motor(f, sizeof(f), &s_last_motor);   if (n) app_proto_emit(f, n); }

/* Auto-enable the stream for whichever subsystem a command targets. This is the
 * "自动启用对应的功能" behaviour: a client that talks to a subsystem starts
 * getting its telemetry without a separate enable step. STREAM_DIS opts out. */
static void auto_enable(uint8_t op)
{
    if (op >= 0x10 && op <= 0x16) s_stream_motor  = true;
    if (op >= 0x40 && op <= 0x47) s_stream_motor  = true;  /* gripper drives the motor */
    if (op >= 0x20 && op <= 0x26 && op != OP_IMU_STREAM_DIS)    s_stream_imu    = true;
    if (op >= 0x30 && op <= 0x34 && op != OP_HAPTIC_STREAM_DIS) s_stream_haptic = true;
}

static void run_self_calibrate(void)
{
    ESP_LOGI(TAG, "self-calibration: IMU + haptic + motor zero");
    imu_uart_cal_accel();
    haptic_uart_calibrate();
    motor_twai_save_zero();
}

/* Execute one decoded user command. */
static void handle_cmd(const app_cmd_t *c)
{
    /* Motor actuation (0x10..0x15); 0x16 GET_STATUS is a read, not gated. */
    const bool motor_act_op = (c->op >= 0x10 && c->op <= 0x15);
    /* Gripper motion ops (0x40..0x45) actuate the motor; 0x46 STOP / 0x47 read. */
    const bool grip_move_op = (c->op >= 0x40 && c->op <= 0x45);
    /* Gripper ops that physically drive the jaw (exclude SET_FORCE 0x45). */
    const bool grip_motion_op = (c->op >= 0x40 && c->op <= 0x44);

    /* Gate motor/gripper actuation and self-calibration behind an unlocked session. */
    if ((motor_act_op || grip_move_op || c->op == OP_SELF_CALIBRATE) && !s_unlocked) {
        ESP_LOGW(TAG, "op 0x%02x rejected: session locked", c->op);
        reply_ack(c->seq, c->op, APP_PROTO_ERR_LOCKED);
        return;
    }
    /* Never let a command re-energize/move the motor out of ERROR/SAFE_STOP.
     * DISABLE/CLEAR_ERR are deliberately NOT blocked so recovery is possible. */
    if ((c->op == OP_MOTOR_ENABLE || grip_motion_op) &&
        (s_state == APP_STATE_ERROR || s_state == APP_STATE_SAFE_STOP)) {
        ESP_LOGW(TAG, "op 0x%02x rejected: state %s", c->op, app_state_name(s_state));
        reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        return;
    }

    auto_enable(c->op);

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
    case OP_SET_TELEM_RATE:
        s_telem_hz = (c->u0 == 0) ? DEFAULT_TELEM_HZ : (int)c->u0;
        /* The control loop runs at 1000/LOOP_PERIOD_MS Hz; telemetry can't
         * exceed that, so clamp to the actually-achievable rate. */
        if (s_telem_hz > (1000 / LOOP_PERIOD_MS)) s_telem_hz = 1000 / LOOP_PERIOD_MS;
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_SAFE_STOP:
        motor_twai_disable();
        gripper_stop();                 /* clear any GRIPPING/HOLDING sub-state */
        enter_state(APP_STATE_SAFE_STOP);
        reply_ack(c->seq, c->op, APP_PROTO_OK);   /* always allowed */
        break;
    case OP_SESSION_UNLOCK:
        s_unlocked = (c->u0 == APP_PROTO_SESSION_KEY);
        reply_ack(c->seq, c->op, s_unlocked ? APP_PROTO_OK : APP_PROTO_ERR_ARG);
        break;
    case OP_SESSION_LOCK:
        s_unlocked = false;
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_SELF_CALIBRATE:
        run_self_calibrate();
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;

    /* --- motor (already unlock-gated above) --- */
    case OP_MOTOR_ENABLE:
        if (s_state == APP_STATE_ERROR || s_state == APP_STATE_SAFE_STOP) {
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        } else {
            motor_twai_enable();
            enter_state(APP_STATE_ACTIVE);
            reply_ack(c->seq, c->op, APP_PROTO_OK);
        }
        break;
    case OP_MOTOR_DISABLE:
        motor_twai_disable();
        enter_state(APP_STATE_IDLE);
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_MOTOR_SET_TARGET:
        if (s_state == APP_STATE_ACTIVE) {
            motor_twai_set_target(c->f0);
            reply_ack(c->seq, c->op, APP_PROTO_OK);
        } else {
            reply_ack(c->seq, c->op, APP_PROTO_ERR_STATE);
        }
        break;
    case OP_MOTOR_SET_GAINS:
        motor_twai_set_gains(c->f0, c->f1);
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_MOTOR_SAVE_ZERO:
        motor_twai_save_zero();
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_MOTOR_CLEAR_ERR:
        motor_twai_clear_error();
        /* Clearing a fault is also the way out of ERROR or a safety stop. */
        if (s_state == APP_STATE_ERROR || s_state == APP_STATE_SAFE_STOP) {
            enter_state(APP_STATE_IDLE);
        }
        reply_ack(c->seq, c->op, APP_PROTO_OK);
        break;
    case OP_MOTOR_GET_STATUS:
        emit_motor();
        break;

    /* --- IMU --- */
    case OP_IMU_STREAM_EN:  s_stream_imu = true;  reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_IMU_STREAM_DIS: s_stream_imu = false; reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_IMU_GET:        emit_imu(); break;
    case OP_IMU_CALIBRATE:  imu_uart_cal_accel(); reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_IMU_SET_ZERO:   imu_uart_set_zero();  reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_IMU_SET_REF:    imu_uart_set_angle_ref();      reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_IMU_SET_AXIS:   imu_uart_set_axis(c->u0 != 0); reply_ack(c->seq, c->op, APP_PROTO_OK); break;

    /* --- haptic --- */
    case OP_HAPTIC_STREAM_EN:  s_stream_haptic = true;  haptic_uart_stream(true);  reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_HAPTIC_STREAM_DIS: s_stream_haptic = false;                            reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_HAPTIC_GET:        emit_haptic(); break;
    case OP_HAPTIC_SET_CONTENT:haptic_uart_set_content((uint8_t)c->u0); reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_HAPTIC_CALIBRATE:  haptic_uart_calibrate(); reply_ack(c->seq, c->op, APP_PROTO_OK); break;

    /* --- gripper high-level app --- */
    case OP_GRIP_CALIBRATE: gripper_calibrate();          reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_GRIP_HOME:      gripper_home();               reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_GRIP_SET_OPEN:  gripper_set_opening(c->f0);   reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_GRIP_GRIP:      gripper_grip(c->f0);          reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_GRIP_RELEASE:   gripper_release();            reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_GRIP_SET_FORCE: gripper_set_grip_force(c->f0);reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_GRIP_STOP:      gripper_stop();               reply_ack(c->seq, c->op, APP_PROTO_OK); break;
    case OP_GRIP_GET_CALIB: {
        gripper_calib_t cal; gripper_get_calib(&cal);
        uint8_t pl[1 + 1 + 3 * 4 + 1];
        size_t k = 0;
        pl[k++] = OP_GRIP_GET_CALIB;
        pl[k++] = cal.valid ? 1 : 0;
        memcpy(&pl[k], &cal.pos_closed, 4);   k += 4;
        memcpy(&pl[k], &cal.pos_open, 4);     k += 4;
        memcpy(&pl[k], &cal.grip_force_n, 4); k += 4;
        pl[k++] = (uint8_t)gripper_get_state();
        uint8_t f[APP_PROTO_HDR_LEN + sizeof(pl) + 2];
        size_t n = app_proto_build(f, sizeof(f), APP_PROTO_T_RESPONSE, c->seq, pl, (uint16_t)k);
        if (n) app_proto_emit(f, n);
        break;
    }

    default:
        reply_ack(c->seq, c->op, APP_PROTO_ERR_OPCODE);
        break;
    }
}

static void supervisor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "supervisor_task running");

    app_cmd_t cmd;
    int haptic_silence_ms = 0, imu_silence_ms = 0, telem_accum_ms = 0;
    /* Only watchdog a sensor once it has actually produced data. A sensor that
     * was never present (e.g. bench-testing the motor alone) must not fault. */
    bool haptic_seen = false, imu_seen = false;

    enter_state(APP_STATE_IDLE);

    for (;;) {
        bool got_haptic = false, got_imu = false;

        while (xQueueReceive(s_bus->haptic_q, &s_last_haptic, 0) == pdTRUE) {
            got_haptic = true;
            /* Hard over-force cutoff: if the motor is driving and a tactile
             * module reports a force well above any normal grip, stop before we
             * crush the object (or the mechanism). */
            float peak = haptic_peak_force(&s_last_haptic);
            if (peak > s_overforce_n && s_last_motor.enabled &&
                s_state != APP_STATE_SAFE_STOP && s_state != APP_STATE_ERROR) {
                ESP_LOGE(TAG, "over-force %.1f N > %.1f N -> SAFE_STOP", peak, s_overforce_n);
                motor_twai_disable();
                gripper_stop();
                enter_state(APP_STATE_SAFE_STOP);
            }
        }
        while (xQueueReceive(s_bus->imu_q, &s_last_imu, 0) == pdTRUE) {
            got_imu = true;
        }
        while (xQueueReceive(s_bus->motor_status_q, &s_last_motor, 0) == pdTRUE) {
            if (s_last_motor.error_code != 0) {
                ESP_LOGW(TAG, "motor error_code=%u", s_last_motor.error_code);
                gripper_stop();          /* drop any GRIPPING/HOLDING sub-state */
                enter_state(APP_STATE_ERROR);
            }
        }

        while (xQueueReceive(s_bus->ctrl_q, &cmd, 0) == pdTRUE) {
            handle_cmd(&cmd);
        }

        /* advance the high-level gripper state machine with fresh feedback */
        gripper_tick(&s_last_motor, &s_last_haptic);

        /* periodic telemetry for whichever streams are enabled */
        telem_accum_ms += LOOP_PERIOD_MS;
        int telem_period = 1000 / (s_telem_hz > 0 ? s_telem_hz : DEFAULT_TELEM_HZ);
        if (telem_accum_ms >= telem_period) {
            telem_accum_ms = 0;
            if (s_stream_imu)    emit_imu();
            if (s_stream_haptic) emit_haptic();
            if (s_stream_motor)  emit_motor();
        }

        /* sensor-liveness watchdog: only fault on a sensor that was seen alive
         * and then went silent — never on one that has never streamed (bench). */
        if (got_haptic) haptic_seen = true;
        if (got_imu)    imu_seen    = true;
        haptic_silence_ms = got_haptic ? 0 : haptic_silence_ms + LOOP_PERIOD_MS;
        imu_silence_ms    = got_imu    ? 0 : imu_silence_ms + LOOP_PERIOD_MS;
        bool haptic_lost = haptic_seen && haptic_silence_ms > HAPTIC_TIMEOUT_MS;
        bool imu_lost    = imu_seen    && imu_silence_ms    > IMU_TIMEOUT_MS;
        if (s_state == APP_STATE_ACTIVE && (haptic_lost || imu_lost)) {
            ESP_LOGE(TAG, "sensor timeout (haptic=%dms imu=%dms) -> SAFE_STOP",
                     haptic_silence_ms, imu_silence_ms);
            motor_twai_disable();
            gripper_stop();
            enter_state(APP_STATE_SAFE_STOP);
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

int app_supervisor_start(const app_bus_t *bus, const app_supervisor_cfg_t *cfg)
{
    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }
    s_bus = bus;

    int  task_stack = cfg ? cfg->task_stack : 0;
    int  task_prio  = cfg ? cfg->task_prio  : 0;
    if (cfg && cfg->overforce_n > 0.0f) s_overforce_n = cfg->overforce_n;

    uint32_t   stack = task_stack > 0 ? (uint32_t)task_stack : 4096;
    UBaseType_t prio = task_prio  > 0 ? (UBaseType_t)task_prio : 5;
    if (xTaskCreate(supervisor_task, "supervisor", stack, NULL, prio, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create supervisor_task");
        return ESP_FAIL;
    }
    return ESP_OK;
}
