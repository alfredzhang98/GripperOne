#include "gripper.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"

#include "motor_twai.h"

static const char *TAG = "gripper";

/* =========================================================================
 * Tunables. These are hardware-dependent placeholders — calibrate them once
 * the real jaw kinematics/gear ratio and motor direction are known.
 * ========================================================================= */
#define GRIPPER_TICK_MS       20      /* supervisor calls gripper_tick() at ~50 Hz */
#define GRIPPER_CLOSE_DIR    (+1.0f)  /* sign of motor position change that CLOSES the jaw. TODO confirm */

/* --- auto-calibration (stall against the hard stops) --- */
#define CAL_LEAD_RAD          0.010f  /* how far ahead of the jaw we command (bounds push force) */
#define CAL_KP                8.0f
#define CAL_KD                1.0f
#define CAL_STALL_VEL         0.05f   /* rad/s below which the jaw is "not moving" */
#define CAL_STALL_MS          250     /* stalled this long => hard stop reached */
#define CAL_TIMEOUT_MS        8000    /* safety timeout per phase */
#define CAL_BACKOFF_RAD       0.05f   /* back off this far from each hard stop */

/* --- position moves --- */
#define MOVE_KP               12.0f
#define MOVE_KD               1.5f
#define MOVE_SETTLE_VEL       0.05f
#define MOVE_POS_TOL          0.02f

/* --- gripping / force hold --- */
#define GRIP_LEAD_RAD         0.006f  /* closing ramp per tick while seeking contact */
#define GRIP_KP               6.0f
#define GRIP_KD               1.0f
#define GRIP_CONTACT_N        0.5f    /* tactile force that counts as "touched object" */
#define GRIP_FORCE_KP         0.02f   /* N·m of torque per N of force error (P loop) */
#define GRIP_HOLD_KD          0.5f
#define GRIP_MAX_TORQUE_NM    3.0f
#define GRIP_DEFAULT_FORCE_N  5.0f
#define GRIP_SEEK_TIMEOUT_MS  4000    /* give up seeking contact -> hold anyway */
#define MOVE_TIMEOUT_MS       4000    /* position move must settle within this  */

#define NVS_NS   "gripper"
#define NVS_KEY  "calib"

/* =========================================================================
 * State
 * ========================================================================= */
static gripper_state_t s_state = GRIPPER_UNCAL;
static gripper_calib_t s_calib;

static float s_target_pos    = 0.0f;   /* position setpoint (MOVING / IDLE hold) */
static bool  s_hold          = false;  /* whether IDLE actively holds s_target_pos */
static float s_grip_force    = GRIP_DEFAULT_FORCE_N;
static float s_grip_closed   = 0.0f;   /* position field used while force-holding */
static float s_hold_tff      = 0.0f;   /* torque-feedforward integrator (force loop) */

static float s_last_pos      = 0.0f;
static int   s_stall_ms      = 0;
static int   s_phase_ms      = 0;
static bool  s_seen_motion   = false;

/* =========================================================================
 * Helpers
 * ========================================================================= */
const char *gripper_state_name(gripper_state_t s)
{
    switch (s) {
    case GRIPPER_UNCAL:     return "UNCAL";
    case GRIPPER_IDLE:      return "IDLE";
    case GRIPPER_CAL_CLOSE: return "CAL_CLOSE";
    case GRIPPER_CAL_OPEN:  return "CAL_OPEN";
    case GRIPPER_MOVING:    return "MOVING";
    case GRIPPER_GRIPPING:  return "GRIPPING";
    case GRIPPER_HOLDING:   return "HOLDING";
    case GRIPPER_ERROR:     return "ERROR";
    default:                return "?";
    }
}

/* Total normal (Fz) force across all tactile modules, Newtons. */
static float haptic_total_force(const haptic_sample_t *h)
{
    if (!h) return 0.0f;
    float sum = 0.0f;
    uint16_t n = h->module_count > HAPTIC_MAX_MODULES ? HAPTIC_MAX_MODULES : h->module_count;
    for (uint16_t i = 0; i < n; i++) {
        sum += h->force[i].fz;
    }
    return sum;
}

static void calib_load(void)
{
    memset(&s_calib, 0, sizeof(s_calib));
    s_calib.grip_force_n = GRIP_DEFAULT_FORCE_N;

    nvs_handle_t nh;
    if (nvs_open(NVS_NS, NVS_READONLY, &nh) == ESP_OK) {
        gripper_calib_t tmp;
        size_t sz = sizeof(tmp);
        if (nvs_get_blob(nh, NVS_KEY, &tmp, &sz) == ESP_OK && sz == sizeof(tmp)) {
            s_calib = tmp;
            ESP_LOGI(TAG, "calibration loaded: closed=%.3f open=%.3f force=%.1fN",
                     s_calib.pos_closed, s_calib.pos_open, s_calib.grip_force_n);
        }
        nvs_close(nh);
    }
    s_grip_force = s_calib.grip_force_n;
}

static void calib_save(void)
{
    nvs_handle_t nh;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nh);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed (%s), calibration not persisted", esp_err_to_name(err));
        return;
    }
    err = nvs_set_blob(nh, NVS_KEY, &s_calib, sizeof(s_calib));
    if (err == ESP_OK) {
        err = nvs_commit(nh);
    }
    nvs_close(nh);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "calibration saved to flash");
    } else {
        ESP_LOGW(TAG, "calibration save failed: %s", esp_err_to_name(err));
    }
}

static void set_state(gripper_state_t s)
{
    if (s != s_state) {
        ESP_LOGI(TAG, "%s -> %s", gripper_state_name(s_state), gripper_state_name(s));
        s_state = s;
    }
}

/* Clamp a closing command so it never drives past the calibrated closed stop
 * (direction-aware). Uncalibrated: returned unchanged (caller must bound it). */
static float clamp_to_closed(float cmd)
{
    if (!s_calib.valid) return cmd;
    if (GRIPPER_CLOSE_DIR > 0.0f) return cmd < s_calib.pos_closed ? cmd : s_calib.pos_closed;
    return cmd > s_calib.pos_closed ? cmd : s_calib.pos_closed;
}

/* Returns true when the jaw has stalled against a hard stop this phase. */
static bool stall_detect(float vel)
{
    if (fabsf(vel) > CAL_STALL_VEL) {
        s_seen_motion = true;
        s_stall_ms = 0;
        return false;
    }
    if (s_seen_motion) {
        s_stall_ms += GRIPPER_TICK_MS;
        return s_stall_ms >= CAL_STALL_MS;
    }
    return false;
}

/* =========================================================================
 * Public API
 * ========================================================================= */
int gripper_init(void)
{
    calib_load();
    s_state = s_calib.valid ? GRIPPER_IDLE : GRIPPER_UNCAL;
    s_hold = false;
    ESP_LOGI(TAG, "init, state=%s", gripper_state_name(s_state));
    return ESP_OK;
}

void gripper_calibrate(void)
{
    ESP_LOGI(TAG, "auto-calibration: seeking closed hard stop");
    motor_twai_enable();
    s_phase_ms = 0;
    s_stall_ms = 0;
    s_seen_motion = false;
    set_state(GRIPPER_CAL_CLOSE);
}

void gripper_home(void)
{
    if (!s_calib.valid) { ESP_LOGW(TAG, "home ignored: not calibrated"); return; }
    motor_twai_enable();
    s_target_pos = s_calib.pos_closed;
    s_hold = true;
    s_phase_ms = 0;
    set_state(GRIPPER_MOVING);
}

void gripper_set_opening(float frac)
{
    if (!s_calib.valid) { ESP_LOGW(TAG, "set_opening ignored: not calibrated"); return; }
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    motor_twai_enable();
    s_target_pos = s_calib.pos_closed + frac * (s_calib.pos_open - s_calib.pos_closed);
    s_hold = true;
    s_phase_ms = 0;
    set_state(GRIPPER_MOVING);
    ESP_LOGI(TAG, "opening -> %.0f%% (pos %.3f)", frac * 100.0f, s_target_pos);
}

void gripper_grip(float force_n)
{
    /* Require calibration: without a known closed stop there is nothing to
     * clamp the closing command against (crush/runaway risk). */
    if (!s_calib.valid) { ESP_LOGW(TAG, "grip ignored: not calibrated"); return; }
    s_grip_force = (force_n > 0.0f) ? force_n : s_calib.grip_force_n;
    s_grip_closed = s_calib.pos_closed;
    s_hold_tff = 0.0f;
    s_stall_ms = 0;
    s_seen_motion = false;
    s_phase_ms = 0;
    motor_twai_enable();
    set_state(GRIPPER_GRIPPING);
    ESP_LOGI(TAG, "grip @ %.1f N", s_grip_force);
}

void gripper_release(void)
{
    if (!s_calib.valid) { ESP_LOGW(TAG, "release ignored: not calibrated"); return; }
    motor_twai_enable();
    s_target_pos = s_calib.pos_open;
    s_hold = true;
    s_phase_ms = 0;
    set_state(GRIPPER_MOVING);
}

void gripper_set_grip_force(float n)
{
    if (n < 0.0f) n = 0.0f;
    s_calib.grip_force_n = n;
    s_grip_force = n;
    calib_save();
    ESP_LOGI(TAG, "default grip force = %.1f N (saved)", n);
}

void gripper_stop(void)
{
    s_target_pos = s_last_pos;
    s_hold = true;
    set_state(GRIPPER_IDLE);
}

gripper_state_t gripper_get_state(void) { return s_state; }

void gripper_get_calib(gripper_calib_t *out) { if (out) *out = s_calib; }

float gripper_get_opening(void)
{
    if (!s_calib.valid) return -1.0f;
    float span = s_calib.pos_open - s_calib.pos_closed;
    if (fabsf(span) < 1e-6f) return -1.0f;
    float f = (s_last_pos - s_calib.pos_closed) / span;
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    return f;
}

/* =========================================================================
 * Control tick
 * ========================================================================= */
void gripper_tick(const motor_status_t *m, const haptic_sample_t *h)
{
    float pos = m ? m->position : s_last_pos;
    float vel = m ? m->velocity : 0.0f;
    s_last_pos = pos;
    s_phase_ms += GRIPPER_TICK_MS;

    switch (s_state) {
    case GRIPPER_CAL_CLOSE: {
        float cmd = pos + GRIPPER_CLOSE_DIR * CAL_LEAD_RAD;   /* self-limiting: bounds torque */
        motor_twai_set_mit(cmd, 0.0f, CAL_KP, CAL_KD, 0.0f);
        if (stall_detect(vel)) {
            s_calib.pos_closed = pos - GRIPPER_CLOSE_DIR * CAL_BACKOFF_RAD;
            ESP_LOGI(TAG, "closed stop at %.3f -> zero=%.3f; seeking open stop",
                     pos, s_calib.pos_closed);
            s_phase_ms = 0; s_stall_ms = 0; s_seen_motion = false;
            set_state(GRIPPER_CAL_OPEN);
        } else if (s_phase_ms > CAL_TIMEOUT_MS) {
            ESP_LOGE(TAG, "cal close timeout (no motion/feedback) -> ERROR");
            motor_twai_disable();
            set_state(GRIPPER_ERROR);
        }
        break;
    }
    case GRIPPER_CAL_OPEN: {
        float cmd = pos - GRIPPER_CLOSE_DIR * CAL_LEAD_RAD;
        motor_twai_set_mit(cmd, 0.0f, CAL_KP, CAL_KD, 0.0f);
        if (stall_detect(vel)) {
            s_calib.pos_open = pos + GRIPPER_CLOSE_DIR * CAL_BACKOFF_RAD;
            s_calib.valid = true;
            calib_save();
            ESP_LOGI(TAG, "open stop at %.3f -> open=%.3f; calibration done",
                     pos, s_calib.pos_open);
            s_target_pos = s_calib.pos_open;
            s_hold = true;
            set_state(GRIPPER_IDLE);
        } else if (s_phase_ms > CAL_TIMEOUT_MS) {
            ESP_LOGE(TAG, "cal open timeout (no motion/feedback) -> ERROR");
            motor_twai_disable();
            set_state(GRIPPER_ERROR);   /* do NOT persist a degenerate range */
        }
        break;
    }
    case GRIPPER_MOVING:
        motor_twai_set_mit(s_target_pos, 0.0f, MOVE_KP, MOVE_KD, 0.0f);
        if (fabsf(pos - s_target_pos) < MOVE_POS_TOL && fabsf(vel) < MOVE_SETTLE_VEL) {
            set_state(GRIPPER_IDLE);
        } else if (s_phase_ms > MOVE_TIMEOUT_MS) {
            ESP_LOGW(TAG, "move did not settle; holding current position");
            s_target_pos = pos;             /* stop pushing toward an unreachable target */
            set_state(GRIPPER_IDLE);
        }
        break;

    case GRIPPER_GRIPPING: {
        /* Self-limiting closing command (like calibration): stay just ahead of
         * the actual position so torque is bounded, and never pass the
         * calibrated closed stop. */
        float cmd = clamp_to_closed(pos + GRIPPER_CLOSE_DIR * GRIP_LEAD_RAD);
        motor_twai_set_mit(cmd, 0.0f, GRIP_KP, GRIP_KD, 0.0f);
        float f = haptic_total_force(h);
        bool contact = f >= GRIP_CONTACT_N;
        bool jammed  = stall_detect(vel);          /* rigid object, no tactile pad */
        if (contact || jammed || s_phase_ms > GRIP_SEEK_TIMEOUT_MS) {
            s_grip_closed = pos;                   /* hold near where we made contact */
            s_hold_tff = 0.0f;
            ESP_LOGI(TAG, "contact (%.2f N%s), holding force", f, jammed ? ", stalled" : "");
            set_state(GRIPPER_HOLDING);
        }
        break;
    }
    case GRIPPER_HOLDING: {
        /* Constant-force hold: P loop on tactile force via torque feedforward. */
        float f = haptic_total_force(h);
        float err = s_grip_force - f;
        s_hold_tff += GRIP_FORCE_KP * err;
        if (s_hold_tff < 0.0f) s_hold_tff = 0.0f;
        if (s_hold_tff > GRIP_MAX_TORQUE_NM) s_hold_tff = GRIP_MAX_TORQUE_NM;
        motor_twai_set_mit(s_grip_closed, 0.0f, 0.0f, GRIP_HOLD_KD,
                           GRIPPER_CLOSE_DIR * s_hold_tff);
        break;
    }
    case GRIPPER_IDLE:
        if (s_hold) {
            motor_twai_set_mit(s_target_pos, 0.0f, MOVE_KP, MOVE_KD, 0.0f);
        }
        break;

    case GRIPPER_UNCAL:
    case GRIPPER_ERROR:
    default:
        break;   /* issue no command */
    }
}
