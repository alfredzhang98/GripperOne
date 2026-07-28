#include "gripper.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"

#include "motor_twai.h"

static const char *TAG = "gripper";

/* =========================================================================
 * Tunables. Calibrate these once the real jaw kinematics and motor direction
 * are known. Currents are per-unit (I/Imax) as required by force-position mode.
 * ========================================================================= */
#define GRIPPER_TICK_MS       20      /* supervisor calls gripper_tick() at ~50 Hz */
#define GRIPPER_CLOSE_DIR    (+1.0f)  /* sign of position change that CLOSES the jaw. TODO confirm */

#define TWO_PI                6.283185307f
#define RAD2DEG               57.29578f
#define DEG2RAD               0.017453292f

/* --- auto-calibration: creep into each hard stop at low current ---
 * START LOW. The tactile pads sit on the finger tips, so a hard bottom-out can
 * crush them. These are deliberately gentle: raise CAL_CURRENT_PU only if the
 * jaw cannot reach the stop at all. */
#define CAL_CURRENT_PU        0.08f   /* very gentle (measured: 0.15 pu ~ 0.25 Nm)  */
#define CAL_SPEED             1.0f    /* rad/s while seeking                        */
#define CAL_LEAD_RAD          0.30f   /* how far ahead of the jaw we command        */
#define CAL_STALL_VEL         0.05f   /* rad/s below which the jaw is "not moving"  */
#define CAL_STALL_MS          300     /* stalled this long => hard stop reached     */
#define CAL_CONTACT_TQ        0.06f   /* Nm: torque rise that confirms real contact */
#define CAL_TIMEOUT_MS        8000    /* safety timeout per phase                   */
#define CAL_BACKOFF_RAD       0.05f   /* back off this far from each hard stop      */

/* --- free position moves --- */
#define MOVE_CURRENT_PU       0.50f
#define MOVE_SPEED            3.0f    /* rad/s */
#define MOVE_POS_TOL          0.02f
#define MOVE_SETTLE_VEL       0.05f
#define MOVE_TIMEOUT_MS       4000

/* --- gripping: also start low, raise once the force is characterised --- */
#define GRIP_CURRENT_PU       0.12f   /* default clamp current (per-unit)  */
#define GRIP_SPEED            1.0f    /* rad/s while seeking contact       */
#define GRIP_CONTACT_TQ       0.06f   /* Nm: fingers have touched          */
#define GRIP_CONTACT_MS       250

/* --- boot-time position recovery --- */
#define RANGE_MARGIN_RAD      0.30f   /* slack allowed outside [closed, open] */

#define NVS_NS   "gripper"
#define NVS_KEY  "calib"

/* =========================================================================
 * State
 * ========================================================================= */
static gripper_state_t s_state = GRIPPER_UNCAL;
static gripper_calib_t s_calib;

static float s_target_pos  = 0.0f;   /* absolute position setpoint (rad)   */
static float s_move_speed  = MOVE_SPEED;
static float s_grip_cur    = GRIP_CURRENT_PU;
static bool  s_hold        = false;  /* whether IDLE actively holds the target */
static bool  s_needs_home  = false;

static int   s_stall_ms    = 0;
static int   s_phase_ms    = 0;
static bool  s_seen_motion = false;

/* --- turn tracking: motor reports single-turn; we rebuild the absolute angle --- */
static bool    s_have_raw = false;
static float   s_last_raw = 0.0f;
static int32_t s_turns    = 0;
static float   s_abs      = 0.0f;    /* absolute (multi-turn) position, rad */

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

static void set_state(gripper_state_t s)
{
    if (s != s_state) {
        ESP_LOGI(TAG, "%s -> %s", gripper_state_name(s_state), gripper_state_name(s));
        s_state = s;
    }
}

static float range_lo(void) { return fminf(s_calib.pos_closed, s_calib.pos_open); }
static float range_hi(void) { return fmaxf(s_calib.pos_closed, s_calib.pos_open); }

static void calib_load(void)
{
    memset(&s_calib, 0, sizeof(s_calib));
    s_calib.grip_current = GRIP_CURRENT_PU;

    nvs_handle_t nh;
    if (nvs_open(NVS_NS, NVS_READONLY, &nh) == ESP_OK) {
        gripper_calib_t tmp;
        size_t sz = sizeof(tmp);
        if (nvs_get_blob(nh, NVS_KEY, &tmp, &sz) == ESP_OK && sz == sizeof(tmp)) {
            s_calib = tmp;
            ESP_LOGI(TAG, "calibration loaded: closed=%.3f open=%.3f span=%.3f rad grip=%.2f pu %s",
                     s_calib.pos_closed, s_calib.pos_open,
                     range_hi() - range_lo(), s_calib.grip_current,
                     s_calib.span_ok ? "(turn recoverable)" : "(span >= 1 rev: homing required)");
        }
        nvs_close(nh);
    }
    s_grip_cur = s_calib.grip_current;
}

/* Written once per calibration — never periodically, so there is no flash wear. */
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

/*
 * Turn tracking. The motor's reported position is absolute only within one
 * revolution, so:
 *  - first sample after boot: pick the turn count that lands the reading inside
 *    the calibrated range (unique when the range spans < 1 rev), then sanity-
 *    check it; if it can't be trusted we demand homing.
 *  - afterwards: count wraps. At 50 Hz ticks and this motor's top speed the jaw
 *    moves far less than pi between samples, so a jump > pi is always a wrap.
 */
static void track_position(float raw)
{
    if (!s_have_raw) {
        s_turns = 0;
        if (s_calib.valid && s_calib.span_ok) {
            float center = 0.5f * (s_calib.pos_closed + s_calib.pos_open);
            s_turns = (int32_t)lroundf((center - raw) / TWO_PI);
        }
        s_have_raw = true;
        s_last_raw = raw;
        s_abs = raw + (float)s_turns * TWO_PI;

        if (s_calib.valid) {
            if (!s_calib.span_ok) {
                ESP_LOGW(TAG, "travel spans >= 1 rev: turn count not recoverable, homing required");
                s_needs_home = true;
            } else if (s_abs < range_lo() - RANGE_MARGIN_RAD ||
                       s_abs > range_hi() + RANGE_MARGIN_RAD) {
                ESP_LOGW(TAG, "restored position %.3f rad outside [%.3f, %.3f]: homing required",
                         s_abs, range_lo(), range_hi());
                s_needs_home = true;
            } else {
                ESP_LOGI(TAG, "position restored: %.3f rad (%.1f deg), turns=%d — no homing needed",
                         s_abs, s_abs * RAD2DEG, (int)s_turns);
            }
        }
        if (s_needs_home) {
            set_state(GRIPPER_UNCAL);
        }
        return;
    }

    float d = raw - s_last_raw;
    if (d >  (float)M_PI) s_turns--;
    if (d < -(float)M_PI) s_turns++;
    s_last_raw = raw;
    s_abs = raw + (float)s_turns * TWO_PI;
}

/* Clamp a command so it never drives past the calibrated stops. */
static float clamp_to_range(float cmd)
{
    if (!s_calib.valid) return cmd;
    float lo = range_lo(), hi = range_hi();
    if (cmd < lo) cmd = lo;
    if (cmd > hi) cmd = hi;
    return cmd;
}

/*
 * Contact detection = the jaw stopped moving while we are still commanding into
 * it. Two criteria, because velocity alone is slow and keeps grinding into the
 * stop: a measurable torque rise proves we are really pushing something, so we
 * accept contact in half the time — that is what protects the finger pads.
 * Requires that the jaw moved at least once first, so a stationary start is not
 * mistaken for a hard stop.
 */
static bool contact_detect(float vel, float torque, float tq_thresh, int need_ms)
{
    if (fabsf(vel) > CAL_STALL_VEL) {
        s_seen_motion = true;
        s_stall_ms = 0;
        return false;
    }
    if (!s_seen_motion) {
        return false;
    }
    s_stall_ms += GRIPPER_TICK_MS;
    if (fabsf(torque) >= tq_thresh) {
        return s_stall_ms >= need_ms / 2;   /* torque confirms it: stop early */
    }
    return s_stall_ms >= need_ms;           /* no torque yet: need the full dwell */
}

static void phase_reset(void)
{
    s_phase_ms = 0; s_stall_ms = 0; s_seen_motion = false;
}

/* =========================================================================
 * Public API
 * ========================================================================= */
int gripper_init(void)
{
    calib_load();
    /* The motor zeroes its setpoints on a mode switch and does not persist the
     * mode, so select force-position every boot. */
    motor_twai_set_mode(MOTOR_MODE_FORCE_POS);
    s_state = s_calib.valid ? GRIPPER_IDLE : GRIPPER_UNCAL;
    s_hold  = false;
    ESP_LOGI(TAG, "init, state=%s (mode=force-position)", gripper_state_name(s_state));
    return ESP_OK;
}

void gripper_calibrate(void)
{
    ESP_LOGI(TAG, "auto-calibration: seeking closed hard stop at %.2f pu", (float)CAL_CURRENT_PU);
    motor_twai_set_mode(MOTOR_MODE_FORCE_POS);
    motor_twai_enable();
    phase_reset();
    set_state(GRIPPER_CAL_CLOSE);
}

void gripper_home(void)
{
    ESP_LOGI(TAG, "homing: driving to the closed stop");
    motor_twai_set_mode(MOTOR_MODE_FORCE_POS);
    motor_twai_enable();
    phase_reset();
    set_state(GRIPPER_CAL_CLOSE);   /* same seek; CAL_CLOSE re-anchors pos_closed */
}

void gripper_move_deg(float deg, float dps)
{
    if (!s_calib.valid || s_needs_home) { ESP_LOGW(TAG, "move ignored: not calibrated/homed"); return; }
    s_target_pos = clamp_to_range(deg * DEG2RAD);
    s_move_speed = (dps > 0.0f) ? (dps * DEG2RAD) : MOVE_SPEED;
    s_hold = true;
    phase_reset();
    motor_twai_enable();
    set_state(GRIPPER_MOVING);
    ESP_LOGI(TAG, "move -> %.1f deg (%.3f rad) @ %.1f deg/s", deg, s_target_pos, dps);
}

void gripper_set_opening(float frac)
{
    if (!s_calib.valid || s_needs_home) { ESP_LOGW(TAG, "set_opening ignored: not calibrated/homed"); return; }
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    s_target_pos = s_calib.pos_closed + frac * (s_calib.pos_open - s_calib.pos_closed);
    s_move_speed = MOVE_SPEED;
    s_hold = true;
    phase_reset();
    motor_twai_enable();
    set_state(GRIPPER_MOVING);
    ESP_LOGI(TAG, "opening -> %.0f%% (pos %.3f)", frac * 100.0f, s_target_pos);
}

void gripper_grip(float current_pu)
{
    /* Require calibration: without a known closed stop there is nothing to bound
     * the closing command against. */
    if (!s_calib.valid || s_needs_home) { ESP_LOGW(TAG, "grip ignored: not calibrated/homed"); return; }
    s_grip_cur = (current_pu > 0.0f) ? current_pu : s_calib.grip_current;
    if (s_grip_cur > 1.0f) s_grip_cur = 1.0f;
    phase_reset();
    motor_twai_enable();
    set_state(GRIPPER_GRIPPING);
    ESP_LOGI(TAG, "grip @ %.2f pu current", s_grip_cur);
}

void gripper_release(void)
{
    if (!s_calib.valid || s_needs_home) { ESP_LOGW(TAG, "release ignored: not calibrated/homed"); return; }
    s_target_pos = s_calib.pos_open;
    s_move_speed = MOVE_SPEED;
    s_hold = true;
    phase_reset();
    motor_twai_enable();
    set_state(GRIPPER_MOVING);
}

void gripper_set_grip_current(float current_pu)
{
    if (current_pu < 0.0f) current_pu = 0.0f;
    if (current_pu > 1.0f) current_pu = 1.0f;
    s_calib.grip_current = current_pu;
    s_grip_cur = current_pu;
    calib_save();
    ESP_LOGI(TAG, "default grip current = %.2f pu (saved)", current_pu);
}

void gripper_stop(void)
{
    s_target_pos = s_abs;
    s_hold = true;
    set_state(GRIPPER_IDLE);
}

gripper_state_t gripper_get_state(void) { return s_state; }
void gripper_get_calib(gripper_calib_t *out) { if (out) *out = s_calib; }
float gripper_get_angle_deg(void) { return s_abs * RAD2DEG; }
bool  gripper_needs_homing(void) { return s_needs_home; }

float gripper_get_opening(void)
{
    if (!s_calib.valid) return -1.0f;
    float span = s_calib.pos_open - s_calib.pos_closed;
    if (fabsf(span) < 1e-6f) return -1.0f;
    float f = (s_abs - s_calib.pos_closed) / span;
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    return f;
}

/* =========================================================================
 * Control tick — every state issues one force-position command:
 *   (target position, speed limit, current limit)
 * ========================================================================= */
void gripper_tick(const motor_status_t *m, const haptic_sample_t *h)
{
    (void)h;   /* tactile is the supervisor's over-force backstop, not the force loop */

    if (m) {
        track_position(m->position);
    }
    float vel    = m ? m->velocity : 0.0f;
    float torque = m ? m->torque   : 0.0f;   /* proportional to current */
    s_phase_ms += GRIPPER_TICK_MS;

    switch (s_state) {
    case GRIPPER_CAL_CLOSE: {
        /* Creep into the closed stop: command a little beyond where we are, at a
         * low current limit, so contact simply stalls the motor. */
        float cmd = s_abs + GRIPPER_CLOSE_DIR * CAL_LEAD_RAD;
        motor_twai_set_force_pos(cmd, CAL_SPEED, CAL_CURRENT_PU);
        if (contact_detect(vel, torque, CAL_CONTACT_TQ, CAL_STALL_MS)) {
            s_calib.pos_closed = s_abs - GRIPPER_CLOSE_DIR * CAL_BACKOFF_RAD;
            ESP_LOGI(TAG, "closed stop at %.3f (tq=%.3f Nm) -> zero=%.3f",
                     s_abs, torque, s_calib.pos_closed);
            /* Homing re-anchors the closed stop; a full calibration also needs
             * the open end, so only continue when the span is still unknown. */
            if (s_calib.valid && s_needs_home) {
                s_needs_home = false;
                s_target_pos = s_calib.pos_closed;
                s_hold = true;
                calib_save();
                ESP_LOGI(TAG, "homed");
                set_state(GRIPPER_IDLE);
            } else {
                phase_reset();
                set_state(GRIPPER_CAL_OPEN);
            }
        } else if (s_phase_ms > CAL_TIMEOUT_MS) {
            ESP_LOGE(TAG, "cal close timeout (no motion/feedback) -> ERROR");
            motor_twai_disable();
            set_state(GRIPPER_ERROR);
        }
        break;
    }

    case GRIPPER_CAL_OPEN: {
        float cmd = s_abs - GRIPPER_CLOSE_DIR * CAL_LEAD_RAD;
        motor_twai_set_force_pos(cmd, CAL_SPEED, CAL_CURRENT_PU);
        if (contact_detect(vel, torque, CAL_CONTACT_TQ, CAL_STALL_MS)) {
            s_calib.pos_open = s_abs + GRIPPER_CLOSE_DIR * CAL_BACKOFF_RAD;
            s_calib.valid    = true;
            /* Turn count is only recoverable at boot when the whole travel fits
             * inside one revolution. */
            float span = fabsf(s_calib.pos_open - s_calib.pos_closed);
            s_calib.span_ok = (span < TWO_PI - 2.0f * RANGE_MARGIN_RAD);
            s_needs_home = false;
            calib_save();
            ESP_LOGI(TAG, "open stop at %.3f -> open=%.3f; span=%.3f rad (%.1f deg) %s",
                     s_abs, s_calib.pos_open, span, span * RAD2DEG,
                     s_calib.span_ok ? "-> position survives power-off"
                                     : "-> WARNING: >= 1 rev, homing needed each boot");
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
        motor_twai_set_force_pos(s_target_pos, s_move_speed, MOVE_CURRENT_PU);
        if (fabsf(s_abs - s_target_pos) < MOVE_POS_TOL && fabsf(vel) < MOVE_SETTLE_VEL) {
            set_state(GRIPPER_IDLE);
        } else if (s_phase_ms > MOVE_TIMEOUT_MS) {
            ESP_LOGW(TAG, "move did not settle; holding current position");
            s_target_pos = s_abs;       /* stop pushing toward an unreachable target */
            set_state(GRIPPER_IDLE);
        }
        break;

    case GRIPPER_GRIPPING:
        /* Command the closed stop under the grip current limit: the fingers wrap
         * and the motor stalls at exactly that current => constant clamp force. */
        motor_twai_set_force_pos(s_calib.pos_closed, GRIP_SPEED, s_grip_cur);
        if (contact_detect(vel, torque, GRIP_CONTACT_TQ, GRIP_CONTACT_MS)) {
            ESP_LOGI(TAG, "contact: clamping at %.2f pu (tq=%.3f Nm, pos %.3f)",
                     s_grip_cur, torque, s_abs);
            set_state(GRIPPER_HOLDING);
        }
        break;

    case GRIPPER_HOLDING:
        /* Same command — the current limit holds the clamping force by itself. */
        motor_twai_set_force_pos(s_calib.pos_closed, GRIP_SPEED, s_grip_cur);
        break;

    case GRIPPER_IDLE:
        if (s_hold) {
            motor_twai_set_force_pos(s_target_pos, s_move_speed, MOVE_CURRENT_PU);
        }
        break;

    case GRIPPER_UNCAL:
    case GRIPPER_ERROR:
    default:
        break;   /* issue no command */
    }
}
