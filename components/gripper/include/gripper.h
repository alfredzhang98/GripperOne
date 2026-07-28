/**
 * @file gripper.h
 * @brief High-level gripper application layer (5-finger underactuated hand).
 *
 * Turns the DM-J4310 into gripper-shaped operations. The motor is driven in
 * FORCE-POSITION mode (CTRL_MODE 4): every command carries a target position, a
 * speed limit and a CURRENT limit. That single mode covers both jobs:
 *   - free travel  -> the jaw moves to the commanded position
 *   - contact      -> the motor stalls at the current limit, i.e. a controlled,
 *                     hardware-enforced clamping force (no software force loop)
 * which is exactly how an underactuated hand should grip: the fingers wrap and
 * self-adapt, and the current limit sets how hard.
 *
 * POSITION AFTER POWER-OFF. The motor's encoder is absolute within ONE output
 * revolution but loses the turn count on a power cycle (measured: the reading
 * came back shifted by exactly 2*pi). We recover the turn count without any
 * periodic flash writes: after calibration the travel range is known, so if the
 * range spans less than one revolution there is exactly ONE turn count that puts
 * the power-up reading inside that range. Calibration is written to NVS once;
 * boot-up reconstructs from it and range-checks the result, and only falls back
 * to re-homing when the span is too wide or the reading is implausible.
 */
#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdbool.h>
#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GRIPPER_UNCAL = 0,   /* no valid calibration / needs homing  */
    GRIPPER_IDLE,        /* holding last position                */
    GRIPPER_CAL_CLOSE,   /* seeking the closed hard stop         */
    GRIPPER_CAL_OPEN,    /* seeking the open hard stop           */
    GRIPPER_MOVING,      /* moving to a position target          */
    GRIPPER_GRIPPING,    /* closing under a current (force) limit*/
    GRIPPER_HOLDING,     /* contact reached, clamping            */
    GRIPPER_ERROR,       /* calibration / motion fault           */
} gripper_state_t;

typedef struct {
    bool  valid;         /* true once calibrated / loaded from NVS             */
    float pos_closed;    /* motor position (rad) at opening = 0 (fully closed) */
    float pos_open;      /* motor position (rad) at opening = 1 (fully open)   */
    float grip_current;  /* default clamp current, per-unit 0..1               */
    bool  span_ok;       /* travel < 1 rev -> turn count is recoverable at boot */
} gripper_calib_t;

/** @brief Load calibration from NVS (or mark UNCAL). Call once at startup. */
int gripper_init(void);

/**
 * @brief Advance the gripper state machine one control step.
 * @param m  latest motor feedback (may be NULL if none yet)
 * @param h  latest tactile sample (may be NULL if none yet)
 * Called at a fixed rate by the supervisor (e.g. 50 Hz). Also does the turn
 * tracking that turns the motor's single-turn reading into an absolute angle.
 */
void gripper_tick(const motor_status_t *m, const haptic_sample_t *h);

/* --- high-level actions (all move the motor; call under an unlocked session) */
/** @brief Auto-find the closed+open hard stops at low current, save to NVS. */
void gripper_calibrate(void);
/** @brief Re-establish the reference by driving to the closed stop. */
void gripper_home(void);
/** @brief Move to an absolute output-shaft angle (degrees) at @p dps deg/s. */
void gripper_move_deg(float deg, float dps);
/** @brief Move to a normalized opening: 0.0 = closed ... 1.0 = open. */
void gripper_set_opening(float frac);
/** @brief Close and clamp at @p current_pu (0..1 per-unit; <=0 = stored default). */
void gripper_grip(float current_pu);
/** @brief Open fully. */
void gripper_release(void);
/** @brief Set + persist the default clamp current (per-unit 0..1). */
void gripper_set_grip_current(float current_pu);
/** @brief Hold the current position. */
void gripper_stop(void);

/* --- queries --- */
gripper_state_t gripper_get_state(void);
void            gripper_get_calib(gripper_calib_t *out);
/** @brief Last measured opening 0..1, or -1 if not calibrated. */
float           gripper_get_opening(void);
/** @brief Absolute output-shaft angle in degrees (turn-tracked). */
float           gripper_get_angle_deg(void);
/** @brief True when the boot-time position could not be trusted; home first. */
bool            gripper_needs_homing(void);

const char *gripper_state_name(gripper_state_t s);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_H */
