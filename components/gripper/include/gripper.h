/**
 * @file gripper.h
 * @brief High-level gripper application layer.
 *
 * Turns the low-level motor (TWAI/MIT) and tactile sensor into gripper-shaped
 * operations: auto range calibration (find the closed/open hard stops by
 * stalling against them, then back off), NVS-persisted zero/open positions,
 * position moves in a normalized 0..1 opening, and constant-force gripping
 * using tactile force feedback.
 *
 * The supervisor drives this: it calls gripper_tick() every control cycle with
 * the latest motor + haptic snapshots, and forwards user commands to the
 * gripper_* action functions. The gripper owns the motor for these actions.
 */
#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdbool.h>
#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GRIPPER_UNCAL = 0,   /* no valid calibration yet          */
    GRIPPER_IDLE,        /* holding last position / idle      */
    GRIPPER_CAL_CLOSE,   /* seeking the closed hard stop      */
    GRIPPER_CAL_OPEN,    /* seeking the open hard stop        */
    GRIPPER_MOVING,      /* moving to a position target       */
    GRIPPER_GRIPPING,    /* closing until contact             */
    GRIPPER_HOLDING,     /* force-controlled hold on an object*/
    GRIPPER_ERROR,       /* calibration/motion fault          */
} gripper_state_t;

typedef struct {
    bool  valid;         /* true once calibrated / loaded from NVS */
    float pos_closed;    /* motor position (rad) at opening = 0 (fully closed) */
    float pos_open;      /* motor position (rad) at opening = 1 (fully open)   */
    float grip_force_n;  /* default clamp force, Newtons                       */
} gripper_calib_t;

/** @brief Load calibration from NVS (or mark UNCAL). Call once at startup. */
int gripper_init(void);

/**
 * @brief Advance the gripper state machine one control step.
 * @param m  latest motor feedback (may be NULL if none yet)
 * @param h  latest tactile sample (may be NULL if none yet)
 * Called at a fixed rate by the supervisor (e.g. 50 Hz).
 */
void gripper_tick(const motor_status_t *m, const haptic_sample_t *h);

/* --- high-level actions (all move the motor; call under an unlocked session) */
void gripper_calibrate(void);          /* auto find closed+open stops, save NVS */
void gripper_home(void);               /* go to fully-closed zero               */
void gripper_set_opening(float frac);  /* 0.0 = closed ... 1.0 = open           */
void gripper_grip(float force_n);      /* close & hold force (0 = stored default)*/
void gripper_release(void);            /* open fully                            */
void gripper_set_grip_force(float n);  /* set + persist default clamp force     */
void gripper_stop(void);               /* hold current position                 */

/* --- queries --- */
gripper_state_t gripper_get_state(void);
void            gripper_get_calib(gripper_calib_t *out);
float           gripper_get_opening(void);   /* last measured opening 0..1, -1 if uncal */

const char *gripper_state_name(gripper_state_t s);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_H */
