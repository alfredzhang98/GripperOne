/**
 * @file motor_twai.h
 * @brief Damiao DM-J4310-2EC over TWAI (CAN 2.0), force-position mode only.
 *
 * The gripper drives the motor exclusively in the force-position mode
 * ("力位混控", CTRL_MODE register 0x0A = 4). Every command frame carries a
 * target position, a speed limit and a current limit, so the drive itself caps
 * the current and therefore the jaw force. The other three Damiao modes (MIT,
 * position-velocity, velocity) are deliberately not exposed: a command frame
 * for another mode would be accepted on the same CAN ID (the drive checks only
 * the low 8 ID bits, manual p.12) and decoded with the wrong byte layout.
 *
 * START-UP PROBE. motor_twai_task does not command motion until it has, with
 * the motor disabled:
 *   1. seen a feedback frame in reply to a disable frame (motor powered, IDs
 *      and bit rate right);
 *   2. read PMAX/VMAX/TMAX (0x15..0x17) and switched the feedback decode to
 *      those values instead of compiled-in guesses;
 *   3. written CTRL_MODE = 4 and the comms-loss TIMEOUT (0x09), and read both
 *      back. Neither write is saved to the motor's flash unless the host asks
 *      for it (motor_twai_persist_config, opcode 0x17).
 * The CAN hardware filter accepts every standard ID; the RX callback decodes
 * only MST_ID and records other frames, which the probe prints when the motor
 * does not answer (wrong CAN_ID / MST_ID are the usual first-power-up faults).
 * motor_twai_is_ready() reports the result. A probe that fails is retried
 * every second. If feedback stops for FEEDBACK_LOST_MS the driver drops back
 * to "not ready", disables the motor and probes again; motor_twai_epoch()
 * increments on every successful probe so a caller can tell that the drive may
 * have rebooted and its position frame may have changed.
 *
 * FEEDBACK. The drive only answers frames addressed to it (manual p.12). While
 * enabled, the 100 Hz command stream provides feedback; while disabled, the
 * task sends a disable frame every 50 ms as a status poll, so position and
 * faults stay observable.
 *
 * HARDWARE NOTE: the ESP32-C6 has no built-in CAN transceiver. An external
 * 3.3 V transceiver (TCAN332 / SN65HVD230 / similar) is mandatory.
 */
#ifndef MOTOR_TWAI_H
#define MOTOR_TWAI_H

#include <stdint.h>
#include <stdbool.h>
#include "app_protocol.h"

/* Acceleration and deceleration of the drive's own trapezoid profile, output
 * side, rad/s^2. Written to ACC / DEC (RAM only) at every probe; the drive's
 * flash default of 2 capped every move (bench 2026-10-23). The gripper plans
 * its moves with the same value. */
#define MOTOR_TWAI_ACC_RAD_S2  80.0f

/* Position-loop gain KP_APR written at every probe (RAM only); the drive's
 * flash default is 54. 0 leaves the drive's value. Near the target the jaw
 * closed in at about 11 x (remaining distance) rad/s with 54 (bench
 * 2026-10-23), which took about 0.5 s of each 1.3-1.5 s move. */
#define MOTOR_TWAI_KP_APR      108.0f

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int tx_gpio;
    int rx_gpio;
    int bitrate_kbps;   /* 125 / 250 / 500 / 1000                                */
    int esc_id;         /* host->motor command frame ID (<0 = built-in default)  */
    int mst_id;         /* motor->host feedback frame ID (<0 = built-in default) */
    int timeout_ms;     /* comms-loss TIMEOUT written at probe (<=0 = default)   */
    int task_stack;     /* task stack depth, bytes (0 = built-in default)        */
    int task_prio;      /* FreeRTOS priority       (0 = built-in default)        */
} motor_twai_cfg_t;

/** Values read from the motor during the last successful probe. */
typedef struct {
    float    pmax;            /* rad,   register 0x15 (position map range)      */
    float    vmax;            /* rad/s, register 0x16 (velocity map range)      */
    float    tmax;            /* N*m,   register 0x17 (torque map range)        */
    float    imax;            /* A,     register 0x3B (0 if the read failed)    */
    float    tau_per_pu_est;  /* N*m per unit current, from registers; 0 = n/a  */
    uint32_t timeout_ticks;   /* register 0x09 after the write, 50 us per tick  */
    uint32_t ctrl_mode;       /* register 0x0A after the write (4 expected)     */
} motor_twai_info_t;

/** Install the TWAI driver, start the bus, and spawn motor_twai_task. */
int motor_twai_start(const motor_twai_cfg_t *cfg, const app_bus_t *bus);

/** True after a successful probe while feedback keeps arriving. */
bool motor_twai_is_ready(void);

/** Increments on every successful probe (0 = never probed). */
uint32_t motor_twai_epoch(void);

/** Milliseconds since the last feedback frame (UINT32_MAX if none yet). */
uint32_t motor_twai_feedback_age_ms(void);

/** Copy the probe results. @return false before the first successful probe. */
bool motor_twai_get_info(motor_twai_info_t *out);

/**
 * @brief Set the force-position setpoint: target position (rad, drive frame),
 *        speed limit (rad/s, clamped 0..100) and current limit (per unit of
 *        the drive's maximum phase current, clamped 0..1). The three values are
 *        stored together, so a command frame never mixes old and new fields.
 */
void motor_twai_set_force_pos(float pos, float vel_limit, float cur_limit);

/**
 * @brief Enable the motor and immediately send the current setpoint.
 * @return false if the probe has not succeeded, feedback is stale, or no
 *         setpoint was set since the last disable (call set_force_pos first).
 */
bool motor_twai_enable(void);

/** @brief Disable the motor (FF..FD) and invalidate the setpoint. Thread-safe. */
void motor_twai_disable(void);

/** @brief Clear a latched motor fault (FF..FB). The motor stays disabled. */
void motor_twai_clear_error(void);

/**
 * @brief Save CTRL_MODE = 4 and the comms-loss TIMEOUT in the motor's own flash
 *        (register writes, then the 0xAA store command; manual p.15-16), so a
 *        motor that restarts comes back in force-position mode. Only while the
 *        motor is probed and disabled. The motor's flash has about 10000 write
 *        cycles: call on an explicit request only, never periodically.
 * @return true when the motor acknowledged the store.
 */
bool motor_twai_persist_config(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TWAI_H */
