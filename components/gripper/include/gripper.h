/**
 * @file gripper.h
 * @brief Gripper application layer: boot calibration and tactile force grip.
 *
 * The DM-J4310 runs in force-position mode (CTRL_MODE 4): every command
 * carries a target position, a speed limit and a current limit, so the drive
 * caps the current and therefore the jaw force at all times.
 *
 * CALIBRATION (once per boot, and on command), all at low current:
 *   1. record the tactile baseline with the motor still disabled;
 *   2. open slowly. The current limit starts low and is raised only until the
 *      jaw moves. The open hard stop is detected from the motor current
 *      (torque feedback rises while the position stops changing); the jaw
 *      backs off from it and that position becomes "open". If tactile force
 *      rises while opening, GRIPPER_CLOSE_DIR is wrong and the run aborts;
 *   3. tare both tactile modules while the pads are free;
 *   4. close slowly until BOTH pads read contact (pad meets pad). Optionally
 *      squeeze them to about 1.5 N to measure the force per unit current;
 *   5. back off until both pads read below 0.2 N; that position is "closed".
 * Positions are in the drive's frame. The DM-J4310-2EC reports the output
 * shaft's absolute single-turn position, so a calibration is saved to flash
 * and reused after a reboot (ESP or motor) once gripper_home() has found the
 * open stop within 0.03 rad of the stored one. gripper_calibrate() always
 * measures everything again and overwrites the stored calibration.
 *
 * FORCE GRIP: close at friction current plus a small margin, at a speed set
 * from the measured pad stiffness, until either pad reads contact; then
 * regulate the larger of the two pad forces to the target (push below the
 * band, hold inside it, unload above it). The position target stays within a
 * short distance of the jaw, so it cannot run far if the object slips out.
 *
 * The module has no FreeRTOS dependency: the supervisor passes everything in
 * gripper_input_t, and the only outputs are calls into motor_twai and one tare
 * request to haptic_uart. That keeps it testable on a host.
 */
#ifndef GRIPPER_H
#define GRIPPER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GRIPPER_TACTILE_COUNT 2   /* index 0 = ring-finger module, 1 = little */

typedef enum {
    GRIPPER_UNCAL = 0,     /* no calibration for the current motor power cycle  */
    GRIPPER_IDLE,          /* calibrated; holding a position or motor disabled  */
    GRIPPER_CALIBRATING,
    GRIPPER_MOVING,        /* position move inside the calibrated range         */
    GRIPPER_GRIPPING,      /* closing onto an object or settling the force      */
    GRIPPER_HOLDING,       /* force inside the deadband, still regulated        */
    GRIPPER_ERROR,         /* latched fault, motor disabled                     */
} gripper_state_t;

typedef enum {
    GRIPPER_FAULT_NONE = 0,
    GRIPPER_FAULT_MOTOR_NOT_READY,      /* enable refused / probe not done        */
    GRIPPER_FAULT_MOTOR_LOST,           /* feedback stale or motor re-probed      */
    GRIPPER_FAULT_MOTOR_FAULT,          /* drive reported an error code           */
    GRIPPER_FAULT_MOTOR_NOT_ENABLED,    /* feedback never showed "enabled"        */
    GRIPPER_FAULT_TACTILE_UNAVAILABLE,  /* tactile stale/absent where required    */
    GRIPPER_FAULT_TACTILE_STALE,        /* grip: tactile stale, force frozen      */
    GRIPPER_FAULT_FORCE_WHILE_OPENING,  /* direction check failed                 */
    GRIPPER_FAULT_NO_MOTION,            /* jaw did not move at the current ceiling */
    GRIPPER_FAULT_TRAVEL_LIMIT,         /* a seek ran farther than any real stroke */
    GRIPPER_FAULT_TIMEOUT,
    GRIPPER_FAULT_TARE,                 /* tactile tare failed / did not finish   */
    GRIPPER_FAULT_BASELINE,             /* tactile not near zero after the tare   */
    GRIPPER_FAULT_ONE_SIDED_CONTACT,    /* one pad loaded, the other not          */
    GRIPPER_FAULT_STALL_NO_CONTACT,     /* jaw stopped closing without tactile    */
    GRIPPER_FAULT_CONTACT_TOO_HARD,     /* contact force rose past the abort level */
    GRIPPER_FAULT_GAIN_INVALID,         /* squeeze gave an implausible N per pu    */
    GRIPPER_FAULT_NOT_SEPARATED,        /* pads still loaded after the back-off    */
    GRIPPER_FAULT_SPAN_INVALID,
    GRIPPER_FAULT_OVERFORCE,            /* tactile force or single point too high */
    GRIPPER_FAULT_CONTACT_DURING_MOVE,  /* a position move met resistance          */
    GRIPPER_FAULT_EMPTY_GRASP,          /* grip closed until the pads met          */
    GRIPPER_FAULT_GRIP_NO_TACTILE,      /* grip stalled with no pad force          */
    GRIPPER_FAULT_POSITION_JUMP,        /* feedback position jumped: frame lost    */
    GRIPPER_FAULT_SPEED_MISMATCH,       /* jaw far slower than commanded           */
    GRIPPER_FAULT_FORCE_MISMATCH,       /* pads read far less than the current implies */
    GRIPPER_FAULT_TACTILE_RETARED,      /* tactile zero changed while driving      */
    GRIPPER_FAULT_SPAN_CHANGED,         /* stroke differs from the saved one       */
    GRIPPER_FAULT_FRAME_CHANGED,        /* home check: open stop not where stored  */
} gripper_fault_t;

/** Everything the gripper needs each tick, assembled by the supervisor. */
typedef struct {
    uint32_t dt_ms;          /* time since the previous tick                     */
    bool     allow_motion;   /* supervisor permits actuation                     */

    bool     motor_ready;    /* motor_twai probe succeeded, feedback alive       */
    uint32_t motor_epoch;    /* motor_twai_epoch()                               */
    bool     motor_fresh;    /* feedback newer than the supervisor's limit       */
    uint32_t motor_seq;      /* changes with every feedback frame                */
    bool     motor_enabled;  /* feedback ERR field says enabled                  */
    uint8_t  motor_error;    /* 0, or the Damiao fault code 8..E                 */
    float    pos;            /* rad, drive frame                                 */
    float    vel;            /* rad/s (logged only; not used for decisions)      */
    float    torque;         /* N*m, from the drive's current measurement        */
    float    motor_tau_per_pu_est; /* N*m per pu from the motor's registers
                                    * (Kt x Imax, motor_twai_get_info); 0 = none */

    bool     tac_ok[GRIPPER_TACTILE_COUNT];        /* fresh and slot status OK   */
    uint32_t tac_seq[GRIPPER_TACTILE_COUNT];       /* changes with every sample  */
    float    fz[GRIPPER_TACTILE_COUNT];            /* resultant normal force, N  */
    float    fz_point_max[GRIPPER_TACTILE_COUNT];  /* largest single point, N    */
    uint32_t tare_count;     /* haptic_uart_tare_count()                         */
    bool     tare_ok;        /* result of the last finished tare                 */
} gripper_input_t;

typedef struct {
    bool     valid;          /* calibrated during the current motor epoch        */
    uint32_t motor_epoch;
    float    pos_open;       /* rad: open stop backed off toward closed          */
    float    pos_closed;     /* rad: pad touch backed off toward open            */
    float    pos_touch;      /* rad: both pads first read contact                */
    float    k_force;        /* N per unit current (pad squeeze near closed)     */
    float    i_offset;       /* pu: current at zero force while closing          */
    float    i_move;         /* pu: current that moved the jaw during the seeks  */
    float    tau_per_pu;     /* N*m per pu at the open stop (diagnostic)         */
    float    pad_stiffness;  /* N per rad, pad on pad (sets the approach speed)  */
    float    approach_speed; /* rad/s, grip approach                             */
    float    default_force;  /* N                                                */
    bool     k_estimated;    /* k from the motor's N*m/pu and the nominal lever,
                              * the squeeze could not measure it              */
} gripper_calib_t;

typedef struct {
    gripper_state_t state;
    gripper_fault_t fault;   /* last fault; NONE after a successful command      */
    float pos;               /* rad                                              */
    float opening;           /* 0 = closed .. 1 = open, -1 if not calibrated     */
    float force_target;      /* N, 0 when not gripping                           */
    float force[GRIPPER_TACTILE_COUNT];
    float cur_cmd;           /* pu, last current limit sent                      */
} gripper_status_t;

/** Load the persisted default force. @p autocal_on_boot runs one calibration
 *  as soon as motor and tactile are ready. */
int gripper_init(bool autocal_on_boot);

/** Advance one control step. Call at a fixed rate (the supervisor uses 100 Hz). */
void gripper_tick(const gripper_input_t *in);

/* Commands. Each returns false (and changes nothing) when its preconditions
 * are not met; gripper_get_status() then tells why. */
bool gripper_calibrate(void);                  /* full calibration, saved to flash */
/** Home check: tare the pads where the jaw is (they must be free), close
 *  until they touch (slowing down near the stored touch position), and restore
 *  the stored calibration if they touch within 0.03 rad of it (or a whole
 *  encoder turn away). Without a stored calibration: a full calibration. */
bool gripper_home(void);
bool gripper_has_stored_calib(void);
/* force_n <= 0: stored default. speed: approach rad/s, 0 = 2.0, at most 5.0 */
bool gripper_grip(float force_n, float speed);
bool gripper_set_opening(float frac);          /* 0 closed .. 1 open          */
/* closing percent, 5 .. 95 (0 = open position, 100 = closed; outside is
 * moved to the nearer end); speed rad/s at the output, 0 = 2.0, 0.05 .. 15.0 */
bool gripper_set_closing(float percent, float speed);
bool gripper_move_deg(float deg_from_closed, float dps);
bool gripper_set_default_force(float force_n); /* persisted                   */
void gripper_stop(void);                       /* hold the current position   */
void gripper_abort(void);                      /* motor off now               */
void gripper_clear_fault(void);

gripper_state_t gripper_get_state(void);
bool gripper_is_busy(void);                    /* calibrating, moving, gripping */
bool gripper_motor_active(void);               /* the gripper has the motor enabled */
void gripper_get_calib(gripper_calib_t *out);
void gripper_get_status(gripper_status_t *out);
const char *gripper_state_name(gripper_state_t s);
const char *gripper_fault_name(gripper_fault_t f);

#ifdef __cplusplus
}
#endif

#endif /* GRIPPER_H */
