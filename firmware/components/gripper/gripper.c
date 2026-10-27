#include "gripper.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs.h"

#include "motor_twai.h"
#include "haptic_uart.h"

static const char *TAG = "gripper";

/* =========================================================================
 * Tunables. Units: rad and rad/s at the motor output shaft (drive frame),
 * N for tactile force, "pu" for current as a fraction of the drive's maximum
 * phase current (the force-position frame's i_des field).
 * ========================================================================= */

/* Sign of the position change that CLOSES the jaw. NOT CONFIRMED on the
 * hardware: a wrong sign makes the first seek ("open") close the pads, which
 * the open-seek tactile check stops at 0.5 N above the pre-motion baseline. */
#define CLOSE_DIR              (+1.0f)

/* The force-position frame's vel_limit field is sent as the intended output
 * speed times this. The manual does not say whether the field is output or
 * rotor side. 1 is the safe guess: if the field is rotor side the jaw moves 10x
 * slower, and the first open seek aborts with "speed mismatch" naming the
 * measured speed; then set 10 (the gear ratio). */
#define VEL_FIELD_SCALE        1.0f
/* Position feedback step: 2 * PMAX / 65535, 0.38 mrad at PMAX 12.5 and
 * 0.76 mrad at the 25 rad the probe still accepts. Motion windows are sized so
 * the travel they test is several steps. */
#define POS_LSB_RAD            0.0008f

/* --- tactile (CP-M3025-Core: Fz 0..25 N, 0.1 N resolution, 83.3 Hz,
 *     safe load 200 % = 50 N; resultant Fz is uint8 x 0.1 N, ceiling 25.5 N) */
#define TAC_TOUCH_N            0.3f   /* per pad: counts as contact                */
#define TAC_RISE_ABORT_N       0.5f   /* opening: rise over baseline that aborts   */
#define TAC_BASELINE_MAX_N     0.5f   /* after the tare both pads must read below  */
#define TAC_SEPARATED_N        0.2f   /* after the close back-off                  */
#define TAC_TOUCH_SAMPLES      2      /* consecutive samples per pad for a touch   */
#define TAC_STALE_FAULT_MS     300    /* tactile missing this long -> fault        */
#define TAC_REACTION_S         0.05f  /* sensor delay + sampling + loop, worst case */
#define FORCE_TRIP_N           23.0f  /* any pad at/above: back off. Above the 20 N
                                       * largest target, below the supervisor's 24 N
                                       * and the 25.5 N full scale                 */
#define TAC_SPIKE_N            8.0f   /* a pad sample this far from the previous one
                                       * is held back one sample: kept only if the
                                       * next sample confirms it (bench 2026-10-23:
                                       * single 25.5 N frames right after a tare)  */
#define FORCE_TRIP_SAMPLES     2      /* ... on this many samples in a row: one
                                       * frame of 25.5 N (0xFF) came right after a
                                       * tare with the pads 7 rad apart (bench
                                       * 2026-10-23); 2 samples are 12.5 ms more  */
#define TAC_POINT_LIMIT_N      6.0f   /* any single taxel at/above: back off.
                                       * ESTIMATE - the manual gives no per-point
                                       * rating; it lists local overload as a cause
                                       * of wrong readings. Tune on the bench.     */
#define RETREAT_FORCE_RISE_N   0.5f   /* force rising this much while backing off: motor off */
#define RETREAT_REF_MS         60     /* reference = peak reading over this first window,
                                       * because the reading lags the force          */

/* --- calibration --- */
#define CAL_SPEED              0.3f   /* rad/s, free travel                         */
#define CAL_TOUCH_SPEED        0.05f  /* rad/s, near the expected pad touch         */
#define CAL_SLOW_ZONE_RAD      0.25f  /* slow down this far before the saved stroke */
#define CAL_CUR_START_PU       0.005f /* start low: with little friction a higher
                                       * start pushes needlessly hard on contact   */
#define CAL_TQ_MAX_NM          0.6f   /* ceiling for every seek, as output torque:
                                       * bounds the load on the open stop and on
                                       * the pads when the jaw does not move. The
                                       * simulation's breakaway (0.4 N*m) x 1.5   */
#define CAL_CUR_MAX_PU         0.03f  /* the same in pu when the motor gives no
                                       * Kt x Imax estimate (0.58 N*m at 19.27)    */
#define CAL_ESC_FACTOR         1.15f  /* raise the current by this when not moving */
#define CAL_ESC_INTERVAL_MS    150
#define CAL_MOVING_MARGIN      1.2f   /* once moving, keep this much headroom      */
#define CAL_OPEN_MARGIN        1.5f   /* open seek: more headroom, so that moving at
                                       * the speed limit leaves the current well
                                       * below its limit and the stop shows up as a
                                       * torque rise (the open stop is structure,
                                       * not pads; the 0.08 pu ceiling still holds) */
#define SEEK_TRIM_MS           300    /* after the jaw moves: trim the current once */
#define SEEK_TRIM_FACTOR       1.25f  /* to this x the PEAK sliding torque (ripple
                                       * included: bench 0.01-0.13 N*m closing)    */
#define SEEK_PEAK_DECAY_NM_S   0.05f  /* peak-hold of the sliding torque decays at this */
#define SEEK_TRIM_RISE_NM_RAD  0.2f   /* the trimmed limit may rise this much per rad
                                       * travelled (friction growing along the
                                       * stroke); pressing a pad raises the torque
                                       * at ~1.5 N*m/rad, which it cannot follow    */
#define CAL_OPEN_TQ_NM         0.36f  /* opening torque (0.5 s mean, at 0.05 rad/s)
                                       * taken as the maximum opening. Bench
                                       * 2026-10-22: 0.28 N*m mid-stroke, 0.32 at
                                       * 72 mm, 0.36 at about 75 mm, 0.40 at about
                                       * 77 mm, 0.51 at 80 mm (the opening end is
                                       * soft, not a hard stop). 0.36 chosen by the
                                       * user: 0.08 N*m above mid-stroke friction  */
#define CAL_OPEN_TQ_MIN_MS     1000   /* ... checked only after moving this long   */
#define CAL_OPEN_TQ_CUR_FACTOR 1.25f  /* open seek current limit once moving: this x
                                       * CAL_OPEN_TQ_NM (0.45 N*m, about 78 mm on
                                       * the bench, below the 0.51 N*m at 80 mm)   */
#define SEEK_TQ_SLOW_ALPHA     0.02f  /* per tick: about a 0.5 s mean at 100 Hz    */
#define SEEK_EXCESS_FORCE_N    10.0f  /* ... but never more than this force (at the
                                       * nominal lever) above sliding: the pad force
                                       * a seek can apply when the pads read nothing */
#define CAL_NO_MOTION_MS       450    /* at the ceiling and still not moving       */
#define CAL_MAX_TRAVEL_RAD     8.0f   /* no seek may travel farther. The measured
                                       * stroke is 6.9 rad (395 deg of the output
                                       * shaft, bench 2026-10-22): more than a turn */
#define CAL_PHASE_TIMEOUT_MS   240000 /* 6.9 rad at CAL_TOUCH_SPEED is 138 s       */
#define CAL_SPEED_CHECK_MS     500    /* after the jaw moves: compare speeds once  */
#define CAL_SPEED_MIN_FRAC     0.4f
#define CAL_OPEN_DWELL_MS      100    /* averaging window for the stall torque     */
#define CAL_OPEN_BACKOFF_RAD   0.10f  /* open stop -> "open" position              */
#define CAL_STOP_MARGIN_RAD    0.02f  /* no back-off ever goes nearer the stop     */
#define CAL_RETRY_BACKOFF_RAD  0.20f  /* open seek could not move: close this far  */
#define CAL_OPEN_TQ_MIN_TRAVEL 0.3f   /* opening limit by torque within this travel:
                                       * the jaw started at or past it            */
#define CAL_OPEN_TQ_RETRY_RAD  2.5f   /* ... then close this far and open again    */
#define CAL_OPEN_TQ_EARLY_MS   400    /* after this long, the 0.1 s mean at ...     */
#define CAL_OPEN_TQ_EARLY_FACTOR 1.15f /* ... this x the limit stops at once: a jaw
                                       * started past the limit opens no further  */
#define CAL_CLOSE_BACKOFF_RAD  0.05f  /* pad touch -> "closed" position, per try   */
#define CAL_BACKOFF_TRIES      3
#define CAL_BACKOFF_TIMEOUT_MS 4000
#define CAL_CONTACT_ABORT_N    4.0f   /* any pad this high while calibrating (16 % FS) */
#define CAL_TOUCH_CONFIRM_MS   500    /* first touch -> both pads must confirm     */
#define CAL_SQUEEZE_LO_N       0.5f   /* fit window start (mean of the two pads)   */
#define CAL_SQUEEZE_SPAN_N     1.5f   /* fit window width                          */
#define CAL_SQUEEZE_TQ_STEP    0.002f /* N*m at the output per current step, via the
                                       * N*m/pu measured at the open stop, rounded
                                       * to whole CUR_LSB_PU: at 19.27 N*m/pu one
                                       * LSB = 0.0019 N*m = 0.19 N at 10.4 mm      */
#define CUR_LSB_PU             0.0001f /* i_des resolution: x10000 in the frame (p.14) */
#define CAL_SQUEEZE_DWELL_MS   150    /* hold each step this long: jaw at rest,    */
#define CAL_SQUEEZE_AVG_MS     50     /* tactile averaged over the last 50 ms      */
#define CAL_SQUEEZE_MAX_DWELL_MS 600  /* still moving: step on without a point     */
#define CAL_SQUEEZE_TQ_SPAN_MAX 0.2f  /* N*m above the start current: about
                                       * 2 x the 4 N abort at the longest accepted
                                       * lever; the non-tactile backstop          */
#define CAL_SQUEEZE_CUR_MAX_PU 0.20f  /* absolute ceiling                          */
#define CAL_SQUEEZE_RESID_N    0.2f   /* RMS residual of the fit (sensor 0.1 N)    */
#define CAL_SQUEEZE_SETTLE_MS  300
#define CAL_SQUEEZE_UNLOAD_RAD 0.01f  /* settle this far behind the touch first    */
#define CAL_SQUEEZE_MIN_SAMPLES 5
#define CAL_SQUEEZE_FIT_POINTS 8      /* go past the window's top for this many,
                                       * up to CAL_CONTACT_ABORT_N - 0.5 N        */
#define CAL_SQUEEZE_MAX_SAMPLES 24
#define CAL_CLOSE_TRIM_NM      0.024f /* close seek: torque margin over sliding    */
#define CAL_K_MIN              5.0f   /* N/pu plausibility window                  */
#define CAL_K_MAX              6000.0f
#define CAL_I_OFFSET_MIN      (-0.05f)
/* Effective lever where the pads meet: pad-gap change per output rad near
 * the touch position. (N*m/pu at the open stop) / k must match it within
 * -40/+40 %: a k fitted too low (both pads under-reading by the same factor,
 * or a biased fit) reads as a longer lever and is refused. 10.4 mm is the
 * stroke average (72 mm over 6.9 rad, bench 2026-10-22); replace it with the
 * value measured near touch. */
#define GRIP_LEVER_NOMINAL_M   0.0104f
#define CAL_LEVER_MIN_M        (0.6f * GRIP_LEVER_NOMINAL_M)
#define CAL_LEVER_MAX_M        (1.4f * GRIP_LEVER_NOMINAL_M)
#define CAL_SPAN_MIN_RAD       3.5f   /* half the bench stroke (open position to pad
                                       * touch, about 7 rad): a shorter one means the
                                       * opening stopped early or something sits
                                       * between the pads; refused, not stored     */
#define CAL_SPAN_MAX_RAD       8.0f
#define CAL_SPAN_CHANGE_FRAC   0.15f  /* vs the saved stroke: refuse once           */
#define CAL_VERIFY_TOL_RAD     0.03f  /* home check: open stop vs the stored one    */
#define CAL_TARE_TIMEOUT_MS    4000
#define CAL_TARE_SAMPLES       4
#define CAL_SAMPLE_TIMEOUT_MS  2000
#define AUTOCAL_DELAY_MS       3000   /* motor + tactile ready this long first     */
#define AUTOCAL_WINDOW_MS      60000  /* boot calibration only this soon after boot */

/* --- motion detection from the position history; thresholds are fractions
 *     of the commanded speed, so slow phases are judged on their own scale --- */
#define MOTION_FRAC            0.4f   /* moving: >= 40 % of the commanded speed    */
#define STALL_FRAC             0.15f  /* stalled: < 15 % of it                     */
#define CREEP_MIN_RAD          0.004f /* over WIN_MAX_MS (0.01 rad/s): still moving,
                                       * however slowly - never a stop            */
#define WIN_MIN_MS             50
#define WIN_MAX_MS             400
#define SEEK_MOTION_RAD        0.05f  /* travel that proves the jaw moves          */
#define SEEK_LEAD_RAD          0.05f  /* position target at most this far ahead   */
#define RAMP_MAX_LEAD_RAD      0.05f
#define STOP_TQ_RATIO          1.1f   /* hard stop: torque above ratio x free ...  */
#define STOP_TQ_MARGIN_NM      0.01f  /* ... plus this, while stalled ...          */
#define STOP_FAST_MS           30     /* ... for this long                         */
#define STOP_HALT_MS           100    /* fast path: position window ...            */
#define STOP_HALT_RAD          (2.0f * POS_LSB_RAD) /* ... moved less than this   */
#define STOP_HALT_MIN_SPEED    0.1f   /* below this the short window alone is too
                                       * coarse (0.016 rad/s); see seek_step()     */
#define STOP_SLOW_MS           250    /* stalled without the torque rise           */
#define POS_JUMP_RAD           0.5f   /* between two feedback frames: frame lost   */

/* --- position moves and holding --- */
#define MOVE_SPEED             2.0f   /* rad/s, moves and grips when no speed is
                                       * given (user's choice). A contact at this
                                       * speed is estimated at 15 N: 7.6 N per
                                       * rad/s, from 0.002 kg*m^2 and pads 300
                                       * N/rad, neither measured                   */
#define MOVE_SPEED_MAX         15.0f  /* rad/s, the fastest a command may ask (user's
                                       * choice). Motor at 24 V: rated 12.6 rad/s,
                                       * no-load 20.9 rad/s (manual p.6). With
                                       * MOVE_ACCEL 80 the 6.4 rad 5..95 % move is
                                       * 0.62 s (0.77 s at 10 rad/s). Contact at
                                       * 15 rad/s estimated at 114 N, above the
                                       * pads' 50 N safe load                      */
#define MOVE_ACCEL             MOTOR_TWAI_ACC_RAD_S2 /* rad/s^2, the drive's own
                                       * trapezoid; the gripper's expected profile
                                       * (ramp_profile) uses the same value        */
#define RAMP_LEAD_S            0.1f   /* a move's target leads the jaw by at most
                                       * this time at the ramp speed               */
#define MOVE_RAISE_FACTOR      1.25f  /* drive at its current limit and the jaw
                                       * falling behind while still moving: the
                                       * motion needs more (speed, acceleration,
                                       * friction); raise by this every ...        */
#define MOVE_RAISE_MS          100
#define MOVE_BLOCKED_MS        300    /* at the limit and standing still this long:
                                       * blocked, the move stops there             */
#define MOVE_OPEN_TQ_MAX_NM    3.0f   /* opening moves raise up to this (motor rated
                                       * 3.5 N*m): they press nothing              */
#define MOVE_OPEN_CUR_MAX_PU   0.15f  /* ... in pu without a Kt x Imax estimate    */
#define HOLD_SPEED             0.5f
#define MOVE_FORCE_CAP_N       3.0f   /* move/hold current = friction + this via k  */
#define MOVE_TQ_MAX_NM         1.0f   /* closing move/hold ceiling, output torque  */
#define MOVE_CUR_MAX_PU        0.05f  /* ... in pu without a Kt x Imax estimate    */
#define MOVE_POS_TOL_RAD       0.01f
#define MOVE_TIMEOUT_MS        20000  /* 6.9 rad closing at 0.5 rad/s is 14 s; a
                                       * slower move gets 1.5 x its travel time    */
#define MOVE_SPEED_MIN         0.05f  /* slowest speed a move command may ask for   */
#define MOVE_PCT_MIN           5.0f   /* moves stay within 5 .. 95 % closing: a
                                       * margin to both calibrated ends (user's
                                       * choice). 0 % = open position, 100 % =
                                       * closed position                          */
#define MOVE_PCT_MAX           95.0f
#define MOVE_CONTACT_RISE_N    0.5f   /* stop a closing move if force rises this ... */
#define MOVE_CONTACT_MIN_N     1.0f   /* ... and is at least this                   */
#define RETREAT_RAD            0.30f
#define RETREAT_SPEED          0.6f
#define RETREAT_MS             900

/* --- force grip --- */
#define GRIP_FORCE_MIN_N       0.5f
#define GRIP_FORCE_MAX_N       20.0f  /* 80 % of the 25 N range (user's choice)    */
#define GRIP_FORCE_DEFAULT_N   3.0f
#define GRIP_CONTACT_OVERSHOOT_N 2.0f /* approach speed = this / (stiffness x reaction) */
#define GRIP_APPROACH_SPEED_MIN 0.03f
#define GRIP_APPROACH_SPEED_MAX 0.3f
#define GRIP_FORCE_SPEED       1.0f   /* rad/s, speed limit while regulating: a soft
                                       * object needs travel to build force (bench
                                       * 2026-10-23, sponge: 0.19 rad from contact to
                                       * 3 N, over 1 s at 0.2 rad/s)               */
#define GRIP_APPROACH_FORCE_N  2.0f   /* approach current = friction + this via k   */
#define GRIP_APPROACH_TIMEOUT_MS 300000 /* whole stroke at the slowest approach */
#define GRIP_PEAK_LOG_MS       500    /* window after contact for the logged peak  */
#define GRIP_OVERTRAVEL_RAD    0.10f  /* position target limit past pad touch      */
#define GRIP_EMPTY_TOL_RAD     0.03f  /* contact this close to pad touch = empty   */
#define GRIP_READY_OPEN_FRAC   0.80f  /* a grip starts here, and an empty one comes
                                       * back here: 80 % opening (0 = closed
                                       * position .. 1 = open position)            */
#define GRIP_NO_OBJECT_FRAC    0.95f  /* closed this far (0 = open .. 1 = closed
                                       * position) without pad force: no object.
                                       * 5 % of the 72 mm opening is about 3.6 mm */
#define GRIP_KI_PER_S          2.0f   /* integral gain, normalised by k            */
#define GRIP_DEADBAND_N        0.25f  /* ~ sensor accuracy (1 % of 25 N)           */
#define GRIP_DEADBAND_FRAC     0.04f
#define GRIP_SLEW_N_PER_S      20.0f  /* max current rise, as force rate via k     */
#define GRIP_STALL_RAMP_N_S    20.0f  /* below target with the jaw held by static
                                       * friction: the current rises at this (force
                                       * rate via k), not in proportion to the error.
                                       * Bench 2026-10-23: stuck at 1.7-2.0 N for a
                                       * 3 N target for 25 s at the ceiling          */
#define GRIP_CAP_RAISE_MS      100    /* ... and at the ceiling this long: raise it */
#define GRIP_CAP_RAISE_NM      0.03f  /* ... by this, output torque                 */
#define GRIP_STATIC_FACTOR     1.5f   /* the ceiling allows static friction of this
                                       * x the sliding friction measured on the way
                                       * in (bench: 0.23 vs 0.154 N*m)             */
#define GRIP_CAP_EXTRA_MAX_NM  0.30f  /* ... to at most this above the start value:
                                       * with 0.23 N*m static friction and the 10.4
                                       * mm lever, about 32 N if the pads under-read,
                                       * below their 50 N safe load               */
#define GRIP_HW_CAP_N          30.0f  /* current cap from k, absolute. APPROXIMATE:
                                       * k is measured pad-on-pad near closed and
                                       * may differ at other openings              */
#define GRIP_CAP_FRAC          2.0f   /* and relative to the target:                */
#define GRIP_CAP_MARGIN_N      3.0f   /* cap = (2 x target + 3 N) / k: the target is
                                       * reached even with k 2x too high (k is the
                                       * estimate from the motor when the squeeze
                                       * cannot measure it); the force itself is
                                       * set by the tactile loop                   */
#define GRIP_MIN_LEVER_M       0.0f   /* shortest lever (m) at which an object can be
                                       * held, from the CAD model. 0 = unknown, off.
                                       * When set, the current is also capped so the
                                       * force there stays <= GRIP_HW_CAP_N, using
                                       * the N*m per pu measured at the open stop.
                                       * This is the only bound that holds when the
                                       * pads under-read AND k at the object exceeds
                                       * the pad-on-pad value.                      */
#define GRIP_CUR_ABS_MAX_PU    0.06f  /* ceiling for gripping: 1.16 N*m at 19.27
                                       * N*m/pu                                    */
#define DRIVE_CUR_ABS_MAX_PU   0.16f  /* every command: 3.1 N*m at 19.27 N*m/pu, for
                                       * fast opening moves; closing moves and
                                       * gripping stay under their own ceilings    */
#define GRIP_SOFT_BAND_N       1.0f   /* above target + band: cut current at once  */
#define GRIP_SOFT_BAND_FRAC    0.30f
#define GRIP_SETTLE_MS         150
#define GRIP_UNLOAD_SPEED      0.1f   /* rad/s: target walks back while above band
                                       * (0.02 took 1.4 s to unload 1.3 N of sponge
                                       * overshoot, bench 2026-10-23)              */
#define GRIP_UNLOAD_MAX_RAD    0.03f  /* ... never more than this behind the jaw    */
#define GRIP_LOST_MS           100    /* no pad force this long: back to approach   */
#define GRIP_MISMATCH_FRAC     0.3f   /* pads below this x k*(i - friction) ...     */
#define GRIP_MISMATCH_N        1.0f   /* ... minus this, while pushing and stalled  */
#define GRIP_MISMATCH_MS       300
#define GRIP_DROP_FRAC         0.2f   /* force falls by this fraction of its recent peak */
#define GRIP_DROP_MIN_N        0.5f   /* ... and at least this ...                  */
#define GRIP_DROP_WIN_TICKS    15     /* ... within this many ticks (150 ms)        */
#define GRIP_DROP_WATCH_MS     1000   /* after a fall: watch for the pads meeting   */
#define GRIP_DROP_TRAVEL_RAD   0.01f  /* ... the jaw must close at least this further */
#define GRIP_BLIND_MAX_MS      10000  /* force regulation without tactile ends here */
#define GRIP_LOCKOUT_MS        2000   /* no new grip this soon after an over-force  */

#define PROGRESS_LOG_MS        200
#define HOLD_LOG_MS            1000

#define NVS_NS        "gripper"
#define NVS_KEY       "grip_v5"
#define NVS_VERSION   5u

/* =========================================================================
 * State
 * ========================================================================= */
typedef enum {
    CAL_BASELINE = 0,   /* motor off: record the tactile baseline             */
    CAL_OPEN_SEEK,
    CAL_OPEN_DWELL,     /* at the open stop: average the stall torque         */
    CAL_OPEN_RETRY,     /* open seek could not move: close a little first     */
    CAL_OPEN_BACKOFF,
    CAL_TARE,
    CAL_CLOSE_SEEK,
    CAL_SQUEEZE,
    CAL_CLOSE_BACKOFF,
} cal_phase_t;

typedef enum {
    SEEK_RUNNING = 0,
    SEEK_STOPPED,        /* moved, then stalled                              */
    SEEK_NO_MOTION,      /* never moved, even at the current ceiling         */
    SEEK_TOO_SLOW,       /* creeps far below the commanded speed at the ceiling */
    SEEK_TRAVEL_LIMIT,
    SEEK_DRIVE_FAILED,
} seek_result_t;

typedef struct {
    float    dir;
    float    speed;         /* rad/s, rate of the position-target ramp       */
    float    ramp;          /* current ramp target                           */
    float    start_pos;
    float    cur;           /* current limit in use, pu                      */
    float    cur_max;
    float    cur_moving;    /* current at which motion was established       */
    bool     established;
    bool     has_limit;
    float    limit;         /* never command past this position              */
    uint32_t esc_ms;
    uint32_t no_motion_ms;
    uint32_t stall_ms;
    uint32_t rise_ms;
    uint32_t est_ms;        /* time since motion was established             */
    uint32_t t_ms;          /* time since the seek started                   */
    float    tau_free;      /* |torque| while moving freely, N*m (EMA)       */
    bool     tau_free_valid;
    bool     fast_stop;
    bool     fast_halt;     /* short halt window alone may confirm a stop    */
    float    trim_extra;    /* pu added to the sliding current; <0 = no trim */
    bool     trimmed;
    uint32_t trim_ms;       /* est_ms of the last trim                       */
    float    trim_pos;      /* jaw position at the last trim                 */
    float    tau_peak;      /* peak-hold of the sliding torque, N*m          */
    float    tau_slow;      /* 0.5 s mean of the sliding torque, N*m         */
    float    open_tq_thr;   /* >0: stop when tau_slow reaches it (open seek) */
    bool     tq_stop;       /* stopped by open_tq_thr, not by a stall        */
    float    margin;        /* headroom applied once motion is established   */
} seek_t;

typedef struct {
    uint32_t version;
    float    close_dir;     /* CLOSE_DIR the record was measured with        */
    float    default_force;
    float    span;          /* 0 = no reference                              */
    float    k_force;
    float    i_offset;
    float    i_move;
    float    tau_per_pu;
    float    pad_stiffness;
    /* Positions of the last full calibration, in the drive's frame. The
     * DM-J4310-2EC reports the output shaft's single-turn absolute position
     * (manual p.4: "输出轴单圈绝对位置，不惧掉电"), so they stay valid across
     * power cycles as long as nobody re-zeroes the motor. They are reused
     * only after the home check (gripper_home) has found the open stop where
     * it was. */
    uint32_t pos_valid;     /* 1 = the fields below come from a calibration  */
    float    p_stop;        /* open hard stop                                */
    float    pos_open;
    float    pos_closed;
    float    pos_touch;
    float    approach_speed;
} nvs_rec_t;

static gripper_input_t s_in;
static bool     s_have_in = false;
static uint32_t s_dt = 0;
static uint32_t s_now_ms = 0;

static gripper_state_t s_state = GRIPPER_UNCAL;
static gripper_fault_t s_fault = GRIPPER_FAULT_NONE;
static gripper_calib_t s_cal;      /* active calibration                       */
static gripper_calib_t s_pend;     /* being measured                           */
static nvs_rec_t       s_nvs;      /* last saved record (for comparison)       */
static bool            s_nvs_valid = false;
static uint32_t        s_phase_ms = 0;
static uint32_t        s_stale_ms = 0;
static float           s_stale_hold = 0.0f;
static uint32_t        s_log_ms = 0;
static bool            s_ramp_valid = false;   /* reset on every state/phase change */
static float           s_ramp = 0.0f;
static float           s_ramp_v = 0.0f;        /* ramp_profile(): ramp speed, rad/s   */
static bool            s_ramp_lagging = false; /* ramp_profile(): held back by the jaw */
static float           s_mv_cur = 0.0f;        /* current limit of the running move, pu */
static float           s_mv_ceil = 0.0f;       /* ... raised at most to this, pu       */
static uint32_t        s_mv_raise_ms = 0;
static uint32_t        s_mv_block_ms = 0;
static float           s_mv_p0 = 0.0f;         /* move summary: start position     */
static float           s_mv_v_max = 0.0f;      /* ... fastest jaw speed, rad/s      */
static float           s_mv_tq_max = 0.0f;     /* ... largest |torque|, N*m         */
static float           s_mv_lag_max = 0.0f;    /* ... largest lag behind target     */
static float           s_mv_over = 0.0f;       /* ... farthest past the final target */

static bool     s_autocal_pending = false;
static uint32_t s_autocal_ready_ms = 0;
static uint32_t s_autocal_log_ms = 0;

/* motor output */
static bool     s_motor_on = false;
static uint32_t s_enable_ms = 0;
static bool     s_drove = false;        /* drive() called this tick           */
static float    s_last_tgt = 0.0f, s_last_vel = 0.0f, s_cmd_cur = 0.0f;
static float    s_motion_dir = 0.0f;    /* sign of the last commanded motion  */

/* feedback bookkeeping */
static bool     s_have_epoch = false;
static uint32_t s_epoch_seen = 0;
static bool     s_have_motor = false;
static bool     s_epoch_skip = false;   /* ignore the sample seen at an epoch change */
static uint32_t s_epoch_skip_seq = 0;
static uint32_t s_motor_seq = 0;
static float    s_prev_pos = 0.0f;
#define HIST_LEN 64
static uint32_t s_hist_t[HIST_LEN];
static float    s_hist_p[HIST_LEN];
static float    s_hist_tau[HIST_LEN];   /* |torque| with each position sample */
static unsigned s_hist_head = 0;
static unsigned s_hist_n = 0;
static bool     s_tac_seen[GRIPPER_TACTILE_COUNT];
static uint32_t s_tac_seq[GRIPPER_TACTILE_COUNT];
static bool     s_tac_new[GRIPPER_TACTILE_COUNT];
static bool     s_have_tare_seen = false;
static uint32_t s_tare_seen = 0;

/* calibration */
static cal_phase_t s_cal_phase = CAL_BASELINE;
static bool     s_verify = false;       /* home check: open stop + tare only    */
static seek_t   s_seek;
static int      s_open_retry = 0;
static float    s_retry_dist = CAL_RETRY_BACKOFF_RAD;   /* closing distance of a retry */
static float    s_baseline[GRIPPER_TACTILE_COUNT];
static float    s_base_sum[GRIPPER_TACTILE_COUNT];
static int      s_base_n[GRIPPER_TACTILE_COUNT];
static int      s_touch_cnt[GRIPPER_TACTILE_COUNT];
static int      s_sep_cnt[GRIPPER_TACTILE_COUNT];
static bool     s_touching = false;
static float    s_touch_first = 0.0f;
static uint32_t s_touch_ms = 0;
static bool     s_have_stop = false;
static float    s_p_stop = 0.0f;
static bool     s_have_touch = false;
static float    s_tau_sum = 0.0f;
static int      s_tau_n = 0;
static bool     s_speed_checked = false;
static float    s_backoff_target = 0.0f;
static int      s_backoff_tries = 0;
static uint32_t s_arrived_ms = 0;
static bool     s_tare_requested = false;
static bool     s_tare_done = false;
static uint32_t s_tare_count0 = 0;
static float    s_sq_i0 = 0.0f, s_sq_cur = 0.0f;
static float    s_sq_di = 0.0f;        /* pu per current step                   */
static gripper_fault_t s_cal_note = GRIPPER_FAULT_NONE;   /* reported after calibration */
static int32_t  s_sq_lsb = 0;          /* current limit in CUR_LSB_PU units     */
static int32_t  s_sq_di_lsb = 1;       /* step in CUR_LSB_PU units              */
static float    s_sq_cur_max = 0.0f;   /* current ceiling for this squeeze      */
static uint32_t s_sq_dwell_ms = 0;
static float    s_sq_acc_f = 0.0f, s_sq_acc_p = 0.0f;
static int      s_sq_acc_n = 0;
static bool     s_sq_ramping = false;
static float    s_sq_level_a = 0.0f, s_sq_level_b = 0.0f;
static int      s_sq_n = 0;
static float    s_sq_f_prev = -1.0f;   /* previous mean pad force in the ramp, N */
static float    s_sq_step = 0.0f;      /* largest rise between two samples, N    */
static float    s_sq_step_i = 0.0f;    /* current at that rise, pu               */
static float    s_sq_i[CAL_SQUEEZE_MAX_SAMPLES];
static float    s_sq_f[CAL_SQUEEZE_MAX_SAMPLES];
static float    s_sq_p[CAL_SQUEEZE_MAX_SAMPLES];

/* moves / hold */
static float    s_move_target = 0.0f;
static float    s_move_speed = MOVE_SPEED;
static uint32_t s_move_timeout_ms = MOVE_TIMEOUT_MS;
static float    s_move_f_start = 0.0f;
static bool     s_hold = false;
static float    s_hold_pos = 0.0f;
static float    s_hold_cur = -1.0f;     /* pu; < 0 = move_cap()               */

/* grip */
static bool     s_grip_force_phase = false;
static bool     s_grip_preopen = false;    /* moving to the ready opening first */
static float    s_grip_speed = MOVE_SPEED; /* rad/s, this grip's approach     */
static bool     s_peak_logging = false;    /* pad force peak after contact ...   */
static uint32_t s_peak_ms = 0;
static float    s_peak_f = 0.0f;
static float    s_contact_v = 0.0f;        /* ... and the jaw speed at contact   */
static float    s_appr_tq = 0.0f;          /* approach: decaying peak of the sliding
                                            * torque, N*m (0 = none measured)    */
static float    s_grip_fric = 0.0f;        /* this grip's friction current, pu   */
static float    s_grip_cap_extra = 0.0f;   /* ceiling raised this far, pu        */
static uint32_t s_cap_sat_ms = 0;          /* at the ceiling, below target       */
static float    s_force_target = 0.0f;
static float    s_i_cmd = 0.0f;
static float    s_i_int = 0.0f;
static uint32_t s_settle_ms = 0;
static uint32_t s_lost_ms = 0;
static uint32_t s_mismatch_ms = 0;
static bool     s_soft_logged = false;
static bool     s_unloading = false;
static float    s_unload_tgt = 0.0f;
static bool     s_band_latched = false;
static float    s_band_tgt = 0.0f;
static float    s_drop_ring[GRIP_DROP_WIN_TICKS];   /* recent pad force, one per tick */
static int      s_drop_n = 0, s_drop_head = 0;
static uint32_t s_drop_watch_ms = 0;
static int      s_drop_quiet = 0;      /* ticks with the detector held off      */
static float    s_drop_pos0 = 0.0f;    /* jaw position when the watch started   */
static bool     s_overforce_seen = false;
static uint32_t s_overforce_at = 0;

/* back-off after a fault */
static bool            s_retreat = false;
static float           s_retreat_target = 0.0f;
static uint32_t        s_retreat_ms = 0;
static float           s_retreat_f0 = 0.0f;
static gripper_state_t s_retreat_after = GRIPPER_IDLE;

/* =========================================================================
 * Names and logging
 * ========================================================================= */
const char *gripper_state_name(gripper_state_t s)
{
    switch (s) {
    case GRIPPER_UNCAL:       return "UNCAL";
    case GRIPPER_IDLE:        return "IDLE";
    case GRIPPER_CALIBRATING: return "CALIBRATING";
    case GRIPPER_MOVING:      return "MOVING";
    case GRIPPER_GRIPPING:    return "GRIPPING";
    case GRIPPER_HOLDING:     return "HOLDING";
    case GRIPPER_ERROR:       return "ERROR";
    default:                  return "?";
    }
}

const char *gripper_fault_name(gripper_fault_t f)
{
    switch (f) {
    case GRIPPER_FAULT_NONE:                return "none";
    case GRIPPER_FAULT_MOTOR_NOT_READY:     return "motor not ready";
    case GRIPPER_FAULT_MOTOR_LOST:          return "motor feedback lost";
    case GRIPPER_FAULT_MOTOR_FAULT:         return "motor fault";
    case GRIPPER_FAULT_MOTOR_NOT_ENABLED:   return "motor did not enable";
    case GRIPPER_FAULT_TACTILE_UNAVAILABLE: return "tactile unavailable";
    case GRIPPER_FAULT_TACTILE_STALE:       return "tactile stale while holding";
    case GRIPPER_FAULT_FORCE_WHILE_OPENING: return "force rose while opening";
    case GRIPPER_FAULT_NO_MOTION:           return "jaw does not move";
    case GRIPPER_FAULT_TRAVEL_LIMIT:        return "travel limit";
    case GRIPPER_FAULT_TIMEOUT:             return "timeout";
    case GRIPPER_FAULT_TARE:                return "tactile tare failed";
    case GRIPPER_FAULT_BASELINE:            return "tactile not zero after tare";
    case GRIPPER_FAULT_ONE_SIDED_CONTACT:   return "one-sided contact";
    case GRIPPER_FAULT_STALL_NO_CONTACT:    return "stalled without tactile contact";
    case GRIPPER_FAULT_CONTACT_TOO_HARD:    return "contact force too high";
    case GRIPPER_FAULT_GAIN_INVALID:        return "force/current gain invalid";
    case GRIPPER_FAULT_NOT_SEPARATED:       return "pads did not separate";
    case GRIPPER_FAULT_SPAN_INVALID:        return "stroke invalid";
    case GRIPPER_FAULT_OVERFORCE:           return "over-force";
    case GRIPPER_FAULT_CONTACT_DURING_MOVE: return "contact during move";
    case GRIPPER_FAULT_EMPTY_GRASP:         return "empty grasp";
    case GRIPPER_FAULT_GRIP_NO_TACTILE:     return "grip stalled without pad force";
    case GRIPPER_FAULT_POSITION_JUMP:       return "position jump";
    case GRIPPER_FAULT_SPEED_MISMATCH:      return "speed mismatch";
    case GRIPPER_FAULT_FORCE_MISMATCH:      return "pad force far below current";
    case GRIPPER_FAULT_TACTILE_RETARED:     return "tactile re-zeroed while driving";
    case GRIPPER_FAULT_SPAN_CHANGED:        return "stroke changed";
    case GRIPPER_FAULT_FRAME_CHANGED:       return "position frame changed";
    default:                                return "?";
    }
}

static const char *cal_phase_name(cal_phase_t p)
{
    switch (p) {
    case CAL_BASELINE:      return "baseline";
    case CAL_OPEN_SEEK:     return "open-seek";
    case CAL_OPEN_DWELL:    return "open-dwell";
    case CAL_OPEN_RETRY:    return "open-retry";
    case CAL_OPEN_BACKOFF:  return "open-backoff";
    case CAL_TARE:          return "tare";
    case CAL_CLOSE_SEEK:    return "close-seek";
    case CAL_SQUEEZE:       return "squeeze";
    case CAL_CLOSE_BACKOFF: return "close-backoff";
    default:                return "?";
    }
}

#if defined(__GNUC__)
static void log_fault(bool error, gripper_fault_t f, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));
#endif
static void log_fault(bool error, gripper_fault_t f, const char *fmt, ...)
{
    char buf[320];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (error) {
        ESP_LOGE(TAG, "%s: %s", gripper_fault_name(f), buf);
    } else {
        ESP_LOGW(TAG, "%s: %s", gripper_fault_name(f), buf);
    }
}

/* =========================================================================
 * Small helpers
 * ========================================================================= */
static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static float sgnf(float x)
{
    return x > 0.0f ? 1.0f : (x < 0.0f ? -1.0f : 0.0f);
}

static float dt_s(void)
{
    return (float)s_dt / 1000.0f;
}

static float vel_field(float speed)
{
    return speed * VEL_FIELD_SCALE;
}

static bool tac_both_ok(void)
{
    return s_in.tac_ok[0] && s_in.tac_ok[1];
}

/* Largest pad force among the pads that are currently trustworthy. */
static float tac_max_ok(void)
{
    float m = 0.0f;
    for (int i = 0; i < GRIPPER_TACTILE_COUNT; i++) {
        if (s_in.tac_ok[i] && s_in.fz[i] > m) m = s_in.fz[i];
    }
    return m;
}

static float rel(int i)
{
    return s_in.fz[i] - s_baseline[i];
}

static bool any_rel_above(float thr)
{
    for (int i = 0; i < GRIPPER_TACTILE_COUNT; i++) {
        if (s_in.tac_ok[i] && rel(i) > thr) return true;
    }
    return false;
}

static bool calibrated(void)
{
    return s_cal.valid && s_have_epoch && s_cal.motor_epoch == s_in.motor_epoch;
}

static float range_lo(void) { return fminf(s_cal.pos_closed, s_cal.pos_open); }
static float range_hi(void) { return fmaxf(s_cal.pos_closed, s_cal.pos_open); }

/* Position at a closing percentage: 0 = open position, 100 = closed. */
static float pos_at_pct(float pct)
{
    return s_cal.pos_open + (pct / 100.0f) * (s_cal.pos_closed - s_cal.pos_open);
}

static void hist_reset(void)
{
    s_hist_head = 0;
    s_hist_n = 0;
}

static void hist_push(uint32_t t, float p, float tau)
{
    s_hist_t[s_hist_head] = t;
    s_hist_p[s_hist_head] = p;
    s_hist_tau[s_hist_head] = fabsf(tau);
    s_hist_head = (s_hist_head + 1u) % HIST_LEN;
    if (s_hist_n < HIST_LEN) s_hist_n++;
}

/* Position change over the last window_ms of feedback; false until the
 * history covers the window. */
static bool hist_delta(uint32_t window_ms, float *out)
{
    if (s_hist_n < 2) return false;
    unsigned newest = (s_hist_head + HIST_LEN - 1u) % HIST_LEN;
    for (unsigned k = 1; k < s_hist_n; k++) {
        unsigned idx = (newest + HIST_LEN - k) % HIST_LEN;
        if (s_hist_t[newest] - s_hist_t[idx] >= window_ms) {
            *out = s_hist_p[newest] - s_hist_p[idx];
            return true;
        }
    }
    return false;
}

/* |torque| of the newest sample at least window_ms older than the newest. */
static bool hist_tau_before(uint32_t window_ms, float *out)
{
    if (s_hist_n < 2) return false;
    unsigned newest = (s_hist_head + HIST_LEN - 1u) % HIST_LEN;
    for (unsigned k = 1; k < s_hist_n; k++) {
        unsigned idx = (newest + HIST_LEN - k) % HIST_LEN;
        if (s_hist_t[newest] - s_hist_t[idx] >= window_ms) {
            *out = s_hist_tau[idx];
            return true;
        }
    }
    return false;
}

/* Window long enough that @p dist is covered at @p speed. */
static uint32_t win_ms(float dist, float speed)
{
    float ms = speed > 1e-4f ? 1000.0f * dist / speed : (float)WIN_MAX_MS;
    return (uint32_t)clampf(ms, (float)WIN_MIN_MS, (float)WIN_MAX_MS);
}

static uint32_t moving_win_ms(float speed)
{
    return win_ms(10.0f * POS_LSB_RAD / MOTION_FRAC, speed);
}

/* Moving in @p dir at no less than MOTION_FRAC of @p speed. */
static bool is_moving(float dir, float speed)
{
    uint32_t w = moving_win_ms(speed);
    float d;
    if (!hist_delta(w, &d)) return false;
    return d * dir >= fmaxf(MOTION_FRAC * speed * (float)w / 1000.0f, 3.0f * POS_LSB_RAD);
}

/* Any steady motion in @p dir, however slow, over the longest window. */
static bool is_creeping(float dir)
{
    float d;
    return hist_delta(WIN_MAX_MS, &d) && d * dir >= CREEP_MIN_RAD;
}

/* Position stood still over the short STOP_HALT_MS window. */
static bool is_halted(void)
{
    float d;
    return hist_delta(STOP_HALT_MS, &d) && fabsf(d) < STOP_HALT_RAD;
}

/* Slower than STALL_FRAC of @p speed (and than a few feedback steps). */
static bool is_stalled(float speed)
{
    uint32_t w = win_ms(3.0f * POS_LSB_RAD / STALL_FRAC, speed);
    float d;
    if (!hist_delta(w, &d)) return false;
    return fabsf(d) < fmaxf(STALL_FRAC * speed * (float)w / 1000.0f, 3.0f * POS_LSB_RAD);
}

static void set_state(gripper_state_t s)
{
    if (s != s_state) {
        ESP_LOGI(TAG, "%s -> %s", gripper_state_name(s_state), gripper_state_name(s));
        s_state = s;
    }
    s_phase_ms = 0;
    s_stale_ms = 0;
    s_ramp_valid = false;
}

static void cal_set_phase(cal_phase_t p)
{
    s_cal_phase = p;
    s_phase_ms = 0;
    s_stale_ms = 0;
    s_ramp_valid = false;
    ESP_LOGI(TAG, "calibration: %s", cal_phase_name(p));
}

/* =========================================================================
 * Motor output
 * ========================================================================= */
static bool drive(float target, float vel, float cur)
{
    if (!isfinite(target) || !isfinite(vel) || !isfinite(cur)) {
        return false;
    }
    cur = clampf(cur, 0.0f, DRIVE_CUR_ABS_MAX_PU);
    s_last_tgt = target;
    s_last_vel = vel;
    s_cmd_cur = cur;
    s_drove = true;
    motor_twai_set_force_pos(target, vel, cur);
    if (!s_motor_on) {
        if (!motor_twai_enable()) {
            return false;
        }
        s_motor_on = true;
        s_enable_ms = 0;
    }
    return true;
}

static void motor_off(void)
{
    motor_twai_disable();
    s_motor_on = false;
    s_cmd_cur = 0.0f;
}

/* Advance a position target toward @p target at @p speed (rad/s), never more
 * than RAMP_MAX_LEAD_RAD ahead of the jaw, so a blocked jaw cannot let the
 * target run away and a released one cannot jump far. */
static float ramp_toward(float target, float speed)
{
    if (!s_ramp_valid) {
        s_ramp = s_in.pos;
        s_ramp_valid = true;
    }
    float step = speed * dt_s();
    float d = target - s_ramp;
    s_ramp += fabsf(d) <= step ? d : sgnf(d) * step;
    float lead = s_ramp - s_in.pos;
    if (lead > RAMP_MAX_LEAD_RAD) s_ramp = s_in.pos + RAMP_MAX_LEAD_RAD;
    if (lead < -RAMP_MAX_LEAD_RAD) s_ramp = s_in.pos - RAMP_MAX_LEAD_RAD;
    s_ramp_v = 0.0f;
    return s_ramp;
}

/* Position target for a move: toward @p target at up to @p vmax, speeding up
 * and braking at @p amax so that it arrives at rest. It leads the jaw by at
 * most RAMP_LEAD_S of travel at the ramp speed; when the jaw holds it back,
 * s_ramp_lagging is set and the ramp speed follows the jaw's. */
static float ramp_profile(float target, float vmax, float amax)
{
    if (!s_ramp_valid) {
        s_ramp = s_in.pos;
        s_ramp_v = 0.0f;
        s_ramp_valid = true;
    }
    const float dt = dt_s();
    const float d = target - s_ramp;
    const float v_goal = sgnf(d) * fminf(vmax, sqrtf(2.0f * amax * fabsf(d)));
    const float dv = amax * dt;
    s_ramp_v += clampf(v_goal - s_ramp_v, -dv, dv);
    const float step = s_ramp_v * dt;
    if (step * d >= 0.0f && fabsf(step) >= fabsf(d)) {
        s_ramp = target;
        s_ramp_v = 0.0f;
    } else {
        s_ramp += step;
    }
    const float lead_max = fmaxf(RAMP_MAX_LEAD_RAD, fabsf(s_ramp_v) * RAMP_LEAD_S);
    const float lead = s_ramp - s_in.pos;
    s_ramp_lagging = false;
    if (fabsf(lead) > lead_max) {
        s_ramp = s_in.pos + sgnf(lead) * lead_max;
        if (lead * s_ramp_v > 0.0f) {
            s_ramp_lagging = true;
            float dj = 0.0f;
            const float vj = hist_delta(WIN_MIN_MS, &dj) ? fabsf(dj) * 1000.0f / (float)WIN_MIN_MS : 0.0f;
            s_ramp_v = sgnf(s_ramp_v) * fminf(fabsf(s_ramp_v), vj + dv);
        }
    }
    return s_ramp;
}

/* Current for an output torque, from the motor's register estimate
 * (Kt x Imax, read at the probe); @p fallback_pu when there is no estimate.
 * Never above twice the fallback, so a wrong, low estimate cannot raise a
 * ceiling far. */
static float pu_for_nm(float nm, float fallback_pu)
{
    float t = s_in.motor_tau_per_pu_est;
    return (isfinite(t) && t > 1.0f) ? fminf(nm / t, fallback_pu * 2.0f) : fallback_pu;
}

static float cal_cur_max(void)
{
    return pu_for_nm(CAL_TQ_MAX_NM, CAL_CUR_MAX_PU);
}

/* Current that moved the jaw, from this or the active calibration. */
static float i_move_known(void)
{
    if (s_pend.i_move > 0.0f) return s_pend.i_move;
    if (s_cal.i_move > 0.0f) return s_cal.i_move;
    return CAL_CUR_START_PU;
}

/* Slow positioning inside calibration and for back-offs. */
static float creep_cap(void)
{
    return clampf(CAL_MOVING_MARGIN * i_move_known(), CAL_CUR_START_PU, cal_cur_max());
}

/* Current that overcomes friction plus @p force_n of push, from the measured
 * force per current. Before the gain is known, fall back to creep_cap(). */
static float friction_plus(float force_n, float ceiling)
{
    if (!(s_cal.k_force > 0.0f)) {
        return creep_cap();
    }
    float base = fmaxf(i_move_known(), s_cal.i_offset);
    return clampf(base + force_n / s_cal.k_force, CAL_CUR_START_PU, ceiling);
}

/* Free moves and holding a position: an unexpected obstacle is pushed with
 * about MOVE_FORCE_CAP_N beyond friction. */
static float move_cap(void)
{
    return friction_plus(MOVE_FORCE_CAP_N, pu_for_nm(MOVE_TQ_MAX_NM, MOVE_CUR_MAX_PU));
}

/* Opening moves: the opening torque rises toward the open end, above the cap
 * from the moving current (bench 2026-10-23: 0.0149 pu = 0.29 N*m stopped the
 * move to open at 2.909 rad, 30 % open). Moves run only once calibrated, and
 * the target stays inside the calibrated open position, so the soft end is
 * not pushed: start at MOVE_TQ_MAX_NM (user's choice), raised while the
 * motion needs more (move_current_step()). Opening presses nothing between
 * the pads; something blocking the outside of the fingers meets the limit at
 * that moment, 1 N*m / 10.4 mm = about 96 N or more. */
static float open_move_cap(void)
{
    return pu_for_nm(MOVE_TQ_MAX_NM, MOVE_CUR_MAX_PU);
}

/* Current limit at the start of a move, and how far it may be raised when
 * the motion needs more (move_current_step()). Closing: friction plus
 * MOVE_FORCE_CAP_N, raised at most to MOVE_TQ_MAX_NM; contact is stopped by
 * the tactile reading. Opening: 1 N*m, raised at most to MOVE_OPEN_TQ_MAX_NM. */
static void move_current_init(bool closing)
{
    s_mv_cur = closing ? move_cap() : open_move_cap();
    s_mv_ceil = closing ? pu_for_nm(MOVE_TQ_MAX_NM, MOVE_CUR_MAX_PU)
                        : pu_for_nm(MOVE_OPEN_TQ_MAX_NM, MOVE_OPEN_CUR_MAX_PU);
    s_mv_ceil = fmaxf(s_mv_ceil, s_mv_cur);
    s_mv_raise_ms = 0;
    s_mv_block_ms = 0;
    s_mv_p0 = s_in.pos;
    s_mv_v_max = 0.0f;
    s_mv_tq_max = 0.0f;
    s_mv_lag_max = 0.0f;
    s_mv_over = 0.0f;
}

/* Keep a commanded position inside what is known to be free travel: the
 * calibrated range, or during calibration never nearer the open stop than
 * CAL_STOP_MARGIN_RAD and never past the pad touch. */
static float clamp_travel(float target)
{
    if (calibrated()) {
        return clampf(target, range_lo(), range_hi());
    }
    if (s_have_stop) {
        float lim = s_p_stop + CLOSE_DIR * CAL_STOP_MARGIN_RAD;
        if ((target - lim) * CLOSE_DIR < 0.0f) target = lim;
    }
    if (s_have_touch) {
        float lim = s_pend.pos_touch;
        if ((target - lim) * CLOSE_DIR > 0.0f) target = lim;
    }
    return target;
}

static void hold_here(void)
{
    s_hold = true;
    s_hold_cur = -1.0f;
    s_hold_pos = calibrated() ? clampf(s_in.pos, range_lo(), range_hi()) : s_in.pos;
}

/* =========================================================================
 * Faults
 * ========================================================================= */
static void enter_error(gripper_fault_t f)
{
    s_fault = f;
    motor_off();
    s_retreat = false;
    s_hold = false;
    s_grip_force_phase = false;
    s_grip_preopen = false;
    s_force_target = 0.0f;
    set_state(GRIPPER_ERROR);
}

/* Back off by RETREAT_RAD in @p dir at a low current for up to RETREAT_MS,
 * then hold (after = IDLE) or switch off (after = ERROR). The direction is
 * chosen at each fault site by what the fault means (away from the pads, or
 * away from whatever the jaw was pushing), never by the last motion: the last
 * motion may have been a back-off from the very hazard. */
static void start_retreat(gripper_fault_t f, gripper_state_t after, float dir)
{
    s_fault = f;
    s_grip_force_phase = false;
    s_grip_preopen = false;
    s_force_target = 0.0f;
    if (!s_motor_on || dir == 0.0f) {
        if (after == GRIPPER_ERROR) {
            enter_error(f);
        } else {
            hold_here();
            set_state(GRIPPER_IDLE);
        }
        return;
    }
    s_retreat = true;
    s_ramp_valid = false;
    s_retreat_target = clamp_travel(s_in.pos + dir * RETREAT_RAD);
    /* While opening (and in the retry), the only travel known to be free is
     * what the current seek has covered: never back off past where it began.
     * If CLOSE_DIR is reversed, the "opening" seek has been closing the pads
     * and the way back is exactly that stretch. */
    if (s_state == GRIPPER_CALIBRATING &&
        (s_cal_phase == CAL_OPEN_SEEK || s_cal_phase == CAL_OPEN_DWELL || s_cal_phase == CAL_OPEN_RETRY) &&
        (s_retreat_target - s_seek.start_pos) * dir > 0.0f) {
        s_retreat_target = s_seek.start_pos;
    }
    s_retreat_ms = 0;
    s_retreat_f0 = tac_max_ok();
    s_retreat_after = after;
    ESP_LOGW(TAG, "backing off to %.3f rad", s_retreat_target);
}

#define ERROR_OUT(f, ...)               do { log_fault(true, (f), __VA_ARGS__); enter_error(f); } while (0)
#define RETREAT_OUT(f, after, dir, ...) do { log_fault(true, (f), __VA_ARGS__); start_retreat((f), (after), (dir)); } while (0)
#define NOTE_FAULT(f, ...)              do { log_fault(false, (f), __VA_ARGS__); s_fault = (f); } while (0)

/* Direction away from danger for a fault raised during calibration: before
 * the open stop is known it is "against the seek" (also right if CLOSE_DIR is
 * reversed); from the open back-off on, the pads are the only thing ahead. */
static float cal_away_dir(void)
{
    if (s_cal_phase == CAL_OPEN_SEEK || s_cal_phase == CAL_OPEN_DWELL ||
        s_cal_phase == CAL_OPEN_RETRY) {
        return -s_seek.dir;
    }
    return -CLOSE_DIR;
}

static void retreat_tick(void)
{
    s_retreat_ms += s_dt;
    if (s_retreat_ms <= RETREAT_REF_MS) {
        s_retreat_f0 = fmaxf(s_retreat_f0, tac_max_ok());
    } else if (tac_max_ok() > s_retreat_f0 + RETREAT_FORCE_RISE_N) {
        log_fault(true, s_fault, "pad force rose to %.2f N while backing off - motor off", tac_max_ok());
        enter_error(s_fault);
        return;
    }
    if (!drive(ramp_toward(s_retreat_target, RETREAT_SPEED), vel_field(RETREAT_SPEED), creep_cap())) {
        ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused during back-off");
        return;
    }
    bool arrived = fabsf(s_in.pos - s_retreat_target) < MOVE_POS_TOL_RAD && is_stalled(RETREAT_SPEED);
    if (arrived || s_retreat_ms >= RETREAT_MS) {
        s_retreat = false;
        if (s_retreat_after == GRIPPER_ERROR) {
            enter_error(s_fault);
        } else {
            hold_here();
            set_state(GRIPPER_IDLE);
        }
    }
}

/* Hard limits on the tactile readings, checked every tick while the motor is
 * driven. Returns true after reacting. */
static bool overforce_check(void)
{
    static uint8_t over_n[GRIPPER_TACTILE_COUNT];
    for (int i = 0; i < GRIPPER_TACTILE_COUNT; i++) {
        if (!s_in.tac_ok[i]) {
            over_n[i] = 0;
            continue;
        }
        bool pad = s_in.fz[i] >= FORCE_TRIP_N;
        bool point = s_in.fz_point_max[i] >= TAC_POINT_LIMIT_N;
        if (s_tac_new[i]) {
            over_n[i] = (pad || point) ? (uint8_t)(over_n[i] < 255 ? over_n[i] + 1 : 255) : 0;
        }
        if (over_n[i] < FORCE_TRIP_SAMPLES) continue;
        over_n[i] = 0;

        if (s_state == GRIPPER_CALIBRATING) {
            RETREAT_OUT(GRIPPER_FAULT_OVERFORCE, GRIPPER_ERROR, cal_away_dir(),
                        "pad %d: %.1f N (single point %.1f N)", i, s_in.fz[i], s_in.fz_point_max[i]);
        } else {
            s_overforce_seen = true;
            s_overforce_at = s_now_ms;
            RETREAT_OUT(GRIPPER_FAULT_OVERFORCE, GRIPPER_IDLE, -CLOSE_DIR,
                        "pad %d: %.1f N (single point %.1f N), limits %.1f / %.1f N",
                        i, s_in.fz[i], s_in.fz_point_max[i], FORCE_TRIP_N, TAC_POINT_LIMIT_N);
        }
        return true;
    }
    return false;
}

/* =========================================================================
 * Seek: move in one direction at the smallest current that moves the jaw,
 * and report where it stops.
 * ========================================================================= */
static void seek_init(seek_t *sk, float dir, float speed, float cur_start, float cur_max)
{
    memset(sk, 0, sizeof(*sk));
    sk->dir = dir;
    sk->speed = speed;
    sk->ramp = s_in.pos;
    sk->start_pos = s_in.pos;
    sk->cur_max = cur_max;
    sk->cur = clampf(cur_start, CAL_CUR_START_PU, cur_max);
    sk->trim_extra = -1.0f;
    sk->margin = CAL_MOVING_MARGIN;
    s_motion_dir = dir;
}

/* N*m per pu measured at the open stop (this calibration or the active one). */
static float tau_per_pu_known(void)
{
    if (s_pend.tau_per_pu > 0.0f) return s_pend.tau_per_pu;
    if (s_cal.tau_per_pu > 0.0f) return s_cal.tau_per_pu;
    /* not measured yet: the motor's own Kt x Imax (bench: within 0.5 %) */
    if (isfinite(s_in.motor_tau_per_pu_est) && s_in.motor_tau_per_pu_est > 1.0f) return s_in.motor_tau_per_pu_est;
    return 0.0f;
}

static seek_result_t seek_step(seek_t *sk)
{
    const float pos = s_in.pos;
    /* Target ramp at sk->speed, kept between the jaw and SEEK_LEAD_RAD ahead. */
    sk->ramp += sk->dir * sk->speed * dt_s();
    if ((sk->ramp - pos) * sk->dir > SEEK_LEAD_RAD) sk->ramp = pos + sk->dir * SEEK_LEAD_RAD;
    if ((sk->ramp - pos) * sk->dir < 0.0f) sk->ramp = pos;
    float target = sk->ramp;
    if (sk->has_limit && (target - sk->limit) * sk->dir > 0.0f) {
        target = sk->limit;
    }
    if (!drive(target, vel_field(sk->speed), sk->cur)) {
        return SEEK_DRIVE_FAILED;
    }
    if (fabsf(pos - sk->start_pos) > CAL_MAX_TRAVEL_RAD) {
        return SEEK_TRAVEL_LIMIT;
    }
    sk->t_ms += s_dt;

    if (!sk->established) {
        /* Before the jaw has moved, a stall means "current too low", never
         * "hard stop": raise the current step by step up to the ceiling. */
        if ((pos - sk->start_pos) * sk->dir >= SEEK_MOTION_RAD) {
            sk->established = true;
            sk->cur_moving = sk->cur;
            sk->cur = fminf(sk->cur * sk->margin, sk->cur_max);
            if (sk->open_tq_thr > 0.0f) {
                /* The opening end is where the torque reaches open_tq_thr, so
                 * the limit must lie above it. A limit from the breakaway near
                 * the pads does not (bench 2026-10-23: 0.010 pu breakaway, 0.015
                 * pu = 0.29 N*m limit, stalled at 4.794 rad on the rising
                 * opening torque and stored a 0.135 rad stroke). */
                const float tpu = tau_per_pu_known();
                const float c = tpu > 0.0f ? CAL_OPEN_TQ_CUR_FACTOR * sk->open_tq_thr / tpu : sk->cur_max;
                sk->cur = fminf(fmaxf(sk->cur, c), sk->cur_max);
            }
            sk->est_ms = 0;
            ESP_LOGI(TAG, "jaw moves at %.3f pu, continuing at %.3f pu", sk->cur_moving, sk->cur);
            return SEEK_RUNNING;
        }
        if (is_moving(sk->dir, sk->speed)) {
            sk->esc_ms = 0;
            return SEEK_RUNNING;
        }
        sk->esc_ms += s_dt;
        if (sk->esc_ms >= CAL_ESC_INTERVAL_MS) {
            sk->esc_ms = 0;
            if (sk->cur < sk->cur_max) {
                sk->cur = fminf(sk->cur * CAL_ESC_FACTOR, sk->cur_max);
                ESP_LOGI(TAG, "not moving, current limit -> %.3f pu", sk->cur);
            } else {
                sk->no_motion_ms += CAL_ESC_INTERVAL_MS;
                if (sk->no_motion_ms >= CAL_NO_MOTION_MS) {
                    /* At the ceiling and not moving at the commanded pace.
                     * Still creeping steadily: "moves, but far too slowly".
                     * Moved a little and then stopped (e.g. booted just short
                     * of a stop): "does not move", which takes the retry path. */
                    return is_creeping(sk->dir) ? SEEK_TOO_SLOW : SEEK_NO_MOTION;
                }
            }
        }
        return SEEK_RUNNING;
    }

    /* After the jaw has moved, a stall is a stop. The torque feedback comes
     * from the drive's current measurement; with the speed bounded by the
     * vel_limit field, the free-running torque is the sliding friction, and at
     * a stop the speed loop drives the current to the limit, so a rise above
     * the free torque while the position stands still confirms it. */
    sk->est_ms += s_dt;
    const float tau = fabsf(s_in.torque);
    /* The free-running torque is averaged from the sample one motion window
     * back: is_moving() still holds for up to that window after the jaw has
     * met a stop, and averaging the current sample would absorb the rise the
     * stop produces (the reference then equals the stall torque and the rise
     * is never seen). */
    float tau_old;
    if (is_moving(sk->dir, sk->speed) && hist_tau_before(moving_win_ms(sk->speed), &tau_old)) {
        if (!sk->tau_free_valid) {
            sk->tau_free = sk->tau_slow = sk->tau_peak = tau_old;
        }
        sk->tau_free = 0.9f * sk->tau_free + 0.1f * tau_old;
        sk->tau_slow = (1.0f - SEEK_TQ_SLOW_ALPHA) * sk->tau_slow + SEEK_TQ_SLOW_ALPHA * tau_old;
        sk->tau_peak = fmaxf(tau_old, sk->tau_peak - SEEK_PEAK_DECAY_NM_S * dt_s());
        sk->tau_free_valid = true;
    }
    /* Opening end of this gripper: no hard stop, the opening torque rises
     * steadily. The opening limit is where its mean reaches open_tq_thr. */
    if (sk->open_tq_thr > 0.0f && sk->tau_free_valid &&
        ((sk->est_ms >= CAL_OPEN_TQ_MIN_MS && sk->tau_slow >= sk->open_tq_thr) ||
         (sk->est_ms >= CAL_OPEN_TQ_EARLY_MS && sk->tau_free >= CAL_OPEN_TQ_EARLY_FACTOR * sk->open_tq_thr))) {
        sk->tq_stop = true;
        return SEEK_STOPPED;
    }
    /* Breaking free can take far more current than sliding: once moving at
     * the commanded speed, the drive's current is what sliding friction
     * needs (the torque feedback measures it). Trim the limit to a margin
     * above its PEAK (the reading ripples by +-0.05 N*m on the bench), so the
     * surplus does not become contact force. Repeated every SEEK_TRIM_MS: it
     * follows the local friction down at once and up at no more than
     * SEEK_TRIM_RISE_NM_RAD per rad travelled. Samples are taken only while
     * moving freely, one motion window back. */
    if (sk->trim_extra >= 0.0f && sk->tau_free_valid && sk->est_ms >= SEEK_TRIM_MS &&
        (!sk->trimmed || sk->est_ms - sk->trim_ms >= SEEK_TRIM_MS)) {
        float tpu = tau_per_pu_known();
        if (tpu > 0.0f) {
            float i_peak = sk->tau_peak / tpu;
            float c = fminf(SEEK_TRIM_FACTOR * i_peak + sk->trim_extra,
                            i_peak + SEEK_EXCESS_FORCE_N * GRIP_LEVER_NOMINAL_M / tpu);
            if (!sk->trimmed) {
                c = fminf(c, sk->cur);            /* the first trim only lowers */
            } else if (c > sk->cur) {
                c = fminf(c, sk->cur + SEEK_TRIM_RISE_NM_RAD * fabsf(pos - sk->trim_pos) / tpu);
            }
            c = clampf(c, CAL_CUR_START_PU, sk->cur_max);
            if (!sk->trimmed || c < 0.9f * sk->cur || c > 1.1f * sk->cur) {
                ESP_LOGI(TAG, "sliding needs %.3f pu (mean %.3f, peak %.3f N*m): limit %.3f -> %.3f pu",
                         sk->tau_free / tpu, sk->tau_free, sk->tau_peak, sk->cur, c);
            }
            sk->cur = c;
        }
        sk->trimmed = true;
        sk->trim_ms = sk->est_ms;
        sk->trim_pos = pos;
    }
    /* Fast path (the current): the torque has risen above the free-running
     * value and the position has not changed over STOP_HALT_MS. The drive
     * raises the current to its limit only when the speed loop cannot reach
     * the commanded speed, so the rise is the primary evidence; the short
     * position window confirms it within about 0.13 s of the impact. */
    bool rise = sk->tau_free_valid && tau >= sk->tau_free * STOP_TQ_RATIO + STOP_TQ_MARGIN_NM;
    /* At low speed the 100 ms window accepts up to 0.016 rad/s, which is half
     * the slowest grip approach: a stick-slip pause would read as a stop. There
     * the long stall window must agree too. The open seek keeps the short
     * window: a false stop there is caught by the dwell (the jaw moves on). */
    bool halted = is_halted() &&
                  (sk->fast_halt || sk->speed >= STOP_HALT_MIN_SPEED || is_stalled(sk->speed));
    sk->rise_ms = (rise && halted) ? sk->rise_ms + s_dt : 0;
    if (sk->rise_ms >= STOP_FAST_MS) { sk->fast_stop = true; return SEEK_STOPPED; }
    if (is_stalled(sk->speed)) {
        /* Slow path: no torque rise, so also require that the jaw is not
         * creeping on the long window - a jaw that is merely slow is not at
         * a stop. */
        sk->stall_ms = is_creeping(sk->dir) ? 0 : sk->stall_ms + s_dt;
        if (sk->stall_ms >= STOP_SLOW_MS) { sk->fast_stop = false; return SEEK_STOPPED; }
    } else {
        sk->stall_ms = 0;
    }
    return SEEK_RUNNING;
}

/* =========================================================================
 * NVS: default force and the last calibration. The gains are compared with a
 * new calibration; the positions are reused only through the home check.
 * ========================================================================= */
static void nvs_load(void)
{
    s_nvs_valid = false;
    nvs_handle_t nh;
    if (nvs_open(NVS_NS, NVS_READONLY, &nh) != ESP_OK) {
        return;
    }
    nvs_rec_t tmp;
    size_t sz = sizeof(tmp);
    if (nvs_get_blob(nh, NVS_KEY, &tmp, &sz) == ESP_OK && sz == sizeof(tmp) &&
        tmp.version == NVS_VERSION) {
        s_nvs = tmp;
        s_nvs_valid = true;
    }
    nvs_close(nh);
}

static void nvs_store(void)
{
    nvs_handle_t nh;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nh);
    if (err == ESP_OK) {
        s_nvs.version = NVS_VERSION;
        err = nvs_set_blob(nh, NVS_KEY, &s_nvs, sizeof(s_nvs));
        if (err == ESP_OK) err = nvs_commit(nh);
        nvs_close(nh);
    }
    if (err == ESP_OK) {
        s_nvs_valid = true;
    } else {
        ESP_LOGW(TAG, "NVS save failed: %s", esp_err_to_name(err));
    }
}

/* A saved stroke exists only after a calibration that succeeded with the
 * current CLOSE_DIR, so it also proves the direction is right. */
static bool have_prev_span(void)
{
    return s_nvs_valid && s_nvs.close_dir == CLOSE_DIR && isfinite(s_nvs.span) &&
           s_nvs.span > CAL_SPAN_MIN_RAD;
}

/* A stored calibration the home check can restore. */
static bool have_stored_cal(void)
{
    return have_prev_span() && s_nvs.pos_valid == 1u &&
           isfinite(s_nvs.p_stop) && isfinite(s_nvs.pos_open) && isfinite(s_nvs.pos_closed) &&
           isfinite(s_nvs.pos_touch) && isfinite(s_nvs.approach_speed) && s_nvs.approach_speed > 0.0f &&
           s_nvs.k_force >= CAL_K_MIN && s_nvs.k_force <= CAL_K_MAX && s_nvs.i_move > 0.0f &&
           fabsf((s_nvs.pos_touch - s_nvs.pos_open) * CLOSE_DIR - s_nvs.span) < 1e-3f;
}

/* =========================================================================
 * Calibration
 * ========================================================================= */

/* Calibration must see both pads. If either goes stale, stop where we are; if
 * it stays stale, give up with the motor off. Returns true when both are ok. */
static bool cal_tactile_guard(void)
{
    if (tac_both_ok()) {
        s_stale_ms = 0;
        return true;
    }
    if (s_stale_ms == 0) {
        s_stale_hold = s_in.pos;
    }
    s_stale_ms += s_dt;
    if (s_motor_on) {
        (void)drive(s_stale_hold, vel_field(CAL_TOUCH_SPEED), creep_cap());
    }
    if (s_stale_ms >= TAC_STALE_FAULT_MS) {
        ERROR_OUT(GRIPPER_FAULT_TACTILE_UNAVAILABLE, "ring %s, little %s for %u ms during %s",
                  s_in.tac_ok[0] ? "ok" : "stale", s_in.tac_ok[1] ? "ok" : "stale",
                  (unsigned)s_stale_ms, cal_phase_name(s_cal_phase));
    }
    return false;
}

/* Position move used inside calibration. 1 = arrived, 0 = running,
 * -1 = failed (already handled: motor off, no back-off, because a move that
 * does not arrive has met something). */
static int cal_move_step(float target, float speed)
{
    if (!drive(ramp_toward(target, speed), vel_field(speed), creep_cap())) {
        ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused in %s", cal_phase_name(s_cal_phase));
        return -1;
    }
    float err = fabsf(s_in.pos - target);
    if (err < MOVE_POS_TOL_RAD && is_stalled(speed)) {
        return 1;
    }
    if (s_phase_ms > CAL_BACKOFF_TIMEOUT_MS) {
        if (err < 3.0f * MOVE_POS_TOL_RAD) {
            return 1;
        }
        ERROR_OUT(GRIPPER_FAULT_TIMEOUT, "%s: %.3f rad short of %.3f", cal_phase_name(s_cal_phase), err, target);
        return -1;
    }
    return 0;
}

/* Median of n values (sorts @p a in place; n is small). */
static float median_inplace(float *a, int n)
{
    for (int i = 1; i < n; i++) {
        float v = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; }
        a[j + 1] = v;
    }
    return (n & 1) ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2]);
}

/* Theil-Sen line y = slope * x + icept: the median of the pairwise slopes and
 * the median intercept. A few stuck or jumped points (friction holding the
 * force over a step, a slip) do not move it, unlike least squares. */
static float s_ts_buf[CAL_SQUEEZE_MAX_SAMPLES * (CAL_SQUEEZE_MAX_SAMPLES - 1) / 2];
static bool ts_line(const float *x, const float *y, int n, float *slope, float *icept)
{
    int m = 0;
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            float dx = x[j] - x[i];
            if (fabsf(dx) > 1e-9f) s_ts_buf[m++] = (y[j] - y[i]) / dx;
        }
    }
    if (m < 1) return false;
    *slope = median_inplace(s_ts_buf, m);
    for (int i = 0; i < n; i++) s_ts_buf[i] = y[i] - *slope * x[i];
    *icept = median_inplace(s_ts_buf, n);
    return isfinite(*slope) && isfinite(*icept);
}

/* Least-squares slope of y over x; false if the spread of x is too small. */
static bool ls_slope(const float *x, const float *y, int n, float *slope, float *mx, float *my)
{
    if (n < 2) return false;
    float sx = 0, sy = 0;
    for (int k = 0; k < n; k++) { sx += x[k]; sy += y[k]; }
    *mx = sx / (float)n;
    *my = sy / (float)n;
    float sxx = 0, sxy = 0;
    for (int k = 0; k < n; k++) {
        float dx = x[k] - *mx;
        sxx += dx * dx;
        sxy += dx * (y[k] - *my);
    }
    if (!(sxx > 1e-12f)) return false;
    *slope = sxy / sxx;
    return isfinite(*slope);
}

static void cal_done_open(void);
static void verify_touch(void);

static void cal_finish(void)
{
    const float span = (s_pend.pos_touch - s_pend.pos_open) * CLOSE_DIR;
    const float closed_from_open = (s_pend.pos_closed - s_pend.pos_open) * CLOSE_DIR;
    if (!(span > CAL_SPAN_MIN_RAD && span < CAL_SPAN_MAX_RAD) || !(closed_from_open > 0.0f)) {
        ERROR_OUT(GRIPPER_FAULT_SPAN_INVALID, "open %.3f, closed %.3f, touch %.3f rad (stroke %.3f)",
                  s_pend.pos_open, s_pend.pos_closed, s_pend.pos_touch, span);
        return;
    }
    if (have_prev_span()) {
        float ds = fabsf(span - s_nvs.span) / s_nvs.span;
        if (ds > CAL_SPAN_CHANGE_FRAC) {
            /* Something between the pads, or the mechanism changed. Refuse
             * once and drop the reference, so a deliberate second run is
             * accepted. */
            ERROR_OUT(GRIPPER_FAULT_SPAN_CHANGED,
                      "stroke %.3f rad differs from the saved %.3f rad by %.0f %% - is something "
                      "between the pads? Run GRIP_CALIBRATE again to accept the new stroke.",
                      span, s_nvs.span, ds * 100.0f);
            s_nvs.span = 0.0f;
            nvs_store();
            return;
        }
        if (s_nvs.k_force > 0.0f && fabsf(s_pend.k_force - s_nvs.k_force) / s_nvs.k_force > 0.5f) {
            ESP_LOGW(TAG, "k %.1f N/pu differs from the saved %.1f N/pu by more than half",
                     s_pend.k_force, s_nvs.k_force);
        }
    }

    s_pend.valid = true;
    s_pend.motor_epoch = s_in.motor_epoch;
    s_pend.default_force = s_cal.default_force;
    float ks = s_pend.pad_stiffness > 1.0f ? s_pend.pad_stiffness : 1e6f;
    s_pend.approach_speed = clampf(GRIP_CONTACT_OVERSHOOT_N / (ks * TAC_REACTION_S),
                                   GRIP_APPROACH_SPEED_MIN, GRIP_APPROACH_SPEED_MAX);
    s_cal = s_pend;

    s_nvs.span = span;
    s_nvs.k_force = s_cal.k_force;
    s_nvs.i_offset = s_cal.i_offset;
    s_nvs.i_move = s_cal.i_move;
    s_nvs.tau_per_pu = s_cal.tau_per_pu;
    s_nvs.pad_stiffness = s_cal.pad_stiffness;
    s_nvs.default_force = s_cal.default_force;
    s_nvs.close_dir = CLOSE_DIR;   /* proven by this calibration, and only by one */
    s_nvs.pos_valid = 1u;
    s_nvs.p_stop = s_p_stop;
    s_nvs.pos_open = s_cal.pos_open;
    s_nvs.pos_closed = s_cal.pos_closed;
    s_nvs.pos_touch = s_cal.pos_touch;
    s_nvs.approach_speed = s_cal.approach_speed;
    nvs_store();

    ESP_LOGI(TAG, "calibrated: open %.3f, closed %.3f, touch %.3f rad (stroke %.3f rad); "
                  "k %.1f N/pu, offset %.3f pu, moving current %.3f pu, %.2f N*m/pu, "
                  "pads %.0f N/rad -> grip approach %.3f rad/s%s; saved to flash",
             s_cal.pos_open, s_cal.pos_closed, s_cal.pos_touch, span,
             s_cal.k_force, s_cal.i_offset, s_cal.i_move, s_cal.tau_per_pu,
             s_cal.pad_stiffness, s_cal.approach_speed, s_cal.k_estimated ? " (k estimated)" : "");
    cal_done_open();
    if (s_cal_note != GRIPPER_FAULT_NONE) {
        s_fault = s_cal_note;            /* calibrated, but the host should see this */
    }
}

/* Home check: the pads touched (both confirmed). Restore the stored
 * calibration if they meet where stored (a whole encoder turn apart is the
 * same place), else discard the stored positions. */
static void verify_touch(void)
{
    const float d = s_pend.pos_touch - s_nvs.pos_touch;
    const float two_pi = 6.2831853f;
    const float d_turn = d - two_pi * roundf(d / two_pi);
    if (fabsf(d_turn) > CAL_VERIFY_TOL_RAD) {
        /* The stored calibration is kept: it is used only after a home check
         * succeeds, and a contact short of the stored touch is most often an
         * object left between the pads (bench 2026-10-23: a sponge, 1.38 rad
         * short, discarded the calibration and forced a full one). */
        RETREAT_OUT(GRIPPER_FAULT_FRAME_CHANGED, GRIPPER_ERROR, -CLOSE_DIR,
                    "pads touch at %.3f rad, stored %.3f rad (%.3f rad apart, limit %.2f). Something between "
                    "the pads? Remove it and send GRIP_HOME again. If nothing is there, the position frame or "
                    "the mechanism changed: run GRIP_CALIBRATE. The stored calibration is kept until then.",
                    s_pend.pos_touch, s_nvs.pos_touch, d, CAL_VERIFY_TOL_RAD);
        return;
    }
    if (fabsf(d) > CAL_VERIFY_TOL_RAD) {
        ESP_LOGW(TAG, "pads touch %.3f rad from the stored position: a whole turn of the output encoder, "
                      "positions shifted accordingly", d);
    }
    /* Shift every stored position by the measured difference. */
    s_pend.pos_open = s_nvs.pos_open + d;
    s_pend.pos_closed = s_nvs.pos_closed + d;
    s_pend.pos_touch = s_nvs.pos_touch + d;
    s_pend.k_force = s_nvs.k_force;
    s_pend.i_offset = s_nvs.i_offset;
    s_pend.i_move = fmaxf(s_nvs.i_move, s_pend.i_move);
    s_pend.tau_per_pu = s_nvs.tau_per_pu;
    s_pend.pad_stiffness = s_nvs.pad_stiffness;
    s_pend.approach_speed = s_nvs.approach_speed;
    s_pend.default_force = s_cal.default_force;
    s_pend.valid = true;
    s_pend.motor_epoch = s_in.motor_epoch;
    s_have_touch = true;
    ESP_LOGI(TAG, "pads touch %.4f rad from the stored position: restoring the calibration", d);
    /* Back off to the closed position, as a full calibration ends. */
    s_backoff_tries = 1;
    s_backoff_target = s_pend.pos_closed;
    s_motion_dir = -CLOSE_DIR;
    s_arrived_ms = 0;
    s_sep_cnt[0] = s_sep_cnt[1] = 0;
    cal_set_phase(CAL_CLOSE_BACKOFF);
}

static void cal_done_open(void)
{
    s_fault = GRIPPER_FAULT_NONE;
    s_verify = false;

    /* Finish at the most open position a move may reach, ready to grasp. */
    s_move_target = pos_at_pct(MOVE_PCT_MIN);
    s_move_speed = MOVE_SPEED;
    s_move_timeout_ms = MOVE_TIMEOUT_MS;
    move_current_init(false);
    s_move_f_start = tac_max_ok();
    s_motion_dir = -CLOSE_DIR;
    s_hold = false;
    set_state(GRIPPER_MOVING);
}

/* The squeeze could not measure k (friction holding or releasing the force
 * in steps, or a fit that disagrees with the motor): finish the calibration
 * with k from the motor itself, (N*m per pu) / GRIP_LEVER_NOMINAL_M. The
 * open and closed positions do not depend on k. @p suspicious marks a fit
 * that was clean but implies a lever far from the nominal one - what both
 * pads under-reading by the same factor looks like - and leaves it as the
 * last fault, so the host sees it. */
static void squeeze_fallback(bool suspicious, const char *why)
{
    const float tpu = tau_per_pu_known();
    if (!(tpu > 0.0f)) {
        RETREAT_OUT(GRIPPER_FAULT_GAIN_INVALID, GRIPPER_ERROR, -CLOSE_DIR,
                    "%s, and no N*m per pu to estimate k from", why);
        return;
    }
    s_pend.k_force = tpu / GRIP_LEVER_NOMINAL_M;
    s_pend.i_offset = s_seek.tau_free_valid ? s_seek.tau_free / tpu : 0.0f;
    if (!(s_pend.pad_stiffness > 1.0f)) s_pend.pad_stiffness = 300.0f;
    s_pend.k_estimated = true;
    if (suspicious) {
        log_fault(false, GRIPPER_FAULT_GAIN_INVALID, "%s. Using k = %.0f N/pu from the motor (%.2f N*m/pu over "
                  "%.1f mm). Check the pad readings with a known weight", why, s_pend.k_force, tpu,
                  GRIP_LEVER_NOMINAL_M * 1000.0f);
        s_cal_note = GRIPPER_FAULT_GAIN_INVALID;
    } else {
        ESP_LOGW(TAG, "k not measured (%s); using k = %.0f N/pu from the motor (%.2f N*m/pu over %.1f mm)",
                 why, s_pend.k_force, tpu, GRIP_LEVER_NOMINAL_M * 1000.0f);
    }
    s_backoff_tries = 1;
    s_backoff_target = s_pend.pos_touch - CLOSE_DIR * CAL_CLOSE_BACKOFF_RAD;
    s_motion_dir = -CLOSE_DIR;
    s_arrived_ms = 0;
    s_sep_cnt[0] = s_sep_cnt[1] = 0;
    cal_set_phase(CAL_CLOSE_BACKOFF);
}

static void cal_tick(void)
{
    switch (s_cal_phase) {
    case CAL_BASELINE:
        /* Motor still off: whatever the pads read now is the reference for
         * the direction check while opening. */
        if (!tac_both_ok()) {
            if (s_phase_ms > CAL_SAMPLE_TIMEOUT_MS) {
                ERROR_OUT(GRIPPER_FAULT_TACTILE_UNAVAILABLE, "no tactile baseline");
            }
            return;
        }
        for (int i = 0; i < GRIPPER_TACTILE_COUNT; i++) {
            if (s_tac_new[i] && s_base_n[i] < 3) {
                s_base_sum[i] += s_in.fz[i];
                s_base_n[i]++;
            }
        }
        if (s_base_n[0] >= 3 && s_base_n[1] >= 3) {
            for (int i = 0; i < GRIPPER_TACTILE_COUNT; i++) {
                s_baseline[i] = s_base_sum[i] / (float)s_base_n[i];
            }
            if (s_verify) {
                /* Home check: tare here with the motor off, then find the pad
                 * touch. The pads must be free now. */
                if (fabsf(s_baseline[0]) > TAC_BASELINE_MAX_N || fabsf(s_baseline[1]) > TAC_BASELINE_MAX_N) {
                    ERROR_OUT(GRIPPER_FAULT_BASELINE, "home check needs free pads: ring %.2f N, little %.2f N. "
                              "Open the jaw a little by hand, then send GRIP_HOME again",
                              s_baseline[0], s_baseline[1]);
                    return;
                }
                s_tare_requested = false;
                s_tare_done = false;
                cal_set_phase(CAL_TARE);
                return;
            }
            /* The opening end is found by the opening torque, at the slow
             * speed the threshold was measured at (and until a saved stroke
             * proves CLOSE_DIR, "opening" may in fact be closing the pads). */
            ESP_LOGI(TAG, "baseline ring %.2f N, little %.2f N; opening at %.2f rad/s until the opening "
                          "torque reaches %.2f N*m", s_baseline[0], s_baseline[1], CAL_TOUCH_SPEED, CAL_OPEN_TQ_NM);
            s_open_retry = 0;
            seek_init(&s_seek, -CLOSE_DIR, CAL_TOUCH_SPEED, CAL_CUR_START_PU, cal_cur_max());
            s_seek.margin = CAL_OPEN_MARGIN;
            s_seek.fast_halt = true;
            s_seek.open_tq_thr = CAL_OPEN_TQ_NM;
            s_speed_checked = false;
            cal_set_phase(CAL_OPEN_SEEK);
        } else if (s_phase_ms > CAL_SAMPLE_TIMEOUT_MS) {
            ERROR_OUT(GRIPPER_FAULT_TACTILE_UNAVAILABLE, "too few tactile samples for the baseline");
        }
        return;

    case CAL_OPEN_SEEK: {
        if (!cal_tactile_guard()) return;
        if (any_rel_above(TAC_RISE_ABORT_N)) {
            RETREAT_OUT(GRIPPER_FAULT_FORCE_WHILE_OPENING, GRIPPER_ERROR, -s_seek.dir,
                        "pads %.2f / %.2f N above baseline while opening - is CLOSE_DIR reversed?",
                        rel(0), rel(1));
            return;
        }
        seek_result_t r = seek_step(&s_seek);
        if (r == SEEK_DRIVE_FAILED) {
            ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused while opening");
        } else if (r == SEEK_TRAVEL_LIMIT) {
            ERROR_OUT(GRIPPER_FAULT_TRAVEL_LIMIT, "opened %.2f rad without a stop", CAL_MAX_TRAVEL_RAD);
        } else if (r == SEEK_TOO_SLOW) {
            float avg = s_seek.t_ms > 0
                ? (s_in.pos - s_seek.start_pos) * s_seek.dir * 1000.0f / (float)s_seek.t_ms : 0.0f;
            ERROR_OUT(GRIPPER_FAULT_SPEED_MISMATCH,
                      "jaw moved %.4f rad in %.1f s (%.4f rad/s) for a commanded %.2f rad/s, even at "
                      "%.3f pu (torque %.3f N*m). Either friction exceeds the current ceiling, or "
                      "the drive reads vel_limit as rotor speed. Measure on the bench before "
                      "changing VEL_FIELD_SCALE: a wrong scale multiplies every speed limit.",
                      (s_in.pos - s_seek.start_pos) * s_seek.dir, (float)s_seek.t_ms / 1000.0f, avg,
                      s_seek.speed, s_seek.cur, s_in.torque);
        } else if (r == SEEK_NO_MOTION) {
            if (s_open_retry == 0) {
                /* Possibly already at the open stop: close a little and retry. */
                ESP_LOGW(TAG, "jaw does not open at %.3f pu; closing %.2f rad and retrying",
                         s_seek.cur, CAL_RETRY_BACKOFF_RAD);
                s_open_retry = 1;
                s_retry_dist = CAL_RETRY_BACKOFF_RAD;
                seek_init(&s_seek, CLOSE_DIR, CAL_TOUCH_SPEED, CAL_CUR_START_PU, cal_cur_max());
                cal_set_phase(CAL_OPEN_RETRY);
            } else {
                ERROR_OUT(GRIPPER_FAULT_NO_MOTION, "jaw does not open at %.3f pu", s_seek.cur);
            }
        } else if (r == SEEK_STOPPED) {
            if (any_rel_above(TAC_TOUCH_N)) {
                RETREAT_OUT(GRIPPER_FAULT_FORCE_WHILE_OPENING, GRIPPER_ERROR, -s_seek.dir,
                            "stopped with pad force %.2f / %.2f N - is CLOSE_DIR reversed?",
                            rel(0), rel(1));
                return;
            }
            s_p_stop = s_in.pos;
            s_have_stop = true;
            s_pend.i_move = s_seek.cur_moving;
            s_tau_sum = 0.0f;
            s_tau_n = 0;
            if (s_seek.tq_stop && (s_in.pos - s_seek.start_pos) * s_seek.dir < CAL_OPEN_TQ_MIN_TRAVEL) {
                /* The opening torque was above the limit almost from the start:
                 * the jaw began at or beyond the opening limit (e.g. left wide
                 * open). Taking this as the limit would keep that opening, so
                 * close well inside the stroke and open again, once. */
                if (s_open_retry == 0) {
                    ESP_LOGW(TAG, "opening torque %.3f N*m already at the limit after %.3f rad: the jaw started at "
                                  "or past the opening limit; closing %.1f rad and opening again",
                             s_seek.tau_slow, (s_in.pos - s_seek.start_pos) * s_seek.dir, CAL_OPEN_TQ_RETRY_RAD);
                    s_open_retry = 1;
                    s_retry_dist = CAL_OPEN_TQ_RETRY_RAD;
                    s_have_stop = false;
                    seek_init(&s_seek, CLOSE_DIR, CAL_TOUCH_SPEED, CAL_CUR_START_PU, cal_cur_max());
                    /* closing from the soft end: the spring that helps at first
                     * fades, so the current must follow the rising friction */
                    if (tau_per_pu_known() > 0.0f) s_seek.trim_extra = CAL_CLOSE_TRIM_NM / tau_per_pu_known();
                    cal_set_phase(CAL_OPEN_RETRY);
                } else {
                    ERROR_OUT(GRIPPER_FAULT_NO_MOTION, "opening torque at the limit again right after the retry "
                              "(%.3f N*m): close the jaw by hand to mid-stroke and calibrate again", s_seek.tau_slow);
                }
                return;
            }
            if (s_seek.tq_stop) {
                /* Opening limit by torque: the jaw was still moving, so there
                 * is no stall to measure N*m per pu; take the motor's own
                 * Kt x Imax (bench: 19.27 from the registers, 19.36 measured
                 * at a stall, 0.5 % apart). */
                s_pend.tau_per_pu = s_in.motor_tau_per_pu_est > 1.0f ? s_in.motor_tau_per_pu_est : 0.0f;
                ESP_LOGI(TAG, "opening limit at %.3f rad: opening torque %.3f N*m (0.5 s mean) reached %.2f N*m; "
                              "%.2f N*m/pu from the motor's registers",
                         s_p_stop, s_seek.tau_slow, CAL_OPEN_TQ_NM, s_pend.tau_per_pu);
                s_backoff_target = s_p_stop + CLOSE_DIR * CAL_OPEN_BACKOFF_RAD;
                s_motion_dir = CLOSE_DIR;
                cal_set_phase(CAL_OPEN_BACKOFF);
                return;
            }
            ESP_LOGI(TAG, "open stop at %.3f rad (%s: torque %.3f N*m, free %.3f N*m)",
                     s_p_stop, s_seek.fast_stop ? "torque rise" : "stall only",
                     s_in.torque, s_seek.tau_free);
            cal_set_phase(CAL_OPEN_DWELL);
        } else {
            /* Once, shortly after the jaw is moving: is it moving at about the
             * commanded speed? A tenfold shortfall means the vel_limit field
             * is rotor side; anything slow means the current ceiling cannot
             * overcome friction. Either way calibration cannot be trusted. */
            if (s_seek.established && !s_speed_checked && s_seek.est_ms >= CAL_SPEED_CHECK_MS) {
                s_speed_checked = true;
                float d = 0.0f;
                /* Only on steady motion: a window that already contains the
                 * stop would read low and blame the speed field. */
                if (hist_delta(WIN_MAX_MS, &d) && is_creeping(s_seek.dir) && !is_stalled(s_seek.speed)) {
                    float v = d * s_seek.dir * 1000.0f / (float)WIN_MAX_MS;
                    if (v < CAL_SPEED_MIN_FRAC * s_seek.speed) {
                        ERROR_OUT(GRIPPER_FAULT_SPEED_MISMATCH,
                                  "jaw moves at %.3f rad/s for a commanded %.2f rad/s at %.3f pu "
                                  "(torque %.3f N*m). Either friction exceeds the current ceiling, "
                                  "or the drive reads vel_limit as rotor speed. Measure on the bench "
                                  "before changing VEL_FIELD_SCALE.",
                                  v, s_seek.speed, s_seek.cur, s_in.torque);
                        return;
                    }
                    ESP_LOGI(TAG, "speed check: %.3f rad/s for a commanded %.2f rad/s", v, s_seek.speed);
                }
            }
            if (s_phase_ms > CAL_PHASE_TIMEOUT_MS) {
                ERROR_OUT(GRIPPER_FAULT_TIMEOUT, "open seek");
            }
        }
        return;
    }

    case CAL_OPEN_DWELL:
        if (!cal_tactile_guard()) return;
        if (any_rel_above(TAC_RISE_ABORT_N)) {
            RETREAT_OUT(GRIPPER_FAULT_FORCE_WHILE_OPENING, GRIPPER_ERROR, -s_seek.dir,
                        "pad force at the open stop - is CLOSE_DIR reversed?");
            return;
        }
        if (!drive(s_in.pos + s_seek.dir * SEEK_LEAD_RAD, vel_field(CAL_SPEED), s_seek.cur)) {
            ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused at the open stop");
            return;
        }
        if ((s_in.pos - s_p_stop) * s_seek.dir > 0.02f) {
            ESP_LOGW(TAG, "jaw moved on past %.3f rad: not the stop, seeking again", s_p_stop);
            s_have_stop = false;
            s_seek.stall_ms = 0;
            s_seek.rise_ms = 0;
            cal_set_phase(CAL_OPEN_SEEK);
            return;
        }
        s_tau_sum += fabsf(s_in.torque);
        s_tau_n++;
        if (s_phase_ms >= CAL_OPEN_DWELL_MS && s_tau_n > 0) {
            float tau_stall = s_tau_sum / (float)s_tau_n;
            s_pend.tau_per_pu = s_seek.cur > 0.0f ? tau_stall / s_seek.cur : 0.0f;
            ESP_LOGI(TAG, "stall torque %.3f N*m at %.3f pu -> %.2f N*m/pu",
                     tau_stall, s_seek.cur, s_pend.tau_per_pu);
            s_backoff_target = s_p_stop + CLOSE_DIR * CAL_OPEN_BACKOFF_RAD;
            s_motion_dir = CLOSE_DIR;
            cal_set_phase(CAL_OPEN_BACKOFF);
        }
        return;

    case CAL_OPEN_RETRY: {
        if (!cal_tactile_guard()) return;
        if (any_rel_above(TAC_RISE_ABORT_N)) {
            RETREAT_OUT(GRIPPER_FAULT_NO_MOTION, GRIPPER_ERROR, -s_seek.dir,
                        "could not open, and closing reached contact (%.2f / %.2f N)",
                        rel(0), rel(1));
            return;
        }
        seek_result_t r = seek_step(&s_seek);
        float traveled = (s_in.pos - s_seek.start_pos) * s_seek.dir;
        if (r == SEEK_DRIVE_FAILED) {
            ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused while retrying");
        } else if (r == SEEK_NO_MOTION || r == SEEK_TOO_SLOW) {
            ERROR_OUT(GRIPPER_FAULT_NO_MOTION, "jaw moves in neither direction at %.3f pu (if it turns "
                      "very slowly, check on the bench whether vel_limit is read as rotor speed)", s_seek.cur);
        } else if (r == SEEK_TRAVEL_LIMIT) {
            ERROR_OUT(GRIPPER_FAULT_TRAVEL_LIMIT, "retry travel");
        } else if (traveled >= s_retry_dist || r == SEEK_STOPPED) {
            float c = s_seek.established ? s_seek.cur_moving : CAL_CUR_START_PU;
            seek_init(&s_seek, -CLOSE_DIR, CAL_TOUCH_SPEED, c, cal_cur_max());
            s_seek.margin = CAL_OPEN_MARGIN;
            s_seek.fast_halt = true;
            s_seek.open_tq_thr = CAL_OPEN_TQ_NM;
            s_speed_checked = false;
            cal_set_phase(CAL_OPEN_SEEK);
        } else if (s_phase_ms > CAL_PHASE_TIMEOUT_MS) {
            ERROR_OUT(GRIPPER_FAULT_TIMEOUT, "open retry");
        }
        return;
    }

    case CAL_OPEN_BACKOFF: {
        if (!cal_tactile_guard()) return;
        if (any_rel_above(TAC_RISE_ABORT_N)) {
            RETREAT_OUT(GRIPPER_FAULT_FORCE_WHILE_OPENING, GRIPPER_ERROR, -CLOSE_DIR,
                        "pad force while backing off from the open stop");
            return;
        }
        int st = cal_move_step(s_backoff_target, CAL_SPEED);
        if (st == 1) {
            s_pend.pos_open = s_backoff_target;
            ESP_LOGI(TAG, "open position %.3f rad", s_pend.pos_open);
            s_tare_requested = false;
            s_tare_done = false;
            cal_set_phase(CAL_TARE);
        }
        return;
    }

    case CAL_TARE:
        /* Hold the open position while the pads are free and tare them (the
         * home check tares where it starts, motor off). The tactile stream
         * pauses for about a second during the tare. */
        if (!s_verify &&
            !drive(ramp_toward(s_pend.pos_open, CAL_SPEED), vel_field(CAL_SPEED), creep_cap())) {
            ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused during the tare");
            return;
        }
        if (!s_tare_requested) {
            s_tare_count0 = s_in.tare_count;
            if (!haptic_uart_calibrate()) {
                ERROR_OUT(GRIPPER_FAULT_TARE, "tare request rejected");
                return;
            }
            s_tare_requested = true;
            ESP_LOGI(TAG, "tactile tare requested");
        } else if (!s_tare_done) {
            if (s_in.tare_count != s_tare_count0) {
                if (!s_in.tare_ok) {
                    ERROR_OUT(GRIPPER_FAULT_TARE, "the tactile board did not acknowledge the tare");
                    return;
                }
                s_tare_done = true;
                s_phase_ms = 0;
                for (int i = 0; i < GRIPPER_TACTILE_COUNT; i++) {
                    s_base_sum[i] = 0.0f;
                    s_base_n[i] = 0;
                }
            } else if (s_phase_ms > CAL_TARE_TIMEOUT_MS) {
                ERROR_OUT(GRIPPER_FAULT_TARE, "tare did not finish in %d ms", CAL_TARE_TIMEOUT_MS);
            }
        } else {
            if (!tac_both_ok()) {
                if (s_phase_ms > CAL_SAMPLE_TIMEOUT_MS) {
                    ERROR_OUT(GRIPPER_FAULT_TACTILE_UNAVAILABLE, "no tactile samples after the tare");
                }
                return;
            }
            for (int i = 0; i < GRIPPER_TACTILE_COUNT; i++) {
                if (s_tac_new[i] && s_base_n[i] < CAL_TARE_SAMPLES) {
                    s_base_sum[i] += s_in.fz[i];
                    s_base_n[i]++;
                }
            }
            if (s_base_n[0] >= CAL_TARE_SAMPLES && s_base_n[1] >= CAL_TARE_SAMPLES) {
                float b0 = s_base_sum[0] / (float)s_base_n[0];
                float b1 = s_base_sum[1] / (float)s_base_n[1];
                if (fabsf(b0) > TAC_BASELINE_MAX_N || fabsf(b1) > TAC_BASELINE_MAX_N) {
                    ERROR_OUT(GRIPPER_FAULT_BASELINE, "after the tare ring %.2f N, little %.2f N", b0, b1);
                    return;
                }
                s_baseline[0] = b0;
                s_baseline[1] = b1;
                s_touch_cnt[0] = s_touch_cnt[1] = 0;
                s_touching = false;
                ESP_LOGI(TAG, "tared (ring %.2f N, little %.2f N); closing until the pads touch%s", b0, b1,
                         s_verify ? " (home check: compared with the stored touch position)"
                         : have_prev_span() ? "" : " (no saved stroke: whole stroke at the slow speed)");
                seek_init(&s_seek, CLOSE_DIR, CAL_TOUCH_SPEED, 0.8f * i_move_known(), cal_cur_max());
                s_seek.trim_extra = tau_per_pu_known() > 0.0f ? CAL_CLOSE_TRIM_NM / tau_per_pu_known() : 0.002f;
                cal_set_phase(CAL_CLOSE_SEEK);
            } else if (s_phase_ms > CAL_SAMPLE_TIMEOUT_MS) {
                ERROR_OUT(GRIPPER_FAULT_TACTILE_UNAVAILABLE, "too few tactile samples after the tare");
            }
        }
        return;

    case CAL_CLOSE_SEEK: {
        if (!cal_tactile_guard()) return;
        const float r0 = rel(0), r1 = rel(1);
        if (r0 >= CAL_CONTACT_ABORT_N || r1 >= CAL_CONTACT_ABORT_N) {
            RETREAT_OUT(GRIPPER_FAULT_CONTACT_TOO_HARD, GRIPPER_ERROR, -CLOSE_DIR,
                        "%.2f / %.2f N while closing", r0, r1);
            return;
        }
        for (int i = 0; i < GRIPPER_TACTILE_COUNT; i++) {
            if (s_tac_new[i]) {
                s_touch_cnt[i] = rel(i) >= TAC_TOUCH_N ? s_touch_cnt[i] + 1 : 0;
            }
        }
        if (!s_touching && (s_touch_cnt[0] > 0 || s_touch_cnt[1] > 0)) {
            /* First touch on either pad: stop advancing at once and hold the
             * position while both pads confirm. Pushing on during the
             * confirmation would add the tactile delay's worth of force. */
            s_touching = true;
            s_touch_first = s_in.pos;
            s_touch_ms = 0;
        }
        const bool both = s_touch_cnt[0] >= TAC_TOUCH_SAMPLES && s_touch_cnt[1] >= TAC_TOUCH_SAMPLES;
        const bool one = s_touch_cnt[0] >= TAC_TOUCH_SAMPLES || s_touch_cnt[1] >= TAC_TOUCH_SAMPLES;
        if (both || (one && s_touching && s_touch_ms >= CAL_TOUCH_CONFIRM_MS)) {
            /* The position of the FIRST touch sample: by the time the pads
             * confirm, they are already compressed a little. One pad is
             * enough and is not a fault: the two jaws are not exactly
             * parallel, so a light pad-on-pad contact can fall outside one
             * pad's sensing area (bench 2026-10-22: ring 1.03 N while little
             * read 0.00 N; pressed by hand, ring 4.10 N and little 1.90 N).
             * The larger reading is the force, here and in the grip. */
            s_pend.pos_touch = s_touching ? s_touch_first : s_in.pos;
            s_have_touch = true;
            if (s_seek.established && s_seek.cur_moving > s_pend.i_move) {
                s_pend.i_move = s_seek.cur_moving;
            }
            ESP_LOGI(TAG, "pads touch at %.3f rad (ring %.2f N, little %.2f N%s)", s_pend.pos_touch, r0, r1,
                     both ? "" : "; one pad reads it, the larger reading is used");
            if (s_verify) {
                verify_touch();
                return;
            }
            s_sq_i0 = 0.5f * s_seek.cur;
            s_sq_cur = s_sq_i0;
            s_sq_ramping = false;
            s_sq_n = 0;
            cal_set_phase(CAL_SQUEEZE);
            return;
        }
        if (s_touching) {
            s_touch_ms += s_dt;
            float hold_cur = s_seek.established ? s_seek.cur_moving : s_seek.cur;
            if (!drive(s_touch_first, vel_field(CAL_TOUCH_SPEED), hold_cur)) {
                ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused while closing");
                return;
            }
            if (s_touch_ms >= CAL_TOUCH_CONFIRM_MS && r0 < TAC_TOUCH_N && r1 < TAC_TOUCH_N) {
                ESP_LOGW(TAG, "touch not confirmed (%.2f / %.2f N): closing on", r0, r1);
                s_touching = false;
                s_seek.ramp = s_in.pos;
            }
            return;
        }
        /* Fast through the free stroke when the last calibration says where
         * the pads meet; slow over the last CAL_SLOW_ZONE_RAD (and the whole
         * way when there is no saved stroke). */
        if (s_verify) {
            /* The pads meet where stored, or a whole encoder turn away (the
             * drive starts from the single-turn angle; the stroke is longer
             * than a turn). Slow near every candidate and once past the stored
             * one; CAL_SPEED in between. */
            float remaining = (s_nvs.pos_touch - s_in.pos) * CLOSE_DIR;
            float wrapped = remaining - 6.2831853f * roundf(remaining / 6.2831853f);
            bool near = fabsf(wrapped) < CAL_SLOW_ZONE_RAD || remaining < CAL_SLOW_ZONE_RAD;
            s_seek.speed = near ? CAL_TOUCH_SPEED : CAL_SPEED;
        } else {
            float traveled = (s_in.pos - s_pend.pos_open) * CLOSE_DIR;
            bool fast = have_prev_span() && traveled < s_nvs.span - CAL_SLOW_ZONE_RAD;
            s_seek.speed = fast ? CAL_SPEED : CAL_TOUCH_SPEED;
        }
        seek_result_t r = seek_step(&s_seek);
        if (r == SEEK_DRIVE_FAILED) {
            ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused while closing");
        } else if (r == SEEK_STOPPED) {
            if (r0 >= TAC_TOUCH_N || r1 >= TAC_TOUCH_N) {
                /* stopped on the pads before the touch count completed: hold
                 * here and let the touch confirmation finish */
                s_touching = true;
                s_touch_first = s_in.pos;
                s_touch_ms = 0;
            } else {
                RETREAT_OUT(GRIPPER_FAULT_STALL_NO_CONTACT, GRIPPER_ERROR, -CLOSE_DIR,
                            "jaw stopped at %.3f rad but neither pad reads contact", s_in.pos);
            }
        } else if (r == SEEK_NO_MOTION || r == SEEK_TOO_SLOW) {
            ERROR_OUT(GRIPPER_FAULT_NO_MOTION, "jaw does not close at %.3f pu", s_seek.cur);
        } else if (r == SEEK_TRAVEL_LIMIT) {
            RETREAT_OUT(GRIPPER_FAULT_TRAVEL_LIMIT, GRIPPER_ERROR, -CLOSE_DIR,
                        "closed %.2f rad without pad contact", CAL_MAX_TRAVEL_RAD);
        } else if (s_phase_ms > CAL_PHASE_TIMEOUT_MS) {
            RETREAT_OUT(GRIPPER_FAULT_TIMEOUT, GRIPPER_ERROR, -CLOSE_DIR, "close seek");
        }
        return;
    }

    case CAL_SQUEEZE: {
        /* Press pad on pad with a stepped current limit and fit the force
         * against the current: k = dF/dI as the median of the pairwise slopes
         * of the points in the window, accepted only if they lie on a line. The same
         * points give the pad stiffness dF/dx, which sets the grip approach
         * speed. */
        if (!cal_tactile_guard()) return;
        const float r0 = rel(0), r1 = rel(1);
        if (r0 >= CAL_CONTACT_ABORT_N || r1 >= CAL_CONTACT_ABORT_N) {
            /* stop pressing at once; the closing back-off unloads the pads */
            char why[96];
            snprintf(why, sizeof(why), "pads reached %.1f / %.1f N in a step (largest rise %.2f N)", r0, r1, s_sq_step);
            squeeze_fallback(false, why);
            return;
        }
        /* the larger pad, as in the grip's force loop */
        const float fpad = fmaxf(r0, r1);
        const float target = s_pend.pos_touch + CLOSE_DIR * SEEK_LEAD_RAD;
        if (!s_sq_ramping) {
            /* Settle just behind the touch position: friction would
             * otherwise hold whatever the touch overshoot left on the pads,
             * and the measurement must start from a nearly unloaded pad.
             * Backing off needs more than friction: use the closing current
             * (>= 1.5 x sliding), not the lower start current of the steps. */
            if (!drive(s_pend.pos_touch - CLOSE_DIR * CAL_SQUEEZE_UNLOAD_RAD,
                       vel_field(CAL_TOUCH_SPEED), fmaxf(s_sq_i0, s_seek.cur))) {
                ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused while squeezing");
                return;
            }
            if (s_phase_ms >= CAL_SQUEEZE_SETTLE_MS) {
                float f0 = fpad;
                s_sq_level_a = fmaxf(CAL_SQUEEZE_LO_N, f0 + 0.2f);
                s_sq_level_b = s_sq_level_a + CAL_SQUEEZE_SPAN_N;
                if (s_sq_level_b > CAL_CONTACT_ABORT_N - 0.5f) {
                    char why[64];
                    snprintf(why, sizeof(why), "pads stay at %.2f N after backing off", f0);
                    squeeze_fallback(false, why);
                    return;
                }
                s_sq_ramping = true;
                const float tpu = s_pend.tau_per_pu;
                /* Whole LSBs of the frame's current field, so the current
                 * recorded with each point is exactly what the drive gets. */
                s_sq_di_lsb = tpu > 0.0f ? (int32_t)lroundf(CAL_SQUEEZE_TQ_STEP / tpu / CUR_LSB_PU) : 3;
                if (s_sq_di_lsb < 1) s_sq_di_lsb = 1;
                if (s_sq_di_lsb > 20) s_sq_di_lsb = 20;
                s_sq_di = (float)s_sq_di_lsb * CUR_LSB_PU;
                s_sq_lsb = (int32_t)floorf(s_sq_i0 / CUR_LSB_PU + 1e-3f);
                s_sq_cur = (float)s_sq_lsb * CUR_LSB_PU;
                s_sq_cur_max = tpu > 0.0f ? fminf(CAL_SQUEEZE_CUR_MAX_PU, s_sq_i0 + CAL_SQUEEZE_TQ_SPAN_MAX / tpu)
                                          : CAL_SQUEEZE_CUR_MAX_PU;
                s_sq_dwell_ms = 0;
                s_sq_acc_f = s_sq_acc_p = 0.0f;
                s_sq_acc_n = 0;
                s_phase_ms = 0;
                s_sq_f_prev = -1.0f;
                s_sq_step = 0.0f;
                s_sq_step_i = 0.0f;
                ESP_LOGI(TAG, "squeeze from %.2f N: fitting between %.2f N and %.2f N, current steps of %.5f pu "
                              "held %d ms each, ceiling %.4f pu",
                         f0, s_sq_level_a, s_sq_level_b, s_sq_di, CAL_SQUEEZE_DWELL_MS, s_sq_cur_max);
            }
            return;
        }
        /* Step the current limit and hold each step until the jaw is at rest,
         * then record one point (current, mean force). Each point is a static
         * balance, so neither the speed limit (which caps how fast the pads
         * can be compressed) nor the tactile delay biases the slope. */
        if (!drive(target, vel_field(CAL_TOUCH_SPEED), s_sq_cur)) {
            ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused while squeezing");
            return;
        }
        s_sq_dwell_ms += s_dt;
        if (s_sq_dwell_ms + CAL_SQUEEZE_AVG_MS > CAL_SQUEEZE_DWELL_MS && (s_tac_new[0] || s_tac_new[1])) {
            s_sq_acc_f += fpad;
            s_sq_acc_p += s_in.pos * CLOSE_DIR;
            s_sq_acc_n++;
        }
        if (s_sq_dwell_ms >= CAL_SQUEEZE_DWELL_MS) {
            const bool at_rest = is_halted() && s_sq_acc_n >= 2;
            if (!at_rest && s_sq_dwell_ms < CAL_SQUEEZE_MAX_DWELL_MS) {
                /* still settling: average a fresh window */
                s_sq_dwell_ms = CAL_SQUEEZE_DWELL_MS - CAL_SQUEEZE_AVG_MS;
                s_sq_acc_f = s_sq_acc_p = 0.0f;
                s_sq_acc_n = 0;
            } else {
                float f = at_rest ? s_sq_acc_f / (float)s_sq_acc_n : fpad;
                if (at_rest) {
                    if (s_sq_f_prev >= 0.0f && f - s_sq_f_prev > s_sq_step) {
                        s_sq_step = f - s_sq_f_prev;
                        s_sq_step_i = s_sq_cur;
                    }
                    s_sq_f_prev = f;
                    ESP_LOGI(TAG, "squeeze point: %.5f pu, %.2f N (ring %.2f, little %.2f), %.4f rad%s",
                             s_sq_cur, f, r0, r1, s_sq_acc_p / (float)s_sq_acc_n,
                             f >= s_sq_level_a ? "" : " (below the window)");
                    if (f >= s_sq_level_a && s_sq_n < CAL_SQUEEZE_MAX_SAMPLES) {
                        s_sq_i[s_sq_n] = s_sq_cur;
                        s_sq_f[s_sq_n] = f;
                        s_sq_p[s_sq_n] = s_sq_acc_p / (float)s_sq_acc_n;
                        s_sq_n++;
                    }
                }
                if (at_rest && ((f >= s_sq_level_b && s_sq_n >= CAL_SQUEEZE_FIT_POINTS) ||
                                f >= CAL_CONTACT_ABORT_N - 0.5f || s_sq_n >= CAL_SQUEEZE_MAX_SAMPLES)) {
                    /* The jaw closes the settle gap and meets the pads with
                     * some speed; friction then holds that overshoot while the
                     * first steps raise the current without changing the force.
                     * Drop that flat start (keep its last point): the force
                     * follows the current only from there on. */
                    /* "Flat" = rising less than half the median rise per
                     * step (the force a step normally adds). */
                    int lead = 0;
                    if (s_sq_n >= 3) {
                        for (int j = 0; j < s_sq_n - 1; j++) s_ts_buf[j] = s_sq_f[j + 1] - s_sq_f[j];
                        const float half_rise = 0.5f * median_inplace(s_ts_buf, s_sq_n - 1);
                        while (s_sq_n - lead > 2 && s_sq_f[lead + 1] - s_sq_f[lead] < fmaxf(half_rise, 0.05f)) lead++;
                    }
                    if (lead > 0) {
                        for (int j = lead; j < s_sq_n; j++) {
                            s_sq_i[j - lead] = s_sq_i[j];
                            s_sq_f[j - lead] = s_sq_f[j];
                            s_sq_p[j - lead] = s_sq_p[j];
                        }
                        s_sq_n -= lead;
                        ESP_LOGI(TAG, "squeeze: %d flat point(s) at the start left out of the fit", lead);
                    }
                    float k = 0.0f, b = 0.0f, kp = 0.0f, mp = 0.0f, mf2 = 0.0f;
                    bool ok_k = s_sq_n >= CAL_SQUEEZE_MIN_SAMPLES && ts_line(s_sq_i, s_sq_f, s_sq_n, &k, &b);
                    float i_off = ok_k && k > 0.0f ? -b / k : NAN;
                    float lever = (ok_k && k > 0.0f && s_pend.tau_per_pu > 0.0f) ? s_pend.tau_per_pu / k : NAN;
                    bool lever_ok = !(s_pend.tau_per_pu > 0.0f) ||
                                    (lever >= CAL_LEVER_MIN_M && lever <= CAL_LEVER_MAX_M);
                    /* Fit quality: points must lie on a line (no stick-slip
                     * staircase) and span several steps of current. */
                    float rss = 0.0f, i_lo = s_sq_i[0], i_hi = s_sq_i[0];
                    for (int j = 0; j < s_sq_n; j++) {
                        float e = s_sq_f[j] - (k * s_sq_i[j] + b);
                        rss += e * e;
                        i_lo = fminf(i_lo, s_sq_i[j]);
                        i_hi = fmaxf(i_hi, s_sq_i[j]);
                    }
                    float rms = s_sq_n > 0 ? sqrtf(rss / (float)s_sq_n) : NAN;
                    bool fit_ok = ok_k && rms <= CAL_SQUEEZE_RESID_N && (i_hi - i_lo) >= 3.0f * s_sq_di;
                    if (!fit_ok || !(k >= CAL_K_MIN && k <= CAL_K_MAX) ||
                        !(i_off >= CAL_I_OFFSET_MIN && i_off <= cal_cur_max()) || !lever_ok) {
                        /* A clean fit whose lever is far from the nominal one is
                         * suspicious (pads reading wrong); anything else is the
                         * friction getting in the way. */
                        const bool suspicious = fit_ok && !lever_ok && k >= CAL_K_MIN && k <= CAL_K_MAX;
                        ESP_LOGW(TAG, "squeeze fit: %d points, k %.1f N/pu, offset %.3f pu, lever %.1f mm, residual "
                                      "%.2f N, largest rise between two steps %.2f N",
                                 s_sq_n, k, i_off, lever * 1000.0f, rms, s_sq_step);
                        char why[96];
                        if (suspicious) {
                            snprintf(why, sizeof(why), "measured k implies a %.1f mm lever, outside %.1f-%.1f mm",
                                     lever * 1000.0f, CAL_LEVER_MIN_M * 1000.0f, CAL_LEVER_MAX_M * 1000.0f);
                        } else {
                            snprintf(why, sizeof(why), "squeeze points do not fit a line (%d points, residual %.2f N)",
                                     s_sq_n, rms);
                        }
                        squeeze_fallback(suspicious, why);
                        return;
                    }
                    /* Pad stiffness from the same points; a position spread too
                     * small to fit means very stiff pads (slowest approach). */
                    if (ls_slope(s_sq_p, s_sq_f, s_sq_n, &kp, &mp, &mf2) && kp > 1.0f) {
                        s_pend.pad_stiffness = kp;
                    } else {
                        s_pend.pad_stiffness = 1e6f;
                    }
                    s_pend.k_force = k;
                    s_pend.i_offset = i_off;
                    s_pend.k_estimated = false;
                    ESP_LOGI(TAG, "k = %.1f N/pu, offset %.3f pu, pads %.0f N/rad (%d points, %.2f..%.2f N, "
                                  "fit residual %.2f N, lever %.1f mm)",
                             k, i_off, s_pend.pad_stiffness, s_sq_n, s_sq_f[0], f, rms, lever * 1000.0f);
                    s_backoff_tries = 1;
                    s_backoff_target = s_pend.pos_touch - CLOSE_DIR * CAL_CLOSE_BACKOFF_RAD;
                    s_motion_dir = -CLOSE_DIR;
                    s_arrived_ms = 0;
                    s_sep_cnt[0] = s_sep_cnt[1] = 0;
                    cal_set_phase(CAL_CLOSE_BACKOFF);
                    return;
                }
                /* Next step: coarse until the force nears the window. */
                s_sq_lsb += (f < s_sq_level_a - 0.5f ? 3 : 1) * s_sq_di_lsb;
                s_sq_cur = (float)s_sq_lsb * CUR_LSB_PU;
                s_sq_dwell_ms = 0;
                s_sq_acc_f = s_sq_acc_p = 0.0f;
                s_sq_acc_n = 0;
                if (s_sq_cur > s_sq_cur_max) {
                    char why[64];
                    snprintf(why, sizeof(why), "force reached only %.2f N at the %.4f pu ceiling", f, s_sq_cur_max);
                    squeeze_fallback(false, why);
                    return;
                }
            }
        }
        if (s_phase_ms > CAL_PHASE_TIMEOUT_MS) {
            RETREAT_OUT(GRIPPER_FAULT_TIMEOUT, GRIPPER_ERROR, -CLOSE_DIR, "squeeze");
        }
        return;
    }

    case CAL_CLOSE_BACKOFF: {
        if (!cal_tactile_guard()) return;
        int st = cal_move_step(s_backoff_target, CAL_SPEED);
        if (st < 0) return;
        for (int i = 0; i < GRIPPER_TACTILE_COUNT; i++) {
            if (s_tac_new[i]) {
                s_sep_cnt[i] = rel(i) < TAC_SEPARATED_N ? s_sep_cnt[i] + 1 : 0;
            }
        }
        if (st == 1) {
            s_arrived_ms += s_dt;
            if (s_sep_cnt[0] >= 3 && s_sep_cnt[1] >= 3) {
                s_pend.pos_closed = s_backoff_target;
                if (s_verify) {
                    s_cal = s_pend;
                    ESP_LOGI(TAG, "home check passed: open %.3f, closed %.3f, touch %.3f rad, k %.1f N/pu",
                             s_cal.pos_open, s_cal.pos_closed, s_cal.pos_touch, s_cal.k_force);
                    cal_done_open();
                    return;
                }
                cal_finish();
            } else if (s_arrived_ms >= 500) {
                if (s_backoff_tries >= CAL_BACKOFF_TRIES) {
                    ERROR_OUT(GRIPPER_FAULT_NOT_SEPARATED, "ring %.2f N, little %.2f N at %.3f rad",
                              rel(0), rel(1), s_in.pos);
                } else {
                    s_backoff_tries++;
                    s_backoff_target -= CLOSE_DIR * CAL_CLOSE_BACKOFF_RAD;
                    s_arrived_ms = 0;
                    s_phase_ms = 0;
                    ESP_LOGW(TAG, "pads still loaded; backing off to %.3f rad", s_backoff_target);
                }
            }
        }
        return;
    }
    }
}

/* =========================================================================
 * Moves and hold
 * ========================================================================= */
static void start_move(float target, float speed)
{
    const float a = pos_at_pct(MOVE_PCT_MIN), b = pos_at_pct(MOVE_PCT_MAX);
    target = clampf(target, fminf(a, b), fmaxf(a, b));
    s_move_target = target;
    s_motion_dir = sgnf(target - s_in.pos);
    move_current_init(s_motion_dir == CLOSE_DIR);
    s_move_speed = speed > 0.0f ? clampf(speed, MOVE_SPEED_MIN, MOVE_SPEED_MAX) : MOVE_SPEED;
    const float t_ms = 1.5f * 1000.0f * fabsf(target - s_in.pos) / s_move_speed;
    s_move_timeout_ms = t_ms > (float)MOVE_TIMEOUT_MS ? (uint32_t)t_ms : MOVE_TIMEOUT_MS;
    s_move_f_start = tac_max_ok();
    s_grip_force_phase = false;
    s_grip_preopen = false;
    s_force_target = 0.0f;
    s_hold = false;
    set_state(GRIPPER_MOVING);
}

/* Raise the move's current limit while the drive sits at it and the jaw
 * falls behind the target but still moves: the motion needs more torque
 * (speed, acceleration, local friction). At the limit and standing still for
 * MOVE_BLOCKED_MS: blocked, returns false. */
static bool move_current_step(void)
{
    const float tpu = tau_per_pu_known();
    float dj = 0.0f;
    if (hist_delta(WIN_MIN_MS, &dj)) s_mv_v_max = fmaxf(s_mv_v_max, fabsf(dj) * 1000.0f / (float)WIN_MIN_MS);
    s_mv_tq_max = fmaxf(s_mv_tq_max, fabsf(s_in.torque));
    s_mv_lag_max = fmaxf(s_mv_lag_max, (s_ramp - s_in.pos) * s_motion_dir);
    const bool at_limit = tpu > 0.0f && fabsf(s_in.torque) >= 0.85f * s_mv_cur * tpu;
    if (!(s_ramp_lagging && at_limit)) {
        s_mv_raise_ms = 0;
        s_mv_block_ms = 0;
        return true;
    }
    if (is_halted()) {
        s_mv_raise_ms = 0;
        s_mv_block_ms += s_dt;
        return s_mv_block_ms < MOVE_BLOCKED_MS;
    }
    s_mv_block_ms = 0;
    s_mv_raise_ms += s_dt;
    if (s_mv_raise_ms >= MOVE_RAISE_MS && s_mv_cur < s_mv_ceil) {
        s_mv_raise_ms = 0;
        const float c = fminf(s_mv_cur * MOVE_RAISE_FACTOR, s_mv_ceil);
        ESP_LOGI(TAG, "the move needs more: current limit %.3f -> %.3f pu (%.2f N*m)", s_mv_cur, c, c * tpu);
        s_mv_cur = c;
    }
    return true;
}

static void move_tick(void)
{
    const bool closing = s_motion_dir == CLOSE_DIR;
    if (closing && !tac_both_ok()) {
        /* Closing blind could press something unseen: stop and wait. */
        if (s_stale_ms == 0) s_stale_hold = s_in.pos;
        s_stale_ms += s_dt;
        (void)drive(s_stale_hold, vel_field(HOLD_SPEED), move_cap());
        if (s_stale_ms >= TAC_STALE_FAULT_MS) {
            NOTE_FAULT(GRIPPER_FAULT_TACTILE_UNAVAILABLE, "closing move stopped at %.3f rad", s_in.pos);
            hold_here();
            set_state(GRIPPER_IDLE);
        }
        return;
    }
    s_stale_ms = 0;
    const float fmax = tac_max_ok();
    /* Only a closing move can press something. An opening move may still see
     * a rising reading for one tactile delay after it starts (e.g. backing
     * off from an empty grasp), which must not stop it. */
    if (closing && fmax > fmaxf(s_move_f_start + MOVE_CONTACT_RISE_N, MOVE_CONTACT_MIN_N)) {
        NOTE_FAULT(GRIPPER_FAULT_CONTACT_DURING_MOVE, "%.2f N at %.3f rad; use GRIP for objects",
                   fmax, s_in.pos);
        hold_here();
        set_state(GRIPPER_IDLE);
        return;
    }
    /* The drive runs the trapezoid to the final target itself; ramp_profile()
     * is the same profile, expected, for the lag check. */
    (void)ramp_profile(s_move_target, s_move_speed, MOVE_ACCEL);
    s_mv_over = fmaxf(s_mv_over, (s_in.pos - s_move_target) * s_motion_dir);
    if (!drive(s_move_target, vel_field(s_move_speed), s_mv_cur)) {
        ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused during a move");
        return;
    }
    if (!move_current_step()) {
        NOTE_FAULT(GRIPPER_FAULT_NO_MOTION, "move blocked at %.3f rad (current limit %.3f pu); holding there",
                   s_in.pos, s_mv_cur);
        hold_here();
        set_state(GRIPPER_IDLE);
        return;
    }
    if (fabsf(s_in.pos - s_move_target) < MOVE_POS_TOL_RAD && is_stalled(s_move_speed)) {
        const float tpu = tau_per_pu_known();
        ESP_LOGI(TAG, "move done: %.3f -> %.3f rad in %u ms; speed %.2f rad/s (max %.2f), torque max %.3f N*m, "
                      "lag max %.3f rad, past the target %.3f rad, current limit %.3f N*m", s_mv_p0, s_in.pos,
                 (unsigned)s_phase_ms, s_move_speed, s_mv_v_max, s_mv_tq_max, s_mv_lag_max, s_mv_over,
                 s_mv_cur * tpu);
        s_hold = true;
        s_hold_cur = -1.0f;
        s_hold_pos = s_move_target;
        set_state(GRIPPER_IDLE);
    } else if (s_phase_ms > s_move_timeout_ms) {
        ESP_LOGW(TAG, "move did not reach %.3f rad (at %.3f); holding here", s_move_target, s_in.pos);
        hold_here();
        set_state(GRIPPER_IDLE);
    }
}

/* =========================================================================
 * Force grip
 * ========================================================================= */
/* The current cap is the only force limit that does not rely on the tactile
 * reading. Tie it to the target, so that if the pads under-read, the force is
 * still bounded near (1.3 x target + 2 N) where k holds, and at most
 * GRIP_HW_CAP_N. */
static float grip_i_cap(void)
{
    float f = fminf(GRIP_CAP_FRAC * s_force_target + GRIP_CAP_MARGIN_N, GRIP_HW_CAP_N);
    float cap = fminf(GRIP_CUR_ABS_MAX_PU,
                      GRIP_STATIC_FACTOR * s_grip_fric + f / s_cal.k_force + s_grip_cap_extra);
    if (GRIP_MIN_LEVER_M > 0.0f && s_cal.tau_per_pu > 0.0f) {
        cap = fminf(cap, s_grip_fric + GRIP_HW_CAP_N * GRIP_MIN_LEVER_M / s_cal.tau_per_pu);
    }
    return cap;
}

static float grip_i_ff(void)
{
    return clampf(s_grip_fric + s_force_target / s_cal.k_force, 0.0f, grip_i_cap());
}

/* Friction current for this grip: the sliding torque measured on the way in,
 * just before contact, never below the calibration's offset. The offset alone
 * comes from the pad-touch region at 0.05 rad/s; elsewhere the friction can
 * be twice that (bench 2026-10-23: 0.10 N*m stored, 0.19-0.21 N*m at 3.9 rad,
 * so the force loop's ceiling only just moved the jaw and the sponge got
 * 0.4 N for a 3 N target). Without an approach (pads already loaded): the
 * moving current. */
static void grip_friction_update(void)
{
    const float tpu = tau_per_pu_known();
    float f = fmaxf(s_cal.i_offset, 0.0f);
    if (s_appr_tq > 0.0f && tpu > 0.0f) {
        f = fmaxf(f, s_appr_tq / tpu);
    } else {
        f = fmaxf(f, i_move_known());
    }
    s_grip_fric = f;
}

/* Ready position for a grip: GRIP_READY_OPEN_FRAC of the opening. */
static float grip_ready_pos(void)
{
    return s_cal.pos_closed + GRIP_READY_OPEN_FRAC * (s_cal.pos_open - s_cal.pos_closed);
}

static void grip_approach_begin(void)
{
    /* Closing at the grip speed toward GRIP_NO_OBJECT_FRAC, speeding up and
     * braking like a move. Current: friction plus GRIP_APPROACH_FORCE_N,
     * raised only while the motion itself needs more (move_current_step()). */
    s_grip_force_phase = false;
    s_grip_preopen = false;
    seek_init(&s_seek, CLOSE_DIR, s_grip_speed, s_cal.i_move,
              friction_plus(GRIP_APPROACH_FORCE_N, cal_cur_max()));
    s_seek.has_limit = true;
    /* closing beyond this without pad force means there is no object */
    s_seek.limit = s_cal.pos_open + GRIP_NO_OBJECT_FRAC * (s_cal.pos_closed - s_cal.pos_open);
    move_current_init(true);
    s_mv_cur = friction_plus(GRIP_APPROACH_FORCE_N, cal_cur_max());
    s_seek.cur = s_mv_cur;
    s_ramp_valid = false;
    s_phase_ms = 0;
    s_appr_tq = 0.0f;
}

static void force_phase_begin(void)
{
    float dj = 0.0f;
    s_contact_v = hist_delta(WIN_MIN_MS, &dj) ? fabsf(dj) * 1000.0f / (float)WIN_MIN_MS : 0.0f;
    s_peak_logging = true;
    s_peak_ms = 0;
    s_peak_f = 0.0f;
    s_grip_force_phase = true;
    grip_friction_update();
    s_grip_cap_extra = 0.0f;
    s_cap_sat_ms = 0;
    s_i_cmd = s_seek.cur;
    s_i_int = s_i_cmd - grip_i_ff();
    s_settle_ms = 0;
    s_lost_ms = 0;
    s_mismatch_ms = 0;
    s_soft_logged = false;
    s_unloading = false;
    s_band_latched = false;
    s_drop_n = 0;
    s_drop_head = 0;
    s_drop_watch_ms = 0;
    s_drop_quiet = GRIP_DROP_WIN_TICKS;
    ESP_LOGI(TAG, "contact at %.3f rad (ring %.2f N, little %.2f N); regulating to %.1f N "
                  "(friction %.3f pu from %s, feed-forward %.3f pu, cap %.3f pu)",
             s_in.pos, s_in.fz[0], s_in.fz[1], s_force_target, s_grip_fric,
             s_appr_tq > 0.0f ? "the approach" : "calibration", grip_i_ff(), grip_i_cap());
}

static void grip_tick(void)
{
    const float limit = s_cal.pos_touch + CLOSE_DIR * GRIP_OVERTRAVEL_RAD;

    if (!tac_both_ok()) {
        if (s_stale_ms == 0) s_stale_hold = s_in.pos;
        s_stale_ms += s_dt;
        if (!s_grip_force_phase) {
            /* Approaching blind: stop where we are. */
            (void)drive(s_stale_hold, vel_field(HOLD_SPEED), s_grip_preopen ? move_cap() : s_seek.cur);
            if (s_stale_ms >= TAC_STALE_FAULT_MS) {
                NOTE_FAULT(GRIPPER_FAULT_TACTILE_UNAVAILABLE, "grip approach stopped at %.3f rad", s_in.pos);
                hold_here();
                set_state(GRIPPER_IDLE);
            }
            return;
        }
        /* Holding an object blind: never raise the current; keep at most the
         * feed-forward value for the target force so the object is not dropped. */
        if (s_fault != GRIPPER_FAULT_TACTILE_STALE) {
            NOTE_FAULT(GRIPPER_FAULT_TACTILE_STALE, "current held at <= %.3f pu until tactile returns",
                       grip_i_ff());
        }
        s_i_cmd = fminf(s_i_cmd, grip_i_ff());
        if (s_stale_ms >= GRIP_BLIND_MAX_MS) {
            /* Not a short dropout: stop regulating a force nobody measures.
             * Keep the object at the same position and current, report it,
             * and leave the decision (SET_CLOSE to open, or GRIP once the pads answer
             * again) to the host. */
            log_fault(true, GRIPPER_FAULT_TACTILE_STALE,
                      "no tactile for %u s: force regulation ended, holding %.3f rad at %.3f pu. "
                      "Open with SET_CLOSE, or GRIP once the pads answer again",
                      (unsigned)(s_stale_ms / 1000u), s_stale_hold, s_i_cmd);
            float cur = s_i_cmd;
            float pos = s_stale_hold;
            s_grip_force_phase = false;
            s_grip_preopen = false;
            s_force_target = 0.0f;
            hold_here();
            s_hold_pos = pos;
            s_hold_cur = cur;
            s_fault = GRIPPER_FAULT_TACTILE_STALE;
            set_state(GRIPPER_IDLE);
            return;
        }
        (void)drive(s_stale_hold, vel_field(GRIP_FORCE_SPEED), s_i_cmd);
        return;
    }
    if (s_stale_ms != 0) {
        s_stale_ms = 0;
        if (s_fault == GRIPPER_FAULT_TACTILE_STALE) {
            ESP_LOGI(TAG, "tactile back, force regulation resumed");
            s_fault = GRIPPER_FAULT_NONE;
        }
    }

    const float fmax = fmaxf(s_in.fz[0], s_in.fz[1]);

    if (s_peak_logging && s_grip_force_phase) {
        s_peak_f = fmaxf(s_peak_f, fmax);
        s_peak_ms += s_dt;
        if (s_peak_ms >= GRIP_PEAK_LOG_MS) {
            s_peak_logging = false;
            ESP_LOGI(TAG, "grip contact: jaw at %.2f rad/s, pad force peak %.2f N in the first %d ms "
                          "(target %.1f N)", s_contact_v, s_peak_f, GRIP_PEAK_LOG_MS, s_force_target);
        }
    }

    if (s_grip_preopen) {
        /* Opening to the ready opening first (only when the jaw is more
         * closed than that; see gripper_grip()). */
        const float target = grip_ready_pos();
        (void)ramp_profile(target, s_grip_speed, MOVE_ACCEL);
        if (!drive(target, vel_field(s_grip_speed), s_mv_cur)) {
            ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused while moving to the grip opening");
            return;
        }
        if (!move_current_step()) {
            NOTE_FAULT(GRIPPER_FAULT_NO_MOTION, "grip: blocked at %.3f rad on the way to the grip opening",
                       s_in.pos);
            hold_here();
            set_state(GRIPPER_IDLE);
            return;
        }
        if (fabsf(s_in.pos - target) < MOVE_POS_TOL_RAD && is_stalled(s_grip_speed)) {
            grip_approach_begin();
            ESP_LOGI(TAG, "grip: at %.0f %% opening; closing at %.2f rad/s, target %.1f N",
                     GRIP_READY_OPEN_FRAC * 100.0f, s_grip_speed, s_force_target);
        } else if (s_phase_ms > MOVE_TIMEOUT_MS) {
            NOTE_FAULT(GRIPPER_FAULT_TIMEOUT, "grip: did not reach the %.0f %% opening (at %.3f rad)",
                       GRIP_READY_OPEN_FRAC * 100.0f, s_in.pos);
            hold_here();
            set_state(GRIPPER_IDLE);
        }
        return;
    }

    if (!s_grip_force_phase) {
        if (fmax >= TAC_TOUCH_N) {
            /* Empty grasp is decided here, at first contact only: the pads
             * meeting each other is contact at the pad-touch position. Later
             * pad compression moves the jaw toward that position for a thin
             * object too, so it is not re-checked in the force phase. */
            if ((s_cal.pos_touch - s_in.pos) * CLOSE_DIR < GRIP_EMPTY_TOL_RAD) {
                NOTE_FAULT(GRIPPER_FAULT_EMPTY_GRASP, "pads met at %.3f rad (touch %.3f): nothing between them",
                           s_in.pos, s_cal.pos_touch);
                start_move(grip_ready_pos(), MOVE_SPEED);
            } else {
                force_phase_begin();
            }
            return;
        }
        if ((s_in.pos - s_seek.limit) * CLOSE_DIR > -MOVE_POS_TOL_RAD) {
            /* Closed to GRIP_NO_OBJECT_FRAC without any pad force. */
            NOTE_FAULT(GRIPPER_FAULT_EMPTY_GRASP, "no object detected up to %.0f %% closed (%.3f rad); back to "
                       "%.0f %% opening", GRIP_NO_OBJECT_FRAC * 100.0f, s_in.pos, GRIP_READY_OPEN_FRAC * 100.0f);
            start_move(grip_ready_pos(), MOVE_SPEED);
            return;
        }
        (void)ramp_profile(s_seek.limit, s_grip_speed, MOVE_ACCEL);
        if (!drive(s_seek.limit, vel_field(s_grip_speed), s_mv_cur)) {
            ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused during the grip approach");
            return;
        }
        if (!move_current_step()) {
            RETREAT_OUT(GRIPPER_FAULT_GRIP_NO_TACTILE, GRIPPER_IDLE, -CLOSE_DIR,
                        "jaw stopped at %.3f rad with no pad force: object outside the pads?", s_in.pos);
            return;
        }
        s_seek.cur = s_mv_cur;   /* the force phase starts from the approach current */
        if (is_moving(CLOSE_DIR, s_grip_speed)) {
            s_appr_tq = fmaxf(fabsf(s_in.torque), s_appr_tq - SEEK_PEAK_DECAY_NM_S * dt_s());
        }
        if (s_phase_ms > GRIP_APPROACH_TIMEOUT_MS) {
            NOTE_FAULT(GRIPPER_FAULT_TIMEOUT, "grip approach");
            hold_here();
            set_state(GRIPPER_IDLE);
        }
        return;
    }

    /* ---- force phase ---- */
    const float k = s_cal.k_force;
    const float ft = s_force_target;
    const float i_ff = grip_i_ff();
    const float i_cap = grip_i_cap();
    const float err = ft - fmax;
    const float db = fmaxf(GRIP_DEADBAND_N, GRIP_DEADBAND_FRAC * ft);
    /* never above the back-off limit: at a 20 N target, 30 % would be 26 N */
    const float soft = fminf(ft + fmaxf(GRIP_SOFT_BAND_N, GRIP_SOFT_BAND_FRAC * ft), FORCE_TRIP_N - 1.0f);

    if (fmax < TAC_TOUCH_N) {
        /* Contact lost (object slipped or was removed): never integrate on no
         * force; drop to the approach current at once and, if it stays lost,
         * go back to the slow approach so the jaw cannot run at the pads. */
        s_lost_ms += s_dt;
        s_i_int = fminf(s_i_int, s_seek.cur - i_ff);
        if (s_lost_ms >= GRIP_LOST_MS) {
            ESP_LOGW(TAG, "contact lost at %.3f rad: approaching again", s_in.pos);
            grip_approach_begin();
            set_state(GRIPPER_GRIPPING);
            return;
        }
    } else {
        s_lost_ms = 0;
    }

    /* Object removed while the pads were pressed past their touch point (a
     * thin object): the force falls, the jaw closes the small gap, the pads
     * meet and the loop would press them together at the target force. A fall
     * of GRIP_DROP_FRAC within 150 ms that this loop did not cause (it is not
     * unloading, and the force is now below the band) starts a watch; if the
     * jaw is at or past the pad-touch position during the watch, nothing is
     * left between the pads. A soft object relaxes over seconds, not 150 ms,
     * and a held object keeps the jaw short of the touch position. */
    {
        /* A fall the loop causes itself (unloading above the band, or a lower
         * target) must not count: the ring is cleared while the force is
         * above the band and stays empty for one window after it leaves. */
        if (s_unloading || fmax > ft + db) {
            s_drop_n = 0;
            s_drop_quiet = GRIP_DROP_WIN_TICKS;
        } else if (s_drop_quiet > 0) {
            s_drop_quiet--;
            s_drop_n = 0;
        } else {
            float peak = fmax;
            for (int j = 1; j <= s_drop_n; j++) {   /* the newest s_drop_n entries */
                peak = fmaxf(peak, s_drop_ring[(s_drop_head + GRIP_DROP_WIN_TICKS - j) % GRIP_DROP_WIN_TICKS]);
            }
            s_drop_ring[s_drop_head] = fmax;
            s_drop_head = (s_drop_head + 1) % GRIP_DROP_WIN_TICKS;
            if (s_drop_n < GRIP_DROP_WIN_TICKS) s_drop_n++;
            if (fmax < ft - db && peak - fmax >= fmaxf(GRIP_DROP_MIN_N, GRIP_DROP_FRAC * peak)) {
                if (s_drop_watch_ms == 0) {
                    s_drop_pos0 = s_in.pos;
                    ESP_LOGW(TAG, "force fell from %.2f to %.2f N within %d ms: watching for the pads meeting",
                             peak, fmax, GRIP_DROP_WIN_TICKS * 10);
                }
                s_drop_watch_ms = GRIP_DROP_WATCH_MS;
            }
        }
        if (s_drop_watch_ms > 0) {
            s_drop_watch_ms = s_drop_watch_ms > s_dt ? s_drop_watch_ms - s_dt : 0;
            /* Removed: the jaw has closed further since the fall (a held
             * object stops it) AND is at the pad-touch position. A jaw that
             * already sat near the touch position on a thin object, without
             * moving, does not count. */
            if ((s_in.pos - s_drop_pos0) * CLOSE_DIR >= GRIP_DROP_TRAVEL_RAD &&
                (s_in.pos - s_cal.pos_touch) * CLOSE_DIR > -GRIP_EMPTY_TOL_RAD) {
                NOTE_FAULT(GRIPPER_FAULT_EMPTY_GRASP, "object removed: force fell and the jaw is at %.3f rad "
                           "(pad touch %.3f); back to the ready opening", s_in.pos, s_cal.pos_touch);
                start_move(grip_ready_pos(), MOVE_SPEED);
                return;
            }
        }
    }

    /* Three zones. Gearbox and linkage friction hold whatever force the jaw
     * was left at, so lowering the current alone cannot reduce an overshoot:
     *   below the band  - push: target ahead of the jaw, current integrates up
     *                     (force on the loading branch is k * (i - i_offset));
     *   inside the band - hold the position where the force was right;
     *   above the band  - integrate down AND walk the target back slowly so
     *                     the drive actively unloads the jaw against friction. */
    const bool above = fmax > ft + db;
    const bool below = fmax < ft - db;
    float tgt;
    if (above) {
        if (fmax > soft) {
            /* Clearly above target: cut to half the feed-forward at once. */
            s_i_int = (s_grip_fric + 0.5f * ft / k) - i_ff;
            if (!s_soft_logged) {
                ESP_LOGW(TAG, "force %.2f N above %.2f N: current cut, unloading", fmax, soft);
                s_soft_logged = true;
            }
        } else {
            s_i_int += GRIP_KI_PER_S * err / k * dt_s();
        }
        if (!s_unloading) {
            s_unloading = true;
            s_unload_tgt = s_in.pos;
        }
        s_unload_tgt -= CLOSE_DIR * GRIP_UNLOAD_SPEED * dt_s();
        if ((s_in.pos - s_unload_tgt) * CLOSE_DIR > GRIP_UNLOAD_MAX_RAD) {
            s_unload_tgt = s_in.pos - CLOSE_DIR * GRIP_UNLOAD_MAX_RAD;
        }
        tgt = s_unload_tgt;
        s_band_latched = false;
    } else if (below) {
        if (fmax >= TAC_TOUCH_N) {
            /* Static friction holds a stalled jaw until the current exceeds it;
             * a rise in proportion to the error is far too slow there. */
            const bool stalled = is_stalled(GRIP_FORCE_SPEED);
            const float rate = stalled ? fmaxf(GRIP_KI_PER_S * err, GRIP_STALL_RAMP_N_S) : GRIP_KI_PER_S * err;
            s_i_int += rate / k * dt_s();
            /* The force decides, not the current: at the ceiling and still
             * short of the target while the pads read the contact, raise it. */
            const float tpu = tau_per_pu_known();
            if (stalled && s_i_cmd >= 0.98f * i_cap && tpu > 0.0f) {
                s_cap_sat_ms += s_dt;
                if (s_cap_sat_ms >= GRIP_CAP_RAISE_MS && s_grip_cap_extra * tpu < GRIP_CAP_EXTRA_MAX_NM) {
                    s_cap_sat_ms = 0;
                    s_grip_cap_extra = fminf(s_grip_cap_extra + GRIP_CAP_RAISE_NM / tpu, GRIP_CAP_EXTRA_MAX_NM / tpu);
                    ESP_LOGI(TAG, "grip: %.2f N of %.1f N at the current ceiling: ceiling +%.2f N*m",
                             fmax, ft, s_grip_cap_extra * tpu);
                }
            } else {
                s_cap_sat_ms = 0;
            }
        }
        tgt = s_in.pos + CLOSE_DIR * SEEK_LEAD_RAD;
        s_unloading = false;
        s_band_latched = false;
    } else {
        if (!s_band_latched) {
            s_band_latched = true;
            s_band_tgt = s_in.pos;
        }
        tgt = s_band_tgt;
        s_unloading = false;
    }
    if ((tgt - limit) * CLOSE_DIR > 0.0f) tgt = limit;
    s_i_int = clampf(s_i_int, -i_ff, i_cap - i_ff);   /* anti-windup */

    const float i_target = i_ff + s_i_int;
    const float up = GRIP_SLEW_N_PER_S / k * dt_s();
    float i_new = i_target > s_i_cmd + up ? s_i_cmd + up : i_target;
    s_i_cmd = clampf(i_new, 0.0f, i_cap);

    if (!drive(tgt, vel_field(GRIP_FORCE_SPEED), s_i_cmd)) {
        ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused while gripping");
        return;
    }

    /* Plausibility: while pushing and stalled, the drive delivers the current
     * limit, so the pads should read about k * (i - offset). Far less means
     * the pads under-read or the object is pressed outside them; the current
     * cap from k is then the only limit left, and it is only approximate. */
    float f_expect = k * (s_i_cmd - s_grip_fric);
    if (below && is_stalled(GRIP_FORCE_SPEED) && f_expect > 2.0f * GRIP_MISMATCH_N &&
        fmax < GRIP_MISMATCH_FRAC * f_expect - GRIP_MISMATCH_N) {
        s_mismatch_ms += s_dt;
        if (s_mismatch_ms >= GRIP_MISMATCH_MS && s_mismatch_ms - s_dt < GRIP_MISMATCH_MS) {
            /* Reported, not acted on: the force is set by the tactile loop and
             * the current is bounded by grip_i_cap(); friction makes the
             * current legitimately high (user's choice, 2026-10-23). */
            ESP_LOGW(TAG, "pads read %.2f N while current %.3f pu implies about %.1f N (friction, or pads "
                          "under-reading); regulating on", fmax, s_i_cmd, f_expect);
        }
    } else {
        s_mismatch_ms = 0;
    }

    if (!above && !below && fmax >= TAC_TOUCH_N) {
        s_settle_ms += s_dt;
    } else {
        s_settle_ms = 0;
    }
    if (s_state == GRIPPER_GRIPPING && s_settle_ms >= GRIP_SETTLE_MS) {
        ESP_LOGI(TAG, "holding %.2f N (target %.1f N) at %.3f pu", fmax, ft, s_i_cmd);
        set_state(GRIPPER_HOLDING);
    } else if (s_state == GRIPPER_HOLDING && fabsf(err) > 3.0f * db) {
        set_state(GRIPPER_GRIPPING);
    }
}

/* =========================================================================
 * Progress log
 * ========================================================================= */
static void progress_log(void)
{
    if (s_state != GRIPPER_CALIBRATING && s_state != GRIPPER_MOVING &&
        s_state != GRIPPER_GRIPPING && s_state != GRIPPER_HOLDING && !s_retreat) {
        s_log_ms = 0;
        return;
    }
    s_log_ms += s_dt;
    uint32_t period = s_state == GRIPPER_HOLDING ? HOLD_LOG_MS : PROGRESS_LOG_MS;
    if (s_log_ms < period) return;
    s_log_ms = 0;
    float d = 0.0f;
    bool have = hist_delta(WIN_MAX_MS / 2u, &d);
    ESP_LOGI(TAG, "%s%s%s pos %.3f rad, speed %.3f rad/s (drive reports %.2f), torque %.3f N*m, "
                  "limit %.3f pu, pads %.2f / %.2f N",
             s_retreat ? "back-off " : gripper_state_name(s_state),
             s_state == GRIPPER_CALIBRATING ? " " : "",
             s_state == GRIPPER_CALIBRATING ? cal_phase_name(s_cal_phase) : "",
             s_in.pos, have ? d * 2000.0f / (float)WIN_MAX_MS : 0.0f, s_in.vel,
             s_in.torque, s_cmd_cur, s_in.fz[0], s_in.fz[1]);
}

/* =========================================================================
 * Public API
 * ========================================================================= */
int gripper_init(bool autocal_on_boot)
{
    memset(&s_cal, 0, sizeof(s_cal));
    memset(&s_pend, 0, sizeof(s_pend));
    s_cal.default_force = GRIP_FORCE_DEFAULT_N;
    nvs_load();
    if (s_nvs_valid && isfinite(s_nvs.default_force) &&
        s_nvs.default_force >= GRIP_FORCE_MIN_N && s_nvs.default_force <= GRIP_FORCE_MAX_N) {
        s_cal.default_force = s_nvs.default_force;
    }
    if (s_nvs_valid) {
        ESP_LOGI(TAG, "last calibration: stroke %.3f rad, k %.1f N/pu, moving current %.3f pu, "
                      "pads %.0f N/rad; default force %.1f N",
                 s_nvs.span, s_nvs.k_force, s_nvs.i_move, s_nvs.pad_stiffness, s_cal.default_force);
        if (have_stored_cal()) {
            ESP_LOGI(TAG, "stored positions: open stop %.3f, open %.3f, closed %.3f rad - GRIP_HOME (0x41) "
                          "finds the pad touch and restores them", s_nvs.p_stop, s_nvs.pos_open, s_nvs.pos_closed);
        } else {
            ESP_LOGI(TAG, "no stored positions for CLOSE_DIR %+.0f - GRIP_CALIBRATE (0x40) needed", CLOSE_DIR);
        }
    }
    s_state = GRIPPER_UNCAL;
    s_fault = GRIPPER_FAULT_NONE;
    s_now_ms = 0;
    s_have_motor = false;
    hist_reset();
    s_motor_on = false;
    s_retreat = false;
    s_hold = false;
    s_verify = false;
    s_overforce_seen = false;
    s_autocal_pending = autocal_on_boot;
    s_autocal_ready_ms = 0;
    ESP_LOGI(TAG, "init (force-position mode, CLOSE_DIR %+.0f, vel field x%.0f), boot calibration %s",
             CLOSE_DIR, VEL_FIELD_SCALE, autocal_on_boot ? "on" : "off");
    return ESP_OK;
}

void gripper_tick(const gripper_input_t *in)
{
    if (!in) return;
    s_in = *in;
    s_have_in = true;
    s_dt = in->dt_ms > 100u ? 100u : in->dt_ms;
    s_now_ms += s_dt;
    s_phase_ms += s_dt;
    s_drove = false;

    /* A new motor epoch means the drive was re-probed (feedback lost, or it
     * dropped out of "enabled" by itself): it may have restarted, so its
     * position frame and registers may have changed. */
    if (!s_have_epoch || in->motor_epoch != s_epoch_seen) {
        if (s_have_epoch) {
            ESP_LOGW(TAG, "motor epoch %u -> %u: position frame may have changed",
                     (unsigned)s_epoch_seen, (unsigned)in->motor_epoch);
            s_have_motor = false;
            hist_reset();
            /* The sample in hand may predate the restart (old frame); take
             * positions only from the next frame on. */
            s_epoch_skip = true;
            s_epoch_skip_seq = in->motor_seq;
            if (s_cal.valid || s_motor_on || s_state == GRIPPER_CALIBRATING) {
                s_cal.valid = false;
                if (s_motor_on) motor_off();
                s_hold = false;
                s_retreat = false;
                if (s_state != GRIPPER_ERROR) {
                    NOTE_FAULT(GRIPPER_FAULT_MOTOR_LOST, "calibration discarded; run GRIP_CALIBRATE");
                    set_state(GRIPPER_UNCAL);
                }
            }
        }
        s_have_epoch = true;
        s_epoch_seen = in->motor_epoch;
    }

    /* Feedback bookkeeping. */
    if (s_epoch_skip && in->motor_seq != s_epoch_skip_seq) {
        s_epoch_skip = false;
    }
    if (in->motor_fresh && !s_epoch_skip && (!s_have_motor || in->motor_seq != s_motor_seq)) {
        if (s_have_motor && fabsf(in->pos - s_prev_pos) > POS_JUMP_RAD) {
            s_cal.valid = false;
            hist_reset();
            s_have_motor = false;
            ERROR_OUT(GRIPPER_FAULT_POSITION_JUMP, "%.3f -> %.3f rad between two feedback frames",
                      s_prev_pos, in->pos);
            return;
        }
        s_have_motor = true;
        s_motor_seq = in->motor_seq;
        s_prev_pos = in->pos;
        hist_push(s_now_ms, in->pos, in->torque);
    }
    for (int i = 0; i < GRIPPER_TACTILE_COUNT; i++) {
        s_tac_new[i] = in->tac_ok[i] && (!s_tac_seen[i] || in->tac_seq[i] != s_tac_seq[i]);
        if (in->tac_ok[i]) {
            s_tac_seq[i] = in->tac_seq[i];
            s_tac_seen[i] = true;
        }
        /* One sample far from the last accepted one is held back until the
         * next sample: a real force change persists, a corrupt frame does not.
         * Everything below sees the filtered value. */
        static float fz_ok[GRIPPER_TACTILE_COUNT], pt_ok[GRIPPER_TACTILE_COUNT];
        static bool held[GRIPPER_TACTILE_COUNT], have_ok[GRIPPER_TACTILE_COUNT];
        if (s_tac_new[i]) {
            if (have_ok[i] && !held[i] && fabsf(s_in.fz[i] - fz_ok[i]) > TAC_SPIKE_N) {
                held[i] = true;
                ESP_LOGW(TAG, "pad %d: sample %.1f N after %.1f N held back until the next one", i,
                         s_in.fz[i], fz_ok[i]);
            } else {
                held[i] = false;
                fz_ok[i] = s_in.fz[i];
                pt_ok[i] = s_in.fz_point_max[i];
                have_ok[i] = true;
            }
        }
        if (have_ok[i]) {
            s_in.fz[i] = fz_ok[i];
            s_in.fz_point_max[i] = pt_ok[i];
        }
    }

    /* Motor health while we drive it. */
    if (s_motor_on) {
        s_enable_ms += s_dt;
        if (!in->allow_motion) {
            ESP_LOGW(TAG, "motion no longer allowed - motor off");
            gripper_abort();
            return;
        }
        if (!in->motor_ready || !in->motor_fresh) {
            ERROR_OUT(GRIPPER_FAULT_MOTOR_LOST, "feedback %s", in->motor_ready ? "stale" : "lost");
            return;
        }
        if (in->motor_error != 0) {
            ERROR_OUT(GRIPPER_FAULT_MOTOR_FAULT, "drive error code 0x%X", in->motor_error);
            return;
        }
        if (!in->motor_enabled && s_enable_ms > 200) {
            ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_ENABLED, "feedback still shows disabled after %u ms",
                      (unsigned)s_enable_ms);
            return;
        }
    }

    /* A tare redefines zero. Anything but the gripper's own tare phase while
     * the motor is driven invalidates the force readings in use. */
    bool retared = false;
    if (s_have_tare_seen && in->tare_count != s_tare_seen) {
        bool own = s_state == GRIPPER_CALIBRATING && s_cal_phase == CAL_TARE;
        if (!own && s_motor_on && !s_retreat) {
            float dir = s_state == GRIPPER_CALIBRATING ? cal_away_dir() : -CLOSE_DIR;
            RETREAT_OUT(GRIPPER_FAULT_TACTILE_RETARED,
                        s_state == GRIPPER_CALIBRATING ? GRIPPER_ERROR : GRIPPER_IDLE, dir,
                        "tactile zero changed while driving");
            retared = true;
        }
    }
    s_tare_seen = in->tare_count;
    s_have_tare_seen = true;

    if (retared) {
        /* the back-off (if any) runs from the next tick */
    } else if (s_motor_on && !s_retreat && s_state != GRIPPER_ERROR && overforce_check()) {
        /* reacted; the back-off (if any) runs from the next tick */
    } else if (s_retreat) {
        retreat_tick();
    } else {
        switch (s_state) {
        case GRIPPER_UNCAL:
            if (s_autocal_pending) {
                if (s_now_ms > AUTOCAL_WINDOW_MS) {
                    s_autocal_pending = false;
                    ESP_LOGW(TAG, "boot calibration skipped: motor and tactile were not ready within "
                                  "%d s of boot. Send GRIP_CALIBRATE.", AUTOCAL_WINDOW_MS / 1000);
                    break;
                }
                bool ready = in->allow_motion && in->motor_ready && in->motor_fresh && tac_both_ok();
                s_autocal_ready_ms = ready ? s_autocal_ready_ms + s_dt : 0;
                s_autocal_log_ms += s_dt;
                if (s_autocal_ready_ms >= AUTOCAL_DELAY_MS) {
                    (void)gripper_home();
                } else if (s_autocal_log_ms >= 5000) {
                    s_autocal_log_ms = 0;
                    ESP_LOGI(TAG, "boot calibration waiting: motor %s, ring pad %s, little pad %s%s",
                             in->motor_ready ? (in->motor_fresh ? "ready" : "stale") : "not ready",
                             in->tac_ok[0] ? "ok" : "missing", in->tac_ok[1] ? "ok" : "missing",
                             in->allow_motion ? "" : ", motion not allowed");
                }
            }
            break;
        case GRIPPER_IDLE:
            if (s_hold) {
                float cur = s_hold_cur >= 0.0f ? s_hold_cur : move_cap();
            if (!drive(ramp_toward(s_hold_pos, HOLD_SPEED), vel_field(HOLD_SPEED), cur)) {
                    ERROR_OUT(GRIPPER_FAULT_MOTOR_NOT_READY, "drive refused while holding");
                }
            }
            break;
        case GRIPPER_CALIBRATING:
            cal_tick();
            break;
        case GRIPPER_MOVING:
            move_tick();
            break;
        case GRIPPER_GRIPPING:
        case GRIPPER_HOLDING:
            grip_tick();
            break;
        case GRIPPER_ERROR:
        default:
            break;
        }
    }

    /* motor_twai disables the motor when the setpoint is not refreshed for a
     * while. Paths that skip drive() for one tick (a phase change, the start
     * of a back-off) re-send the last setpoint here. */
    if (s_motor_on && !s_drove) {
        motor_twai_set_force_pos(s_last_tgt, s_last_vel, s_cmd_cur);
    }
    progress_log();
}

static bool motion_ready(bool need_tactile)
{
    if (!s_have_in || !s_in.allow_motion || !s_in.motor_ready || !s_in.motor_fresh) {
        s_fault = GRIPPER_FAULT_MOTOR_NOT_READY;
        return false;
    }
    if (need_tactile && !tac_both_ok()) {
        s_fault = GRIPPER_FAULT_TACTILE_UNAVAILABLE;
        return false;
    }
    return true;
}

static bool cal_start(bool verify)
{
    if (!motion_ready(true)) {
        ESP_LOGW(TAG, "%s refused: %s", verify ? "home check" : "calibration", gripper_fault_name(s_fault));
        return false;
    }
    motor_off();
    memset(&s_pend, 0, sizeof(s_pend));
    s_cal.valid = false;
    s_fault = GRIPPER_FAULT_NONE;
    s_retreat = false;
    s_hold = false;
    s_grip_force_phase = false;
    s_grip_preopen = false;
    s_force_target = 0.0f;
    s_motion_dir = 0.0f;
    s_have_stop = false;
    s_have_touch = false;
    for (int i = 0; i < GRIPPER_TACTILE_COUNT; i++) {
        s_base_sum[i] = 0.0f;
        s_base_n[i] = 0;
        s_touch_cnt[i] = 0;
        s_baseline[i] = 0.0f;
    }
    s_open_retry = 0;
    s_autocal_pending = false;
    s_verify = verify;
    s_cal_note = GRIPPER_FAULT_NONE;
    if (verify) {
        /* The close seek starts from the stored moving current and trims with
         * the stored N*m per pu. */
        s_pend.i_move = s_nvs.i_move;
        s_pend.tau_per_pu = s_nvs.tau_per_pu > 0.0f ? s_nvs.tau_per_pu : s_in.motor_tau_per_pu_est;
    }
    ESP_LOGW(TAG, "%s", verify ? "home check: tare the pads, close until they touch, compare with the "
                                 "stored calibration"
                               : "calibration: the jaw opens to its limit, then closes until the pads touch");
    set_state(GRIPPER_CALIBRATING);
    cal_set_phase(CAL_BASELINE);
    return true;
}

bool gripper_calibrate(void)
{
    return cal_start(false);
}

bool gripper_home(void)
{
    if (!have_stored_cal()) {
        ESP_LOGW(TAG, "home: no stored calibration for CLOSE_DIR %+.0f - running a full calibration", CLOSE_DIR);
        return cal_start(false);
    }
    return cal_start(true);
}

bool gripper_has_stored_calib(void)
{
    return have_stored_cal();
}

bool gripper_grip(float force_n, float speed)
{
    if (!isfinite(force_n) || !isfinite(speed)) return false;
    float f = force_n > 0.0f ? force_n : s_cal.default_force;
    f = clampf(f, GRIP_FORCE_MIN_N, GRIP_FORCE_MAX_N);
    if (!calibrated() || s_state == GRIPPER_ERROR || s_state == GRIPPER_CALIBRATING) {
        ESP_LOGW(TAG, "grip refused: state %s%s", gripper_state_name(s_state),
                 calibrated() ? "" : ", not calibrated");
        return false;
    }
    if (!motion_ready(true)) {
        ESP_LOGW(TAG, "grip refused: %s", gripper_fault_name(s_fault));
        return false;
    }
    if (s_overforce_seen && s_now_ms - s_overforce_at < GRIP_LOCKOUT_MS) {
        /* A host that re-sends GRIP in a loop would otherwise drive straight
         * back into whatever just tripped the over-force limit. */
        ESP_LOGW(TAG, "grip refused: over-force %u ms ago (wait %d ms)",
                 (unsigned)(s_now_ms - s_overforce_at), GRIP_LOCKOUT_MS);
        s_fault = GRIPPER_FAULT_OVERFORCE;
        return false;
    }
    if ((s_state == GRIPPER_GRIPPING || s_state == GRIPPER_HOLDING) && s_grip_force_phase && !s_retreat) {
        s_force_target = f;
        s_settle_ms = 0;
        s_soft_logged = false;
        s_drop_n = 0;
        s_drop_watch_ms = 0;
        s_drop_quiet = GRIP_DROP_WIN_TICKS;
        ESP_LOGI(TAG, "grip force target -> %.1f N", f);
        return true;
    }
    /* Shortcut below: only for an object held still in IDLE (after a
     * tactile dropout, or a closing move that met it), never right after an
     * empty grasp (the pads may still touch while the jaw opens). */
    const bool held = s_state == GRIPPER_IDLE && s_hold && s_motor_on && s_fault != GRIPPER_FAULT_EMPTY_GRASP;
    const float held_cur = s_cmd_cur;
    s_retreat = false;
    s_force_target = f;
    s_fault = GRIPPER_FAULT_NONE;
    s_hold = false;
    s_lost_ms = 0;
    s_grip_force_phase = false;
    s_grip_preopen = false;
    set_state(GRIPPER_GRIPPING);
    if (held && tac_max_ok() >= TAC_TOUCH_N) {
        grip_approach_begin();
        /* Something is already pressed between the pads (held after a tactile
         * dropout, or met by a closing move). The empty-grasp test compares
         * the FIRST contact with the pad-touch position; here the jaw is
         * already compressed onto the object, so the test does not apply:
         * regulate from the present current. */
        s_seek.cur = held_cur;
        ESP_LOGI(TAG, "grip: pads already loaded (%.2f N): regulating to %.1f N from %.3f pu",
                 tac_max_ok(), f, s_seek.cur);
        force_phase_begin();
        return true;
    }
    s_grip_speed = speed > 0.0f ? clampf(speed, MOVE_SPEED_MIN, MOVE_SPEED_MAX) : MOVE_SPEED;
    if ((s_in.pos - grip_ready_pos()) * CLOSE_DIR > MOVE_POS_TOL_RAD) {
        /* more closed than the ready opening: open to it first */
        s_grip_preopen = true;
        s_motion_dir = -CLOSE_DIR;
        move_current_init(false);
        ESP_LOGI(TAG, "grip: to %.0f %% opening first, then closing at %.2f rad/s; target %.1f N",
                 GRIP_READY_OPEN_FRAC * 100.0f, s_grip_speed, f);
    } else {
        grip_approach_begin();
        ESP_LOGI(TAG, "grip: closing at %.2f rad/s; target %.1f N", s_grip_speed, f);
    }
    return true;
}

/* Closing moves are refused for GRIP_LOCKOUT_MS after an over-force, like
 * GRIP: a host loop must not drive straight back into what tripped it. */
static bool closing_locked(float target, const char *what)
{
    if (!s_overforce_seen || s_now_ms - s_overforce_at >= GRIP_LOCKOUT_MS ||
        (target - s_in.pos) * CLOSE_DIR <= 0.0f) {
        return false;
    }
    ESP_LOGW(TAG, "%s refused: closing move %u ms after an over-force (wait %d ms)",
             what, (unsigned)(s_now_ms - s_overforce_at), GRIP_LOCKOUT_MS);
    s_fault = GRIPPER_FAULT_OVERFORCE;
    return true;
}

static bool move_allowed(void)
{
    if (!calibrated() || s_state == GRIPPER_ERROR || s_state == GRIPPER_CALIBRATING) {
        ESP_LOGW(TAG, "move refused: state %s%s", gripper_state_name(s_state),
                 calibrated() ? "" : ", not calibrated");
        return false;
    }
    if (!motion_ready(false)) {
        ESP_LOGW(TAG, "move refused: %s", gripper_fault_name(s_fault));
        return false;
    }
    s_retreat = false;
    s_fault = GRIPPER_FAULT_NONE;
    return true;
}

bool gripper_set_closing(float percent, float speed)
{
    if (!isfinite(percent) || !isfinite(speed) || !move_allowed()) return false;
    if (percent < MOVE_PCT_MIN || percent > MOVE_PCT_MAX) {
        ESP_LOGW(TAG, "closing %.1f %% outside %.0f..%.0f %%: moving to the nearer end", percent,
                 MOVE_PCT_MIN, MOVE_PCT_MAX);
        percent = clampf(percent, MOVE_PCT_MIN, MOVE_PCT_MAX);
    }
    float target = pos_at_pct(percent);
    if (closing_locked(target, "set-closing")) return false;
    start_move(target, speed);
    return true;
}

bool gripper_set_opening(float frac)
{
    if (!isfinite(frac) || !move_allowed()) return false;
    frac = clampf(frac, 0.0f, 1.0f);
    float target = s_cal.pos_closed + frac * (s_cal.pos_open - s_cal.pos_closed);
    if (closing_locked(target, "set-opening")) return false;
    start_move(target, MOVE_SPEED);
    return true;
}

bool gripper_move_deg(float deg_from_closed, float dps)
{
    if (!isfinite(deg_from_closed) || !isfinite(dps) || !move_allowed()) return false;
    const float deg2rad = 0.017453292f;
    float target = s_cal.pos_closed - CLOSE_DIR * deg_from_closed * deg2rad;
    float speed = dps > 0.0f ? dps * deg2rad : 0.0f;   /* start_move() bounds it */
    if (closing_locked(target, "move")) return false;
    start_move(target, speed);
    return true;
}

bool gripper_set_default_force(float force_n)
{
    if (!isfinite(force_n) || force_n < GRIP_FORCE_MIN_N || force_n > GRIP_FORCE_MAX_N) {
        return false;
    }
    s_cal.default_force = force_n;
    s_nvs.default_force = force_n;
    nvs_store();
    ESP_LOGI(TAG, "default grip force %.1f N (saved)", force_n);
    return true;
}

void gripper_stop(void)
{
    if (s_state == GRIPPER_CALIBRATING) {
        motor_off();
        s_retreat = false;
        ESP_LOGW(TAG, "calibration stopped");
        set_state(GRIPPER_UNCAL);
        return;
    }
    s_retreat = false;
    s_grip_force_phase = false;
    s_grip_preopen = false;
    s_force_target = 0.0f;
    if (s_motor_on && calibrated()) {
        hold_here();
        set_state(GRIPPER_IDLE);
    } else if (s_state != GRIPPER_ERROR) {
        motor_off();
        s_hold = false;
        set_state(calibrated() ? GRIPPER_IDLE : GRIPPER_UNCAL);
    }
}

void gripper_abort(void)
{
    motor_off();
    s_retreat = false;
    s_hold = false;
    s_grip_force_phase = false;
    s_grip_preopen = false;
    s_force_target = 0.0f;
    if (s_state == GRIPPER_CALIBRATING) {
        set_state(GRIPPER_UNCAL);
    } else if (s_state != GRIPPER_ERROR && s_state != GRIPPER_UNCAL) {
        set_state(calibrated() ? GRIPPER_IDLE : GRIPPER_UNCAL);
    }
}

void gripper_clear_fault(void)
{
    s_fault = GRIPPER_FAULT_NONE;
    if (s_state == GRIPPER_ERROR) {
        set_state(calibrated() ? GRIPPER_IDLE : GRIPPER_UNCAL);
    }
}

gripper_state_t gripper_get_state(void) { return s_state; }

bool gripper_is_busy(void)
{
    return s_state == GRIPPER_CALIBRATING || s_state == GRIPPER_MOVING ||
           s_state == GRIPPER_GRIPPING || s_state == GRIPPER_HOLDING || s_retreat;
}

bool gripper_motor_active(void) { return s_motor_on; }

void gripper_get_calib(gripper_calib_t *out)
{
    if (!out) return;
    *out = s_cal;
    out->valid = calibrated();
}

void gripper_get_status(gripper_status_t *out)
{
    if (!out) return;
    out->state = s_state;
    out->fault = s_fault;
    out->pos = s_in.pos;
    out->opening = -1.0f;
    if (calibrated()) {
        float span = s_cal.pos_open - s_cal.pos_closed;
        if (fabsf(span) > 1e-6f) {
            out->opening = clampf((s_in.pos - s_cal.pos_closed) / span, 0.0f, 1.0f);
        }
    }
    out->force_target = s_force_target;
    out->force[0] = s_in.fz[0];
    out->force[1] = s_in.fz[1];
    out->cur_cmd = s_cmd_cur;
}
