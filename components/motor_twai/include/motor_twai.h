/**
 * @file motor_twai.h
 * @brief Motor control over TWAI (CAN 2.0), targeting the Damiao DM-J4310-2EC.
 *
 * Owns the TWAI driver and a task that (a) periodically transmits the command
 * frame for the currently-selected control mode and (b) drains received frames
 * into motor_status_t messages on bus->motor_status_q.
 *
 * The DM-J4310 supports FOUR control modes, selected by register 0x0A
 * (CTRL_MODE). Each mode has its OWN command-frame ID offset and byte layout,
 * but the feedback frame is identical across modes. This driver implements all
 * four, plus the register read/write/save frames used to switch mode, retune
 * ranges, and change baud rate over CAN. See motor_twai.c for the exact byte
 * packing taken from the DM-J4310-2EC V1.2 manual (CAN communication chapter).
 *
 * TYPICAL USE
 *   1. motor_twai_start(&cfg, &bus);              // bring up the bus
 *   2. motor_twai_set_mode(MOTOR_MODE_VELOCITY);  // set motor CTRL_MODE (0x0A)
 *   3. motor_twai_enable();                        // FF..FC enable frame
 *   4. motor_twai_set_velocity(5.0f);              // stream setpoints @100 Hz
 *   ...
 *   5. motor_twai_disable();
 *
 * The command frame for the active mode is re-sent every 10 ms regardless of
 * whether the setpoint changed — this also services the motor's comms-loss
 * timeout protection (register 0x09 TIMEOUT), which disables the motor if no
 * CAN command arrives within the configured window.
 *
 * HARDWARE NOTE: the ESP32-C6 has no built-in CAN transceiver. An external
 * 3.3 V transceiver (TCAN332 / SN65HVD230 / similar) is mandatory between the
 * TWAI TX/RX pins and the differential CAN bus.
 */
#ifndef MOTOR_TWAI_H
#define MOTOR_TWAI_H

#include <stdint.h>
#include <stdbool.h>
#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Control modes = the value of register 0x0A (CTRL_MODE). The motor only
 * accepts the command frame that matches its current CTRL_MODE register. */
typedef enum {
    MOTOR_MODE_MIT       = 1,  /* MIT impedance: pos/vel/kp/kd/torque         */
    MOTOR_MODE_POS_VEL   = 2,  /* position with trapezoidal velocity limit    */
    MOTOR_MODE_VELOCITY  = 3,  /* constant velocity                           */
    MOTOR_MODE_FORCE_POS = 4,  /* position + velocity-limit + current-limit   */
} motor_ctrl_mode_t;

/* DM-J4310 register addresses (RID) used by the CAN param read/write frames.
 * Full list is in the manual's "寄存器列表及范围" table. */
#define MOTOR_REG_MST_ID     0x07  /* feedback frame ID          (uint32)     */
#define MOTOR_REG_ESC_ID     0x08  /* receive/command frame ID   (uint32)     */
#define MOTOR_REG_TIMEOUT    0x09  /* comms-loss timeout, 50us ticks (uint32) */
#define MOTOR_REG_CTRL_MODE  0x0A  /* control mode 1..4          (uint32)     */
#define MOTOR_REG_PMAX       0x15  /* position map range, rad    (float)      */
#define MOTOR_REG_VMAX       0x16  /* velocity map range, rad/s  (float)      */
#define MOTOR_REG_TMAX       0x17  /* torque map range, Nm       (float)      */
#define MOTOR_REG_CAN_BR     0x23  /* CAN baud-rate code 0..4    (uint32)     */
#define MOTOR_REG_P_MOTOR    0x50  /* current motor-side position, rad (float, RO) */
#define MOTOR_REG_XOUT       0x51  /* output-shaft position, rad (float, RO)  */

/* CAN baud-rate codes for register 0x23 (MOTOR_REG_CAN_BR). */
typedef enum {
    MOTOR_BAUD_125K = 0,
    MOTOR_BAUD_200K = 1,
    MOTOR_BAUD_250K = 2,
    MOTOR_BAUD_500K = 3,
    MOTOR_BAUD_1M   = 4,
} motor_baud_code_t;

typedef struct {
    int tx_gpio;
    int rx_gpio;
    int bitrate_kbps;   /* 125 / 250 / 500 / 1000                             */
    int esc_id;         /* host->motor command frame ID (<0 = built-in default) */
    int mst_id;         /* motor->host feedback frame ID (<0 = built-in default) */
    int task_stack;     /* task stack depth, bytes (0 = built-in default)     */
    int task_prio;      /* FreeRTOS priority       (0 = built-in default)     */
} motor_twai_cfg_t;

/**
 * @brief Install the TWAI driver, start the bus, and spawn motor_twai_task.
 * @return ESP_OK on success. The active control mode defaults to MIT.
 */
int motor_twai_start(const motor_twai_cfg_t *cfg, const app_bus_t *bus);

/* ---- enable / fault control (data-segment fixed, any mode) -------------- */

/** @brief Request the motor be enabled (FF..FC). Thread-safe. */
void motor_twai_enable(void);
/** @brief Request the motor be disabled (FF..FD). */
void motor_twai_disable(void);
/** @brief Set the current shaft position as zero (FF..FE). */
void motor_twai_save_zero(void);
/** @brief Clear a latched motor fault (FF..FB). */
void motor_twai_clear_error(void);

/* ---- mode selection ----------------------------------------------------- */

/**
 * @brief Switch the motor's control mode by writing register 0x0A, and select
 *        the matching command frame for the periodic sender.
 *
 * The motor zeroes its position/velocity/torque/kp/kd command values on a mode
 * switch, and the change is NOT saved to flash (call motor_twai_save_params()
 * while disabled to persist). Switch at (near) zero speed to avoid a jolt; for
 * a position-holding mode, read register 0x50 first and command that position.
 */
void motor_twai_set_mode(motor_ctrl_mode_t mode);

/** @brief The control mode the periodic sender is currently transmitting. */
motor_ctrl_mode_t motor_twai_get_mode(void);

/* ---- per-mode setpoint interfaces --------------------------------------- *
 * Each setter updates the setpoint AND selects its mode as the one the
 * periodic sender transmits. The motor's CTRL_MODE register (0x0A) must
 * already match — use motor_twai_set_mode() (or the debug assistant) first. */

/**
 * @brief MODE 1 (MIT). Full impedance command.
 * kp=0,kd!=0 + vel  -> constant-velocity spin.
 * kp=0,kd=0  + tff  -> pure torque output.
 * For position control kd MUST be non-zero or the motor oscillates / runs away.
 */
void motor_twai_set_mit(float pos, float vel, float kp, float kd, float tff);

/** @brief MODE 1 convenience: position only (vel/torque zeroed, gains kept). */
void motor_twai_set_target(float target);
/** @brief MODE 1 impedance gains. Kd should stay non-zero for stable holds. */
void motor_twai_set_gains(float kp, float kd);

/**
 * @brief MODE 2 (position-velocity). Drive to @p pos (rad) with a trapezoidal
 *        profile capped at |@p vel| (rad/s). ACC/DEC come from registers 0x04/0x05.
 */
void motor_twai_set_pos_vel(float pos, float vel);

/** @brief MODE 3 (velocity). Hold constant @p vel (rad/s). */
void motor_twai_set_velocity(float vel);

/**
 * @brief MODE 4 (force-position hybrid). Drive to @p pos (rad) under a speed
 *        cap and a torque-current cap.
 * @param vel_limit rotor speed cap, rad/s, clamped to 0..100.
 * @param cur_limit current-limit per-unit (I/Imax), clamped to 0..1.0.
 */
void motor_twai_set_force_pos(float pos, float vel_limit, float cur_limit);

/* ---- register access (config frames on CAN ID 0x7FF) -------------------- */

/** @brief Write a float register (RID). Takes effect immediately, not saved. */
void motor_twai_write_param_f32(uint8_t rid, float value);
/** @brief Write a uint32 register (RID). Takes effect immediately, not saved. */
void motor_twai_write_param_u32(uint8_t rid, uint32_t value);
/** @brief Persist all registers to flash. DISABLED mode only; ~1e4 write cycles. */
void motor_twai_save_params(void);
/** @brief Change CAN baud rate (writes register 0x23). Effective after switch-over. */
void motor_twai_set_baudrate(motor_baud_code_t code);

/**
 * @brief Request a register read (RID). The reply arrives asynchronously on the
 *        feedback ID; poll motor_twai_get_param_*() to retrieve the cached value.
 */
void motor_twai_request_param(uint8_t rid);
/** @brief Fetch the last cached read of register @p rid as uint32. @return false if never read. */
bool motor_twai_get_param_u32(uint8_t rid, uint32_t *out);
/** @brief Fetch the last cached read of register @p rid as float. @return false if never read. */
bool motor_twai_get_param_f32(uint8_t rid, float *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TWAI_H */
