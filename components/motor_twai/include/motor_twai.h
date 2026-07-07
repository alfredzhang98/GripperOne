/**
 * @file motor_twai.h
 * @brief Motor control over TWAI (CAN 2.0), targeting the DM-J4310-2EC.
 *
 * Owns the TWAI driver and a task that (a) periodically transmits a motor
 * command frame and (b) drains received frames into motor_status_t messages
 * on bus->motor_status_q.
 *
 * HARDWARE NOTE: the ESP32-C6 has no built-in CAN transceiver. An external
 * 3.3 V transceiver (TCAN332 / SN65HVD230 / similar) is mandatory between the
 * TWAI TX/RX pins and the differential CAN bus.
 *
 * The DM-J4310 MIT-mode framing is not yet implemented; the command/feedback
 * codecs are placeholders (see TODO markers in motor_twai.c).
 */
#ifndef MOTOR_TWAI_H
#define MOTOR_TWAI_H

#include <stdint.h>
#include <stdbool.h>
#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int tx_gpio;
    int rx_gpio;
    int bitrate_kbps;   /* 125 / 250 / 500 / 1000 */
    int esc_id;         /* host->motor command frame ID (<0 = built-in default) */
    int mst_id;         /* motor->host feedback frame ID (<0 = built-in default) */
    int task_stack;     /* task stack depth, bytes (0 = built-in default) */
    int task_prio;      /* FreeRTOS priority       (0 = built-in default) */
} motor_twai_cfg_t;

/**
 * @brief Install the TWAI driver, start the bus, and spawn motor_twai_task.
 * @return ESP_OK on success.
 */
int motor_twai_start(const motor_twai_cfg_t *cfg, const app_bus_t *bus);

/** @brief Request the motor be enabled (thread-safe; may be called from BLE). */
void motor_twai_enable(void);

/** @brief Request the motor be disabled. */
void motor_twai_disable(void);

/** @brief Set the motor target position (rad); velocity and torque are zeroed. */
void motor_twai_set_target(float target);

/**
 * @brief Set the full MIT command (position/velocity/gains/torque-feedforward).
 * This is what the gripper layer uses for position, velocity, or constant-force
 * (torque) control. Values are re-sent every command cycle until changed.
 */
void motor_twai_set_mit(float pos, float vel, float kp, float kd, float tff);

/** @brief Set MIT impedance gains. Kd should stay non-zero for stable holds. */
void motor_twai_set_gains(float kp, float kd);

/** @brief Set the current shaft position as zero (Damiao save-zero frame). */
void motor_twai_save_zero(void);

/** @brief Clear a latched motor fault (Damiao clear-error frame). */
void motor_twai_clear_error(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TWAI_H */
