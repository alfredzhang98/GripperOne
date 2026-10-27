/**
 * @file app_supervisor.h
 * @brief Command dispatch hub and gripper control loop.
 *
 * The single place where a decoded user command becomes a driver call. The
 * transports (UART / BLE / Wi-Fi) never interpret bytes: they feed the raw
 * stream to app_proto, which validates and decodes it onto bus->ctrl_q, and
 * this module executes it. Adding a command therefore touches two files — the
 * opcode in app_protocol.h and one case here — and no transport.
 *
 * It drains bus->imu_q, bus->haptic_q and bus->motor_status_q, publishes
 * opt-in telemetry, and runs gripper_tick() every 10 ms. Motion needs an
 * unlocked session and is blocked by SAFE_STOP or a motor fault until
 * MOTOR_CLEAR_ERR.
 */
#ifndef APP_SUPERVISOR_H
#define APP_SUPERVISOR_H

#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int task_stack;   /* task stack depth, bytes (0 = built-in default) */
    int task_prio;    /* FreeRTOS priority       (0 = built-in default) */
    float overforce_n;/* pad force (N) that forces SAFE_STOP while the motor is
                       * enabled; last resort behind the gripper's own limits
                       * (<= 0 = built-in default 24 N)                     */
} app_supervisor_cfg_t;

/**
 * @brief Spawn supervisor_task.
 * @param bus  message bus it will drain (needs imu_q, haptic_q, motor_status_q
 *             and ctrl_q).
 * @param cfg  task sizing (may be NULL for defaults).
 * @return ESP_OK on success.
 *
 * NOTE: it becomes the consumer of imu_q, haptic_q and motor_status_q. A queue has
 * exactly one reader per message, so nothing else may drain them or the two
 * consumers will split the stream between them.
 */
int app_supervisor_start(const app_bus_t *bus, const app_supervisor_cfg_t *cfg);

/**
 * @brief Read-only pointer to the current system state (for status/telemetry).
 *        Valid after app_supervisor_start().
 */
const volatile app_state_t *app_supervisor_state_ptr(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SUPERVISOR_H */
