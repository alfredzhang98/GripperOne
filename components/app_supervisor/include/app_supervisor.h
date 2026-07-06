/**
 * @file app_supervisor.h
 * @brief Central state machine and safety authority.
 *
 * The supervisor is the only module that decides what the gripper actually
 * does. It drains the sensor and control queues, runs the
 * INIT/IDLE/ACTIVE/ERROR/SAFE_STOP state machine, forwards vetted commands to
 * the motor, and publishes status back over BLE.
 *
 * Drivers never talk to each other directly — they only produce/consume queue
 * messages, and the supervisor is the hub.
 */
#ifndef APP_SUPERVISOR_H
#define APP_SUPERVISOR_H

#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Spawn supervisor_task.
 * @param bus  message bus it will drain.
 * @return ESP_OK on success.
 */
int app_supervisor_start(const app_bus_t *bus);

/**
 * @brief Read-only pointer to the current system state (for the OLED, etc.).
 *        Valid after app_supervisor_start().
 */
const volatile app_state_t *app_supervisor_state_ptr(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SUPERVISOR_H */
