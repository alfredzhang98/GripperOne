/**
 * @file wireless_ble.h
 * @brief BLE GATT control interface (NimBLE host).
 *
 * Advertises a single custom "Gripper Control" service and turns GATT writes
 * into ctrl_msg_t messages on bus->ctrl_q. The supervisor decides what to do
 * with them. A status characteristic is exposed for read/notify.
 *
 * Control characteristic write payload (little-endian):
 *   byte 0      : command id (see ctrl_cmd_id_t: 1=enable, 2=disable,
 *                 3=set_target, 4=request_status)
 *   bytes 1..4  : float32 target   (only meaningful for set_target)
 */
#ifndef WIRELESS_BLE_H
#define WIRELESS_BLE_H

#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the NimBLE host, register the GATT service and begin
 *        advertising. Runs the host on its own task (ble_control_task).
 * @param bus          message bus; control writes are posted to bus->ctrl_q
 * @param device_name  advertised name
 * @return ESP_OK on success.
 */
int wireless_ble_start(const app_bus_t *bus, const char *device_name);

/**
 * @brief Publish the latest status so a connected central can read/get
 *        notified. Safe to call from the supervisor task.
 */
void wireless_ble_publish_status(const motor_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* WIRELESS_BLE_H */
