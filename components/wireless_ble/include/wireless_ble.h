/**
 * @file wireless_ble.h
 * @brief BLE GATT control interface (NimBLE host).
 *
 * Advertises a single custom "Gripper Control" service and turns GATT writes
 * into ctrl_msg_t messages on bus->ctrl_q. The supervisor decides what to do
 * with them. A status characteristic is exposed for read/notify.
 *
 * The control characteristic carries the framed app_proto packets (see
 * app_proto.h) — NOT ad-hoc bytes. Writes are fed into the protocol parser;
 * decoded commands are posted to bus->ctrl_q for the supervisor. On connect,
 * BLE registers itself as the app_proto telemetry sink so responses/telemetry
 * are delivered as GATT notifications on the telemetry characteristic.
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
 * @param bus          message bus; decoded commands are posted to bus->ctrl_q
 * @param device_name  advertised name
 * @return ESP_OK on success.
 */
int wireless_ble_start(const app_bus_t *bus, const char *device_name);

#ifdef __cplusplus
}
#endif

#endif /* WIRELESS_BLE_H */
