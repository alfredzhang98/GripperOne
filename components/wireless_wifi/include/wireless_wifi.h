/**
 * @file wireless_wifi.h
 * @brief Optional Wi-Fi control channel (skeleton).
 *
 * Disabled by default (APP_ENABLE_WIFI == 0 in main/app_config.h). This is a
 * placeholder so the architecture reserves a second wireless path (e.g. a
 * TCP/UDP or HTTP control endpoint) alongside BLE. It does not connect to an
 * AP yet — see TODO markers in wireless_wifi.c.
 */
#ifndef WIRELESS_WIFI_H
#define WIRELESS_WIFI_H

#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bring up Wi-Fi (station mode) and any control server.
 * @return ESP_OK on success. Currently a logged no-op skeleton.
 */
int wireless_wifi_start(const app_bus_t *bus);

#ifdef __cplusplus
}
#endif

#endif /* WIRELESS_WIFI_H */
