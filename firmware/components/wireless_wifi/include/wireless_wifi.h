/**
 * @file wireless_wifi.h
 * @brief Wi-Fi control channel: app_proto frames over one TCP connection.
 *
 * The gripper always runs its own access point (SSID "<prefix>-XXXX", XXXX
 * from the MAC, address 192.168.4.1). If station credentials are stored (set
 * with OP_WIFI_SET_STA over USB), it also joins that network and serves the
 * same TCP port on the address the router gives it. One TCP client at a
 * time: a new connection replaces the old one. Frames are the same app_proto
 * frames as on USB and BLE.
 */
#ifndef WIRELESS_WIFI_H
#define WIRELESS_WIFI_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *ap_ssid_prefix;   /* "GripperOne" -> "GripperOne-1A2B"        */
    const char *ap_password;      /* WPA2, at least 8 characters                */
    uint8_t     ap_channel;       /* 1..13                                      */
    uint16_t    tcp_port;         /* app_proto server port                      */
} wireless_wifi_cfg_t;

typedef struct {
    bool     started;
    char     ap_ssid[33];
    uint32_t ap_ip;              /* network byte order, as lwIP stores it       */
    char     sta_ssid[33];       /* empty: no network stored                    */
    uint8_t  sta_state;          /* 0 none, 1 connecting, 2 connected           */
    uint32_t sta_ip;
    bool     client;             /* a TCP client is connected                   */
    uint32_t client_ip;
} wireless_wifi_info_t;

/** Bring up the access point (and the station if credentials are stored) and
 *  the TCP server. */
esp_err_t wireless_wifi_start(const app_bus_t *bus, const wireless_wifi_cfg_t *cfg);

/** Store station credentials in NVS and join that network now. An empty
 *  @p ssid forgets the network (the access point stays). */
esp_err_t wireless_wifi_set_sta(const char *ssid, const char *password);

/** Current state; returns false (and started = false) if Wi-Fi is not running. */
bool wireless_wifi_get_info(wireless_wifi_info_t *out);

#ifdef __cplusplus
}
#endif

#endif /* WIRELESS_WIFI_H */
