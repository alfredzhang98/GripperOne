#include "wireless_wifi.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "wireless_wifi";

int wireless_wifi_start(const app_bus_t *bus)
{
    (void)bus;

    /* TODO: implement the Wi-Fi control path. A typical bring-up is:
     *   1. esp_netif_init(); esp_event_loop_create_default();
     *   2. esp_netif_create_default_wifi_sta();
     *   3. esp_wifi_init(&cfg); set WIFI_MODE_STA; set SSID/pass; esp_wifi_start();
     *   4. On IP_EVENT_STA_GOT_IP, start a TCP/UDP or HTTP control server that
     *      translates network messages into ctrl_msg_t on bus->ctrl_q
     *      (mirroring wireless_ble).
     *
     * Kept as a logged no-op so the component compiles and the feature flag
     * (APP_ENABLE_WIFI) can be flipped on later without restructuring.
     */
    ESP_LOGW(TAG, "Wi-Fi control path is a skeleton and not yet implemented");
    return ESP_OK;
}
