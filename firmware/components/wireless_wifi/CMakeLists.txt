idf_component_register(
    SRCS "wireless_wifi.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event esp_netif esp_timer nvs_flash lwip app_protocol
)
