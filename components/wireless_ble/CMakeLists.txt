idf_component_register(
    SRCS "wireless_ble.c"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash app_protocol
)
