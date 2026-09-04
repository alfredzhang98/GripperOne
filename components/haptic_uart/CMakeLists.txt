idf_component_register(
    SRCS "haptic_uart.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart esp_driver_gpio esp_timer app_protocol
)
