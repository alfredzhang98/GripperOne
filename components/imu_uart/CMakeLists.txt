idf_component_register(
    SRCS "imu_uart.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart app_protocol
)
