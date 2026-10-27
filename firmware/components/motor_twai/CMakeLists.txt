idf_component_register(
    SRCS "motor_twai.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_twai app_protocol
)
