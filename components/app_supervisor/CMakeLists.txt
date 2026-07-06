idf_component_register(
    SRCS "app_supervisor.c"
    INCLUDE_DIRS "include"
    REQUIRES app_protocol motor_twai wireless_ble
)
