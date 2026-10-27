idf_component_register(
    SRCS "app_supervisor.c"
    INCLUDE_DIRS "include"
    REQUIRES app_protocol imu_uart haptic_uart motor_twai gripper nvs_flash esp_timer wireless_wifi
)
