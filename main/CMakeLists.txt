idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_uart
        board_xiao_esp32c6
        app_protocol
        haptic_uart
        imu_uart
        motor_twai
        gripper
        transport_uart
        wireless_ble
        wireless_wifi
        app_supervisor
)
