idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_uart
        esp_driver_i2c
        board_xiao_esp32c6
        app_protocol
        haptic_uart
        imu_uart
        motor_twai
        oled_sh1106
        wireless_ble
        wireless_wifi
        app_supervisor
)
