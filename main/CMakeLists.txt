# IMU + haptic bring-up build. Motor/gripper/wireless remain isolated.
#
# motor_twai, gripper, wireless_ble and wireless_wifi are
# deliberately absent from main's dependency graph and are never started.
# ESP-IDF can still compile auto-discovered components into libraries, but
# unreferenced libraries add no tasks or runtime output. Their source is
# untouched under components/; restore a dependency together with its start
# call and supervisor cases.
idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_uart     # UART_NUM_x constants referenced by app_config.h
        board_xiao_esp32c6  # NVS + banner
        app_protocol        # app_bus_t, imu_sample_t
        imu_uart
        haptic_uart
        app_supervisor      # command dispatch + imu_q consumer
        transport_uart      # app_proto over USB-Serial/JTAG
)
