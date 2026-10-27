# IMU + haptic + motor/gripper build; control over USB, BLE and Wi-Fi.
idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_uart     # UART_NUM_x constants referenced by app_config.h
        board_xiao_esp32c6  # NVS + banner
        app_protocol        # app_bus_t, sample types
        imu_uart
        haptic_uart
        motor_twai          # DM-J4310 over TWAI, force-position mode
        gripper             # calibration + tactile force grip
        app_supervisor      # command dispatch + control loop
        transport_uart      # app_proto over USB-Serial/JTAG
        wireless_ble        # app_proto over BLE GATT
        wireless_wifi       # app_proto over TCP (access point + optional station)
        esp_system          # free heap at start-up
)
