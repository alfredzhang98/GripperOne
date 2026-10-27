idf_component_register(
    SRCS "transport_uart.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_usb_serial_jtag app_protocol
)
