idf_component_register(
    SRCS "oled_sh1106.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_i2c app_protocol
)
