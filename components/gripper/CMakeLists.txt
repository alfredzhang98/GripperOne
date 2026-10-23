idf_component_register(
    SRCS "gripper.c"
    INCLUDE_DIRS "include"
    REQUIRES app_protocol motor_twai haptic_uart nvs_flash
)
