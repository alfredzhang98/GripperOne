idf_component_register(
    SRCS "gripper.c"
    INCLUDE_DIRS "include"
    REQUIRES app_protocol motor_twai nvs_flash
)
