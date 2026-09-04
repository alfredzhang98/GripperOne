# IMU + haptic bring-up: motor_twai and gripper remain deliberately out.
idf_component_register(
    SRCS "app_supervisor.c"
    INCLUDE_DIRS "include"
    REQUIRES app_protocol imu_uart haptic_uart nvs_flash
)
