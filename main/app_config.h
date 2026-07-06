/**
 * @file app_config.h
 * @brief Firmware-wide tunables: baud rates, UART/TWAI ports, task sizing
 *        and compile-time feature flags.
 *
 * Anything a person might reasonably want to change without touching driver
 * logic lives here. Pin numbers live in app_pins.h.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* =========================================================================
 * Compile-time feature flags
 * Set to 1 to compile-in and start the feature, 0 to leave it out.
 * ========================================================================= */
#ifndef APP_ENABLE_OLED
#define APP_ENABLE_OLED     1   /* Optional SH1106 OLED display + oled_task */
#endif

#ifndef APP_ENABLE_BLE
#define APP_ENABLE_BLE      1   /* NimBLE GATT control interface */
#endif

#ifndef APP_ENABLE_WIFI
#define APP_ENABLE_WIFI     0   /* Optional Wi-Fi control channel (skeleton) */
#endif

/* =========================================================================
 * UART links
 * The XIAO ESP32-C6 has two hardware UART controllers (UART0, UART1). The
 * console runs on the USB Serial/JTAG peripheral, so both are free here.
 * ========================================================================= */
#define HAPTIC_UART_PORT        UART_NUM_1
#define HAPTIC_UART_BAUD        921600      /* Per the vendor tool (Read_Single_Sensor_Hand.py) */

#define IMU_UART_PORT           UART_NUM_0
#define IMU_UART_BAUD           9600        /* JY901P factory default. TODO: confirm for your unit */

/* RX ring-buffer / read-chunk sizing shared by the UART drivers. */
#define UART_RX_BUFFER_SIZE     2048
#define UART_TX_BUFFER_SIZE     512
#define UART_READ_CHUNK         256

/* =========================================================================
 * TWAI (CAN) motor bus
 * ========================================================================= */
#define MOTOR_TWAI_BITRATE_KBPS 1000        /* DM-J4310 defaults to 1 Mbit/s. TODO: confirm */

/* =========================================================================
 * OLED (SH1106) I2C
 * ========================================================================= */
#define OLED_I2C_PORT           I2C_NUM_0
#define OLED_I2C_FREQ_HZ        400000
#define OLED_I2C_ADDR           0x3C        /* Common SH1106 address; 0x3D on some boards */

/* =========================================================================
 * BLE
 * ========================================================================= */
#define BLE_DEVICE_NAME         "GripperPlus"

/* =========================================================================
 * Task stack sizes (bytes) and priorities.
 * Higher number = higher priority in FreeRTOS.
 * ========================================================================= */
#define TASK_STACK_HAPTIC       4096
#define TASK_STACK_IMU          4096
#define TASK_STACK_MOTOR        4096
#define TASK_STACK_OLED         3072
#define TASK_STACK_SUPERVISOR   4096

#define TASK_PRIO_HAPTIC        6   /* high-rate sensor ingest */
#define TASK_PRIO_IMU           6
#define TASK_PRIO_MOTOR         7   /* motor I/O is time-critical */
#define TASK_PRIO_SUPERVISOR    5   /* decision loop */
#define TASK_PRIO_OLED          3   /* cosmetic, lowest */

/* =========================================================================
 * Inter-task queue depths (number of messages).
 * ========================================================================= */
#define QUEUE_LEN_HAPTIC        8
#define QUEUE_LEN_IMU           8
#define QUEUE_LEN_MOTOR_STATUS  8
#define QUEUE_LEN_CTRL          8

#endif /* APP_CONFIG_H */
