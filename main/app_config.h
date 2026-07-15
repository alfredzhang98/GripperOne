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
 * User-layer transport
 * All three (UART / BLE / Wi-Fi) speak the same app_proto protocol and expose
 * the same start(bus) + app_proto sink. Pick exactly one; the others are still
 * compiled but not started. Default: UART over the native USB-Serial/JTAG.
 * ========================================================================= */
#define TRANSPORT_UART      0
#define TRANSPORT_BLE       1
#define TRANSPORT_WIFI      2
#ifndef APP_TRANSPORT
#define APP_TRANSPORT       TRANSPORT_UART
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
/* Damiao CAN IDs. ESC_ID = the motor's receive ID (host -> motor command);
 * MST_ID = the ID the motor transmits feedback on. Match these to the motor's
 * configured registers (ESC_ID reg 0x08, MST_ID reg 0x07). The RX path filters
 * strictly on MST_ID so foreign bus traffic can't be mis-read as feedback. */
#define MOTOR_TWAI_ESC_ID       0x001
#define MOTOR_TWAI_MST_ID       0x000

/* =========================================================================
 * BLE
 * ========================================================================= */
#define BLE_DEVICE_NAME         "GripperPlus"

/* =========================================================================
 * Safety limits
 * ========================================================================= */
/* Per-module tactile normal force (N) that forces an immediate SAFE_STOP. Set
 * well above any normal grip (default clamp is ~5 N); tune to the sensor range. */
#define GRIPPER_OVERFORCE_N     40.0f

/* =========================================================================
 * Task stack sizes (bytes) and priorities.
 * Higher number = higher priority in FreeRTOS.
 * These are passed into each driver via its cfg struct (see app_main.c), so
 * editing them here actually changes the created tasks.
 * ========================================================================= */
#define TASK_STACK_HAPTIC       4096
#define TASK_STACK_IMU          4096
#define TASK_STACK_MOTOR        4096
#define TASK_STACK_SUPERVISOR   4096

#define TASK_PRIO_HAPTIC        6   /* high-rate sensor ingest */
#define TASK_PRIO_IMU           6
#define TASK_PRIO_MOTOR         7   /* motor I/O is time-critical */
#define TASK_PRIO_SUPERVISOR    5   /* decision loop */

/* =========================================================================
 * Inter-task queue depths (number of messages) are owned by app_protocol.h
 * (QUEUE_LEN_*), because the queues are created there in app_bus_init() — a
 * component that does not include this file. Override them at the build level
 * (e.g. a compile definition) if you need a different depth.
 * ========================================================================= */

#endif /* APP_CONFIG_H */
