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
#define TRANSPORT_UART 0
#define TRANSPORT_BLE 1
#define TRANSPORT_WIFI 2
#ifndef APP_TRANSPORT
#define APP_TRANSPORT TRANSPORT_UART
#endif

/* =========================================================================
 * UART links
 * The XIAO ESP32-C6 has two hardware UART controllers (UART0, UART1). The
 * console runs on the USB Serial/JTAG peripheral, so both are free here.
 * ========================================================================= */
#define HAPTIC_UART_PORT UART_NUM_1
#define HAPTIC_UART_BAUD 921600 /* PX-6AX GEN3 manual: fixed 921600, 8N1 */
/* CP-M3025-Core is rated at 83.3 complete samples/s. Each poll is two reads
 * (6-byte resultant + 231-byte distributed map), about 3 ms per module at
 * 921600 baud, so 80 Hz for both fits comfortably.
 *
 * Module indices are slots in the integration board's fixed 28-module order
 * (通信盒通信协议 section 4.6): thumb 0-3, index 4-7, middle 8-11, ring 12-15,
 * little 16-19, palm 20-27, each finger as proximal/middle/fingertip/nail.
 * A CP-M3025 is the proximal ("近节") module, so ring = 12, little = 16. */
#define HAPTIC_POLL_HZ 80
#define HAPTIC_RING_MODULE 12
#define HAPTIC_LITTLE_MODULE 16

#define IMU_UART_PORT UART_NUM_0

/* The two IMU knobs worth touching. They interact: every output period costs
 * three 11-byte frames (accel + gyro + angle), so
 *
 *     bytes/s = 3 x 11 x IMU_OUT_RATE_HZ      link capacity = baud / 10
 *
 * imu_check_bandwidth() prints both figures at boot and warns past 80%, so
 * raise the baud whenever you raise the rate. Current pair:
 *
 * Detection happens before RRATE is changed, so the unit's saved low-rate
 * stream is enough to identify a factory-9600 device. After migration:
 *
 *     115200 + 100 Hz -> 3300 B/s of 11520 (29%)
 *
 * 230400 is the ceiling for THIS part. The WIT table also lists 460800 and
 * 921600, but wit_reg.h marks both "WT931/JY931/HWT606/HWT906 only" - a JY901P
 * ignores those codes, so asking for one breaks the link instead of speeding
 * it up. imu_apply_baud() now refuses them outright.
 *
 * Startup checks only these two known states, never the full baud table. If a
 * valid WIT frame is found at 9600, the driver changes the module to 115200 for
 * this powered session and verifies the new link. If the ESP resets while the
 * IMU remains powered, it can also recognise that it is already at 115200. */
#define IMU_UART_BOOT_BAUD 9600   /* WIT factory/project setting */
#define IMU_UART_RUN_BAUD 115200  /* use after a valid 9600 link */
#define IMU_OUT_RATE_HZ 100       /* sensor -> ESP32 sample rate */

/* RX ring-buffer / read-chunk sizing shared by the UART drivers. */
#define UART_RX_BUFFER_SIZE 2048
#define UART_TX_BUFFER_SIZE 512
#define UART_READ_CHUNK 256

/* =========================================================================
 * TWAI (CAN) motor bus
 * ========================================================================= */
#define MOTOR_TWAI_BITRATE_KBPS 1000 /* DM-J4310 defaults to 1 Mbit/s. TODO: confirm */
/* Damiao CAN IDs. ESC_ID = the motor's receive ID (host -> motor command);
 * MST_ID = the ID the motor transmits feedback on. Match these to the motor's
 * configured registers (ESC_ID reg 0x08, MST_ID reg 0x07). The RX path filters
 * strictly on MST_ID so foreign bus traffic can't be mis-read as feedback. */
#define MOTOR_TWAI_ESC_ID 0x001
#define MOTOR_TWAI_MST_ID 0x000
/* Comms-loss timeout written to the motor (register 0x09) at every probe: if
 * no command frame arrives for this long while enabled, the drive disables
 * itself. Commands go out every 10 ms; 300 ms also covers an NVS flash write
 * stalling the CPU. Not saved to the motor's flash. */
#define MOTOR_TWAI_TIMEOUT_MS 300

/* =========================================================================
 * Gripper
 * ========================================================================= */
/* 1 = run GRIP_HOME once after boot as soon as the motor and both tactile
 * pads are ready: with a calibration stored in NVS, the jaw opens to its stop,
 * checks it against the stored one and restores the calibration (about 10 s);
 * without one, a full calibration (open stop, then pad touch).
 * 0 = no motion until the host sends GRIP_HOME (0x41) or GRIP_CALIBRATE (0x40). */
#define GRIPPER_AUTOCAL_ON_BOOT 0  /* bench bring-up: start by command */

/* =========================================================================
 * BLE
 * ========================================================================= */
#define BLE_DEVICE_NAME "GripperPlus"

/* =========================================================================
 * Safety limits
 * ========================================================================= */
/* Pad normal force (N) that forces SAFE_STOP (motor off) while the motor is
 * enabled. Last resort only: the gripper itself cuts current above its target
 * and backs off at 20 N. CP-M3025 range is 0-25 N, so this must stay below
 * 25 N to be detectable at all. */
#define GRIPPER_OVERFORCE_N 24.0f

/* =========================================================================
 * Task stack sizes (bytes) and priorities.
 * Higher number = higher priority in FreeRTOS.
 * These are passed into each driver via its cfg struct (see app_main.c), so
 * editing them here actually changes the created tasks.
 * ========================================================================= */
#define TASK_STACK_HAPTIC 4096
#define TASK_STACK_IMU 4096
#define TASK_STACK_MOTOR 4096
#define TASK_STACK_SUPERVISOR 6144 /* gripper_tick + formatted logs */

#define TASK_PRIO_HAPTIC 6 /* high-rate sensor ingest */
#define TASK_PRIO_IMU 6
#define TASK_PRIO_MOTOR 7      /* motor I/O is time-critical */
#define TASK_PRIO_SUPERVISOR 5 /* decision loop */

/* =========================================================================
 * Inter-task queue depths (number of messages) are owned by app_protocol.h
 * (QUEUE_LEN_*), because the queues are created there in app_bus_init() — a
 * component that does not include this file. Override them at the build level
 * (e.g. a compile definition) if you need a different depth.
 * ========================================================================= */

#endif /* APP_CONFIG_H */
