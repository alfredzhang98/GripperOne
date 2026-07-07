/**
 * @file app_protocol.h
 * @brief Shared data types and the inter-task "message bus".
 *
 * This component owns the vocabulary the whole firmware speaks: the sensor
 * sample structs, the control-command struct, the system state enum, and the
 * set of FreeRTOS queues that connect the driver tasks to the supervisor.
 *
 * Nothing here touches hardware, so every component can depend on it without
 * pulling in a driver. Data flow:
 *
 *     haptic_uart_task --\
 *     imu_uart_task ------+--> [queues] --> supervisor_task --> motor_twai_task
 *     ble_control_task --/
 */
#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Default bus queue depths. These are protocol-level (not hardware) so they
 * live here rather than in main/app_config.h. */
#ifndef QUEUE_LEN_HAPTIC
#define QUEUE_LEN_HAPTIC        8
#endif
#ifndef QUEUE_LEN_IMU
#define QUEUE_LEN_IMU           8
#endif
#ifndef QUEUE_LEN_MOTOR_STATUS
#define QUEUE_LEN_MOTOR_STATUS  8
#endif
#ifndef QUEUE_LEN_CTRL
#define QUEUE_LEN_CTRL          8
#endif

/* -------------------------------------------------------------------------
 * System states owned by the supervisor.
 * ------------------------------------------------------------------------- */
typedef enum {
    APP_STATE_INIT = 0,   /* bringing peripherals up */
    APP_STATE_IDLE,       /* ready, motor disabled */
    APP_STATE_ACTIVE,     /* motor enabled, tracking a target */
    APP_STATE_ERROR,      /* a subsystem reported a fault */
    APP_STATE_SAFE_STOP,  /* motor forced off, awaiting operator reset */
} app_state_t;

const char *app_state_name(app_state_t s);

/* -------------------------------------------------------------------------
 * Haptic / pressure sample (PaXini "通信盒" auto-push frame, head 0xAA 0x56).
 *
 * The driver configures the box for resultant-force output (register 0x0016 =
 * 0x01), so each connected tactile module contributes one 3-axis resultant
 * force (Fx, Fy, Fz). module_count = payload_len / 6.
 * ------------------------------------------------------------------------- */
#define HAPTIC_MAX_MODULES 28              /* 5 fingers x 4 + 8 palm pads */
typedef struct {
    uint32_t seq;                          /* monotonically increasing */
    uint8_t  error_code;                   /* total-error byte (reg 0x0023): 0 = OK */
    uint16_t module_count;                 /* valid entries in force[] */
    struct {
        float fx, fy, fz;                  /* resultant force per module, Newtons */
    } force[HAPTIC_MAX_MODULES];
} haptic_sample_t;

/* -------------------------------------------------------------------------
 * IMU sample. Either Euler angles or a quaternion, per the source device.
 * TODO: pick the representation the JY901P is configured to emit.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint32_t seq;
    float yaw;    /* deg */
    float pitch;  /* deg */
    float roll;   /* deg */
    float quat[4]; /* w,x,y,z (optional; 0 if unused) */
} imu_sample_t;

/* -------------------------------------------------------------------------
 * Motor status reported back from the TWAI bus.
 * TODO: map to the DM-J4310-2EC feedback frame fields.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint32_t seq;
    bool     enabled;
    float    position;  /* rad */
    float    velocity;  /* rad/s */
    float    torque;    /* Nm */
    uint8_t  error_code;
} motor_status_t;

/* -------------------------------------------------------------------------
 * Control commands. Sourced by the BLE (or Wi-Fi) task, consumed by the
 * supervisor which decides whether to forward them to the motor.
 * ------------------------------------------------------------------------- */
/*
 * Operation codes for the user-facing protocol. Grouped by subsystem so both
 * transports (BLE, Wi-Fi) and the supervisor speak one vocabulary. The wire
 * framing/CRC/session handling lives in app_proto.h; this enum is the shared
 * command set carried inside a decoded app_cmd_t.
 */
typedef enum {
    /* --- system (0x0-) --- */
    OP_PING            = 0x01,  /* liveness check                              */
    OP_GET_VERSION     = 0x02,  /* -> version response                        */
    OP_GET_STATE       = 0x03,  /* -> supervisor state                       */
    OP_SELF_CALIBRATE  = 0x04,  /* run IMU+haptic+motor calibration sequence  */
    OP_SET_TELEM_RATE  = 0x05,  /* u0 = telemetry Hz                          */
    OP_SAFE_STOP       = 0x06,  /* force SAFE_STOP (always allowed)           */
    OP_SESSION_UNLOCK  = 0x07,  /* u0 = key; required before motor actuation  */
    OP_SESSION_LOCK    = 0x08,

    /* --- motor (0x1-) --- (actuation requires an unlocked session) */
    OP_MOTOR_ENABLE     = 0x10,
    OP_MOTOR_DISABLE    = 0x11,
    OP_MOTOR_SET_TARGET = 0x12, /* f0 = target position (rad) / grip          */
    OP_MOTOR_SET_GAINS  = 0x13, /* f0 = Kp, f1 = Kd                           */
    OP_MOTOR_SAVE_ZERO  = 0x14, /* set current position as zero               */
    OP_MOTOR_CLEAR_ERR  = 0x15,
    OP_MOTOR_GET_STATUS = 0x16, /* -> motor telemetry                        */

    /* --- IMU (0x2-) --- */
    OP_IMU_STREAM_EN    = 0x20,
    OP_IMU_STREAM_DIS   = 0x21,
    OP_IMU_GET          = 0x22, /* -> one IMU telemetry frame                */
    OP_IMU_CALIBRATE    = 0x23, /* gyro/accel auto-calibration (自校准)       */
    OP_IMU_SET_ZERO     = 0x24, /* yaw/Z-axis angle reference to zero         */

    /* --- haptic (0x3-) --- */
    OP_HAPTIC_STREAM_EN = 0x30,
    OP_HAPTIC_STREAM_DIS= 0x31,
    OP_HAPTIC_GET       = 0x32, /* -> one haptic telemetry frame             */
    OP_HAPTIC_SET_CONTENT=0x33, /* u0 = content bits (bit0 resultant,1 dist) */
    OP_HAPTIC_CALIBRATE = 0x34, /* zero/tare the tactile modules             */

    /* --- gripper high-level app (0x4-) --- (motion; require unlock) */
    OP_GRIP_CALIBRATE  = 0x40,  /* auto-find closed+open stops, save to flash */
    OP_GRIP_HOME       = 0x41,  /* go to fully-closed zero                    */
    OP_GRIP_SET_OPEN   = 0x42,  /* f0 = opening 0.0 (closed) .. 1.0 (open)    */
    OP_GRIP_GRIP       = 0x43,  /* f0 = grip force N (0 = stored default)     */
    OP_GRIP_RELEASE    = 0x44,  /* open fully                                 */
    OP_GRIP_SET_FORCE  = 0x45,  /* f0 = default grip force N, persist         */
    OP_GRIP_STOP       = 0x46,  /* hold current position                     */
    OP_GRIP_GET_CALIB  = 0x47,  /* -> calibration response (read-only)        */
} app_op_t;

/* A decoded, validated user command handed to the supervisor via ctrl_q. */
typedef struct {
    uint8_t  seq;      /* echoed in the response                            */
    uint8_t  op;       /* app_op_t                                          */
    uint8_t  transport;/* origin, for logging (0 = BLE, 1 = Wi-Fi)          */
    float    f0;       /* primary float arg (target / Kp / ...)             */
    float    f1;       /* secondary float arg (Kd)                          */
    uint32_t u0;       /* integer arg (session key / rate / content bits)   */
} app_cmd_t;

/* -------------------------------------------------------------------------
 * The message bus: a bundle of queues created once at startup. Driver tasks
 * push samples; the supervisor drains them. Ownership is central so no
 * component has to know about any other component's internals.
 * ------------------------------------------------------------------------- */
typedef struct {
    QueueHandle_t haptic_q;        /* haptic_sample_t  */
    QueueHandle_t imu_q;           /* imu_sample_t     */
    QueueHandle_t motor_status_q;  /* motor_status_t   */
    QueueHandle_t ctrl_q;          /* app_cmd_t        */
} app_bus_t;

/**
 * @brief Create every queue in the bus.
 * @return ESP_OK on success. On failure any partially-created queues are
 *         freed and the pointers are left NULL.
 */
int app_bus_init(app_bus_t *bus);

#ifdef __cplusplus
}
#endif

#endif /* APP_PROTOCOL_H */
