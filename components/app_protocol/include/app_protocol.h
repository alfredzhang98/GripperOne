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
typedef enum {
    CTRL_CMD_ENABLE_MOTOR = 1,
    CTRL_CMD_DISABLE_MOTOR,
    CTRL_CMD_SET_TARGET,
    CTRL_CMD_REQUEST_STATUS,
} ctrl_cmd_id_t;

typedef struct {
    ctrl_cmd_id_t id;
    float         target;  /* meaning depends on control mode (e.g. grip position) */
} ctrl_msg_t;

/* -------------------------------------------------------------------------
 * The message bus: a bundle of queues created once at startup. Driver tasks
 * push samples; the supervisor drains them. Ownership is central so no
 * component has to know about any other component's internals.
 * ------------------------------------------------------------------------- */
typedef struct {
    QueueHandle_t haptic_q;        /* haptic_sample_t  */
    QueueHandle_t imu_q;           /* imu_sample_t     */
    QueueHandle_t motor_status_q;  /* motor_status_t   */
    QueueHandle_t ctrl_q;          /* ctrl_msg_t       */
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
