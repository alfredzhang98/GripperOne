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
#define QUEUE_LEN_IMU           32  /* 320 ms headroom at the 100 Hz IMU rate */
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
 * Haptic sample: one direct-UART PaXini CP-M3025-Core module.
 *
 * The CP-M3025 has a 7 x 11 array (77 points). Each point is three bytes in
 * application register space: signed Fx, signed Fy, unsigned Fz, all at
 * 0.1 N/LSB. Keeping the point data raw preserves the whole spatial map while
 * keeping one sensor sample compact enough for the queue and app_proto.
 * ------------------------------------------------------------------------- */
#define HAPTIC_SENSOR_COUNT 2
#define HAPTIC_GRID_COLS    7
#define HAPTIC_GRID_ROWS    11
#define HAPTIC_POINT_COUNT  (HAPTIC_GRID_COLS * HAPTIC_GRID_ROWS)

/* Result of the integration board's start-up discovery for one slot. It rides
 * along in every sample so a host can tell "no force" from "nothing plugged
 * into that finger's connector", which otherwise look identical: the board
 * happily answers reads for an empty slot and returns zeros. */
typedef enum {
    HAPTIC_SLOT_OK         = 0, /* present, reports HAPTIC_POINT_COUNT points */
    HAPTIC_SLOT_EMPTY      = 1, /* absent from the board's module map         */
    HAPTIC_SLOT_WRONG_SIZE = 2, /* present but not a 7x11 module              */
    HAPTIC_SLOT_UNKNOWN    = 3, /* discovery never completed                  */
} haptic_slot_status_t;

typedef struct {
    uint32_t seq;             /* per-sensor monotonically increasing sequence */
    uint32_t timestamp_ms;    /* ESP receive time                              */
    uint8_t  sensor_index;    /* 0 = ring finger, 1 = little finger            */
    uint8_t  device_addr;     /* integration-board module slot (12=ring 16=little) */
    uint8_t  status;          /* haptic_slot_status_t for this slot              */
    uint8_t  point_count;     /* 77 for CP-M3025                                */
    float    resultant[3];    /* Fx, Fy, Fz in N from registers 1008..1010      */
    int8_t   fx[HAPTIC_POINT_COUNT]; /* raw signed value, multiply by 0.1 N */
    int8_t   fy[HAPTIC_POINT_COUNT]; /* raw signed value, multiply by 0.1 N */
    uint8_t  fz[HAPTIC_POINT_COUNT]; /* raw unsigned value, multiply by 0.1 N */
} haptic_sample_t;

/* -------------------------------------------------------------------------
 * IMU sample (JY901P). Yaw is intentionally dropped: it needs the magnetometer
 * (unreliable next to the arm/motors). We keep gravity-referenced roll/pitch
 * plus acceleration and angular velocity for vibration/impact sensing.
 * ------------------------------------------------------------------------- */
typedef struct {
    uint32_t seq;
    uint32_t timestamp_ms; /* ESP receive time; wraps naturally after ~49 d */
    float roll;         /* deg  (gravity-referenced tilt) */
    float pitch;        /* deg */
    float accel[3];     /* g     (ax, ay, az) — raw, includes gravity */
    float lin_accel[3]; /* g     2.5 s high-pass in sensor axes (~0 at rest) */
    float gyro[3];      /* deg/s (gx, gy, gz) — rotational shake */
    float temp;         /* degC  */
} imu_sample_t;

/* -------------------------------------------------------------------------
 * Motor status decoded from the DM-J4310-2EC feedback frame (same in every
 * control mode; frame ID = MST_ID).
 * ------------------------------------------------------------------------- */
typedef struct {
    uint32_t seq;
    uint32_t t_ms;       /* receive time, FreeRTOS tick in ms (ISR stamp)     */
    bool     enabled;    /* ERR field == 1 (enabled); 0 = disabled            */
    float    position;   /* rad                                               */
    float    velocity;   /* rad/s                                             */
    float    torque;     /* Nm                                                */
    float    t_mos;      /* driver MOSFET temperature, °C  (feedback D6)      */
    float    t_rotor;    /* motor coil temperature, °C     (feedback D7)      */
    uint8_t  error_code; /* 0 = healthy; 8..E = fault (see DM ERR table)      */
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
    /* 0x05 was a separate telemetry rate. Retired: telemetry now forwards
     * every sample the sensor produces, so the sensor's own output rate is the
     * only rate there is - see OP_IMU_SET_RATE. Two independent clocks meant
     * the device silently discarded most of what it measured. */
    OP_SAFE_STOP       = 0x06,  /* force SAFE_STOP (always allowed)           */
    OP_SESSION_UNLOCK  = 0x07,  /* u0 = key; required before motor actuation  */
    OP_SESSION_LOCK    = 0x08,
    OP_WIFI_SET_STA    = 0x09,  /* data = u8 ssid_len, ssid, u8 pass_len, pass;
                                 * stored in NVS, joins that network at once;
                                 * ssid_len 0 clears it (unlock)              */
    OP_WIFI_GET_INFO   = 0x0A,  /* -> Wi-Fi access point / station / client info */

    /* --- motor (0x1-) ---
     * 0x10 ENABLE, 0x12 SET_TARGET, 0x13 SET_GAINS and 0x14 SAVE_ZERO are
     * RETIRED and answer ERR_OPCODE: the gripper owns the motor in
     * force-position mode. ENABLE bypassed its limits, SET_TARGET/SET_GAINS
     * were MIT-mode commands, SAVE_ZERO moves the drive's position frame under
     * the calibration. The numbers stay reserved. */
    OP_MOTOR_ENABLE     = 0x10, /* retired                                    */
    OP_MOTOR_DISABLE    = 0x11, /* motor off now (always allowed)             */
    OP_MOTOR_SET_TARGET = 0x12, /* retired                                    */
    OP_MOTOR_SET_GAINS  = 0x13, /* retired                                    */
    OP_MOTOR_SAVE_ZERO  = 0x14, /* retired                                    */
    OP_MOTOR_CLEAR_ERR  = 0x15, /* clear drive fault + SAFE_STOP latch (unlock) */
    OP_MOTOR_GET_STATUS = 0x16, /* -> motor telemetry                        */
    OP_MOTOR_SAVE_CONFIG= 0x17, /* store CTRL_MODE 4 + TIMEOUT in the motor's
                                 * flash (unlock; motor disabled; ~10000 flash
                                 * cycles - only when the motor is new/reset) */

    /* --- IMU (0x2-) --- */
    OP_IMU_STREAM_EN    = 0x20,
    OP_IMU_STREAM_DIS   = 0x21,
    OP_IMU_GET          = 0x22, /* -> one IMU telemetry frame                */
    OP_IMU_CALIBRATE    = 0x23, /* accelerometer calibration (still & level)  */
    OP_IMU_SET_REF      = 0x25, /* set current roll/pitch as reference (zero) */
    OP_IMU_POWON_SET    = 0x27, /* u0: 0 = off, 1 = on. Power-on auto data     *
                                 * output (WIT POWONSEND 0x2D), persisted in   *
                                 * the unit.                                   */
    OP_IMU_SET_RATE     = 0x28, /* u0 = output Hz (10/20/50/100/200). Sets the *
                                 * sensor itself; telemetry follows 1:1.       *
                                 * Session-only - a reboot restores the built  *
                                 * -in rate.                                   */
    OP_IMU_ACQ_EN       = 0x29, /* resume IMU -> ESP32 frames (session only)   */
    OP_IMU_ACQ_DIS      = 0x2A, /* stop IMU -> ESP32 frames (session only)     */
    /* 0x24 and 0x26 are retired, not free. 0x24 was a yaw zero: yaw is never
     * decoded here (it needs the magnetometer, unreliable beside the motors),
     * so the command could only ever ACK a no-op. 0x26 switched the fusion
     * algorithm at runtime; the algorithm is pinned at start-up instead, and
     * changing it mid-run would swap the meaning of every angle already sent.
     * Leave the numbers unused so a future opcode cannot inherit old meaning. */

    /* --- haptic (0x3-) --- */
    OP_HAPTIC_STREAM_EN = 0x30,
    OP_HAPTIC_STREAM_DIS= 0x31,
    OP_HAPTIC_GET       = 0x32, /* -> latest frame for each tactile module   */
    /* 0x33 retired: direct CP-M3025 reads always include resultant + array. */
    OP_HAPTIC_CALIBRATE = 0x34, /* zero/tare both tactile modules             */
    OP_HAPTIC_ACQ_EN    = 0x35, /* resume CP-M3025 polling                    */
    OP_HAPTIC_ACQ_DIS   = 0x36, /* stop polling; USB stream switch unchanged */

    /* --- gripper high-level app (0x4-) --- (0x40-0x45, 0x48 need unlock) */
    OP_GRIP_CALIBRATE  = 0x40,  /* full: open stop (current) + pad touch (tactile), saved */
    OP_GRIP_HOME       = 0x41,  /* open-stop check, then the saved calibration (else full) */
    OP_GRIP_SET_CLOSE  = 0x42,  /* f0 = closing percent 5 .. 95 (0 = open
                                 * position, 100 = closed); optional f1 = speed
                                 * rad/s, 0.05 .. 15 (0 or absent = 2.0)      */
    OP_GRIP_GRIP       = 0x43,  /* f0 = target pad force N (<= 0 = default);
                                 * optional f1 = approach speed rad/s, 0.05 ..
                                 * 15 (0 or absent = 2.0). Moves
                                 * to 80 % opening, then closes; no pad force by
                                 * 95 % closed -> "empty grasp", back to 80 %  */
    /* 0x44 (GRIP_RELEASE) removed: GRIP_SET_CLOSE 5 opens as far as allowed */
    OP_GRIP_SET_FORCE  = 0x45,  /* f0 = default pad force N, persisted        */
    OP_GRIP_STOP       = 0x46,  /* hold current position (always allowed)     */
    OP_GRIP_GET_CALIB  = 0x47,  /* -> calibration + status response          */
    OP_GRIP_MOVE_DEG   = 0x48,  /* f0 = deg from the closed position, f1 = deg/s */

    /* --- internal, never accepted from a link --- */
    OP_INTERNAL_TRANSPORT_CLOSED = 0xF0,  /* a transport's peer left: relock it */
} app_op_t;

/* Transport ids, stamped into every decoded command. Replies go back to the
 * transport a command came from; telemetry goes to the transports that
 * enabled it. */
#define APP_TR_BLE    0u
#define APP_TR_WIFI   1u
#define APP_TR_USB    2u
#define APP_TR_COUNT  3u
#define APP_TR_BIT(t) ((uint8_t)(1u << (t)))

#define APP_CMD_DATA_MAX 98   /* OP_WIFI_SET_STA: 1 + 32 + 1 + 64 */

/* A decoded, validated user command handed to the supervisor via ctrl_q. */
typedef struct {
    uint8_t  seq;      /* echoed in the response                            */
    uint8_t  op;       /* app_op_t                                          */
    uint8_t  transport;/* origin: APP_TR_BLE / APP_TR_WIFI / APP_TR_USB     */
    float    f0;       /* primary float arg (target / Kp / ...)             */
    float    f1;       /* secondary float arg (Kd)                          */
    uint32_t u0;       /* integer arg (session key / rate / content bits)   */
    uint8_t  data_len; /* raw argument bytes (OP_WIFI_SET_STA)              */
    uint8_t  data[APP_CMD_DATA_MAX];
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

/**
 * @brief Tell the supervisor that a transport's peer has gone (BLE
 *        disconnect, TCP close): its session is relocked, its telemetry
 *        streams stop and a move it started stops. A grip keeps holding.
 */
void app_bus_post_transport_closed(const app_bus_t *bus, uint8_t transport);

#ifdef __cplusplus
}
#endif

#endif /* APP_PROTOCOL_H */
