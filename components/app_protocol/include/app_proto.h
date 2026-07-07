/**
 * @file app_proto.h
 * @brief User-facing wireless control/telemetry protocol (BLE + Wi-Fi share it).
 *
 * The transports (wireless_ble, wireless_wifi) do NOT interpret bytes. They
 * feed the raw stream into app_proto_feed(); this module validates a framed,
 * versioned, CRC-checked packet and produces a decoded app_cmd_t. That command
 * goes onto bus->ctrl_q and the supervisor is the single place that executes it
 * (auto-enabling the relevant subsystem). Replies/telemetry are built here and
 * pushed back through whichever transport is currently the "sink".
 *
 * Wire frame (little-endian):
 *
 *   +------+------+-----+------+-----+--------+-----------+---------+
 *   | 0xA5 | 0x5A | VER | TYPE | SEQ | LEN(2) | PAYLOAD[] | CRC16(2)|
 *   +------+------+-----+------+-----+--------+-----------+---------+
 *      0      1     2     3      4     5..6     7..7+LEN-1  last two
 *
 *   VER    = APP_PROTO_VERSION
 *   TYPE   = app_proto_type_t (command / response / telemetry / ack / error)
 *   SEQ    = sequence number (echoed in the response)
 *   LEN    = payload length (uint16 LE); payload[0] is the opcode (app_op_t)
 *   CRC16  = CRC-16/CCITT-FALSE over bytes [2 .. 7+LEN-1] (VER..payload)
 *
 * This is a structured, validated channel — a stray/plaintext byte cannot move
 * the motor because it will fail the SOF/version/CRC check, and motor actuation
 * additionally requires a prior OP_SESSION_UNLOCK with APP_PROTO_SESSION_KEY.
 * (For real confidentiality/authentication, pair this with BLE bonding or a
 * per-session token — see README.)
 */
#ifndef APP_PROTO_H
#define APP_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_PROTO_SOF0        0xA5
#define APP_PROTO_SOF1        0x5A
#define APP_PROTO_VERSION     0x01
#define APP_PROTO_HDR_LEN     7      /* SOF0..LEN (before payload) */
#define APP_PROTO_MAX_PAYLOAD 400    /* fits 28-module haptic telemetry */
#define APP_PROTO_MAX_FRAME   (APP_PROTO_HDR_LEN + APP_PROTO_MAX_PAYLOAD + 2)

/* Session key gating motor actuation. NOTE: a shared constant is obfuscation,
 * not security — it stops casual/plaintext control, not a determined attacker.
 * Replace with BLE bonding / a provisioned token for real deployments. */
#define APP_PROTO_SESSION_KEY 0x47521000u  /* 'GR' 0x1000 */

typedef enum {
    APP_PROTO_T_COMMAND   = 0x00,  /* host -> device */
    APP_PROTO_T_RESPONSE  = 0x01,  /* device -> host, answer to a command */
    APP_PROTO_T_TELEMETRY = 0x02,  /* device -> host, unsolicited stream */
    APP_PROTO_T_ACK       = 0x03,  /* device -> host, command accepted/rejected */
    APP_PROTO_T_ERROR     = 0x04,  /* device -> host, malformed/denied */
} app_proto_type_t;

/* ACK / error status codes (payload byte after the echoed opcode). */
typedef enum {
    APP_PROTO_OK           = 0x00,
    APP_PROTO_ERR_CRC      = 0x01,
    APP_PROTO_ERR_VERSION  = 0x02,
    APP_PROTO_ERR_OPCODE   = 0x03,
    APP_PROTO_ERR_LOCKED   = 0x04,  /* motor command without a session unlock */
    APP_PROTO_ERR_STATE    = 0x05,  /* not valid in the current supervisor state */
    APP_PROTO_ERR_ARG      = 0x06,
} app_proto_status_t;

/* ---- CRC ---------------------------------------------------------------- */
uint16_t app_proto_crc16(const uint8_t *data, size_t len);

/* ---- Sink: where responses/telemetry go (set by the active transport) --- */
typedef void (*app_proto_sink_fn)(const uint8_t *frame, size_t len, void *ctx);

/** Register the current output transport (BLE notify / Wi-Fi send). Passing
 *  NULL clears it (e.g. on disconnect). */
void app_proto_set_sink(app_proto_sink_fn fn, void *ctx);

/** Send a pre-built frame through the current sink (no-op if none). */
void app_proto_emit(const uint8_t *frame, size_t len);

/* ---- Streaming parser --------------------------------------------------- */
typedef struct {
    uint8_t buf[APP_PROTO_MAX_FRAME];
    size_t  len;
} app_proto_parser_t;

/** Callback invoked once per fully-decoded, CRC-valid command. */
typedef void (*app_proto_cmd_fn)(const app_cmd_t *cmd, void *ctx);

/**
 * Feed received bytes. Complete valid commands invoke @p cb. Malformed frames
 * are dropped (and an ERROR frame is emitted to the sink for CRC/version
 * failures). @p transport is stamped into each decoded command.
 */
void app_proto_feed(app_proto_parser_t *p, uint8_t transport,
                    const uint8_t *in, size_t n,
                    app_proto_cmd_fn cb, void *ctx);

/* ---- Frame builders (return total frame length, 0 on overflow) ---------- */
size_t app_proto_build(uint8_t *out, size_t cap, uint8_t type, uint8_t seq,
                       const uint8_t *payload, uint16_t plen);
size_t app_proto_ack(uint8_t *out, size_t cap, uint8_t seq, uint8_t op, uint8_t status);
size_t app_proto_resp_state(uint8_t *out, size_t cap, uint8_t seq, uint8_t state);
size_t app_proto_tele_imu(uint8_t *out, size_t cap, const imu_sample_t *s);
size_t app_proto_tele_haptic(uint8_t *out, size_t cap, const haptic_sample_t *s);
size_t app_proto_tele_motor(uint8_t *out, size_t cap, const motor_status_t *s);

#ifdef __cplusplus
}
#endif

#endif /* APP_PROTO_H */
