#include "app_proto.h"

#include <string.h>
#include "esp_log.h"

static const char *TAG = "app_proto";

/* ---- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) ---------------------- */
uint16_t app_proto_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ---- little-endian helpers ---------------------------------------------- */
static inline void put_u16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static inline uint16_t get_u16(const uint8_t *p)   { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline float get_f32(const uint8_t *p) { float f; memcpy(&f, p, 4); return f; }
static inline size_t put_f32(uint8_t *p, float f) { memcpy(p, &f, 4); return 4; }

/* ---- sink ----------------------------------------------------------------
 * set_sink runs on the BLE host task; emit runs on the supervisor task (and
 * re-entrantly on the BLE task). Guard the (fn,ctx) pair with a spinlock so a
 * reader can never latch a torn fn-from-one-transport / ctx-from-another. */
static portMUX_TYPE       s_sink_mux = portMUX_INITIALIZER_UNLOCKED;
static app_proto_sink_fn  s_sink = NULL;
static void              *s_sink_ctx = NULL;

void app_proto_set_sink(app_proto_sink_fn fn, void *ctx)
{
    portENTER_CRITICAL(&s_sink_mux);
    s_sink = fn;
    s_sink_ctx = ctx;
    portEXIT_CRITICAL(&s_sink_mux);
}

void app_proto_emit(const uint8_t *frame, size_t len)
{
    portENTER_CRITICAL(&s_sink_mux);
    app_proto_sink_fn fn = s_sink;
    void *ctx = s_sink_ctx;
    portEXIT_CRITICAL(&s_sink_mux);
    if (fn && frame && len) {
        fn(frame, len, ctx);   /* call outside the critical section */
    }
}

/* ---- frame builder ------------------------------------------------------ */
size_t app_proto_build(uint8_t *out, size_t cap, uint8_t type, uint8_t seq,
                       const uint8_t *payload, uint16_t plen)
{
    size_t total = APP_PROTO_HDR_LEN + plen + 2;
    if (plen > APP_PROTO_MAX_PAYLOAD || total > cap) {
        return 0;
    }
    out[0] = APP_PROTO_SOF0;
    out[1] = APP_PROTO_SOF1;
    out[2] = APP_PROTO_VERSION;
    out[3] = type;
    out[4] = seq;
    put_u16(&out[5], plen);
    if (plen && payload) {
        memcpy(&out[APP_PROTO_HDR_LEN], payload, plen);
    }
    uint16_t crc = app_proto_crc16(&out[2], (size_t)(APP_PROTO_HDR_LEN - 2) + plen);
    put_u16(&out[total - 2], crc);
    return total;
}

size_t app_proto_ack(uint8_t *out, size_t cap, uint8_t seq, uint8_t op, uint8_t status)
{
    uint8_t pl[2] = { op, status };
    return app_proto_build(out, cap, APP_PROTO_T_ACK, seq, pl, sizeof(pl));
}

size_t app_proto_resp_state(uint8_t *out, size_t cap, uint8_t seq, uint8_t state)
{
    uint8_t pl[2] = { OP_GET_STATE, state };
    return app_proto_build(out, cap, APP_PROTO_T_RESPONSE, seq, pl, sizeof(pl));
}

size_t app_proto_tele_imu(uint8_t *out, size_t cap, const imu_sample_t *s)
{
    uint8_t pl[1 + 7 * 4];
    size_t k = 0;
    pl[k++] = OP_IMU_GET;
    k += put_f32(&pl[k], s->yaw);
    k += put_f32(&pl[k], s->pitch);
    k += put_f32(&pl[k], s->roll);
    for (int i = 0; i < 4; i++) k += put_f32(&pl[k], s->quat[i]);
    return app_proto_build(out, cap, APP_PROTO_T_TELEMETRY, 0, pl, (uint16_t)k);
}

size_t app_proto_tele_haptic(uint8_t *out, size_t cap, const haptic_sample_t *s)
{
    uint8_t pl[1 + 2 + HAPTIC_MAX_MODULES * 3 * 4];
    size_t k = 0;
    uint16_t n = s->module_count > HAPTIC_MAX_MODULES ? HAPTIC_MAX_MODULES : s->module_count;
    pl[k++] = OP_HAPTIC_GET;
    pl[k++] = s->error_code;
    pl[k++] = (uint8_t)n;
    for (uint16_t i = 0; i < n; i++) {
        k += put_f32(&pl[k], s->force[i].fx);
        k += put_f32(&pl[k], s->force[i].fy);
        k += put_f32(&pl[k], s->force[i].fz);
    }
    return app_proto_build(out, cap, APP_PROTO_T_TELEMETRY, 0, pl, (uint16_t)k);
}

size_t app_proto_tele_motor(uint8_t *out, size_t cap, const motor_status_t *s)
{
    uint8_t pl[1 + 1 + 3 * 4 + 1];
    size_t k = 0;
    pl[k++] = OP_MOTOR_GET_STATUS;
    pl[k++] = s->enabled ? 1 : 0;
    k += put_f32(&pl[k], s->position);
    k += put_f32(&pl[k], s->velocity);
    k += put_f32(&pl[k], s->torque);
    pl[k++] = s->error_code;
    return app_proto_build(out, cap, APP_PROTO_T_TELEMETRY, 0, pl, (uint16_t)k);
}

/* ---- decode one payload into an app_cmd_t ------------------------------- */
static void decode_args(app_cmd_t *cmd, const uint8_t *pl, uint16_t plen)
{
    /* pl[0] is the opcode (already stored). Args start at pl[1]. */
    switch (cmd->op) {
    case OP_MOTOR_SET_TARGET:
    case OP_GRIP_SET_OPEN:
    case OP_GRIP_GRIP:
    case OP_GRIP_SET_FORCE:
        if (plen >= 1 + 4) cmd->f0 = get_f32(&pl[1]);
        break;
    case OP_MOTOR_SET_GAINS:
        if (plen >= 1 + 8) { cmd->f0 = get_f32(&pl[1]); cmd->f1 = get_f32(&pl[5]); }
        break;
    case OP_SESSION_UNLOCK:
        if (plen >= 1 + 4) cmd->u0 = get_u32(&pl[1]);
        break;
    case OP_SET_TELEM_RATE:
    case OP_HAPTIC_SET_CONTENT:
        if (plen >= 1 + 2) cmd->u0 = get_u16(&pl[1]);
        break;
    default:
        break;   /* opcode-only commands */
    }
}

/* ---- streaming parser --------------------------------------------------- */
static void emit_error(uint8_t seq, uint8_t op, uint8_t status)
{
    uint8_t out[APP_PROTO_HDR_LEN + 2 + 2];
    uint8_t pl[2] = { op, status };
    size_t n = app_proto_build(out, sizeof(out), APP_PROTO_T_ERROR, seq, pl, sizeof(pl));
    if (n) app_proto_emit(out, n);
}

void app_proto_feed(app_proto_parser_t *p, uint8_t transport,
                    const uint8_t *in, size_t n,
                    app_proto_cmd_fn cb, void *ctx)
{
    for (size_t i = 0; i < n; i++) {
        if (p->len < sizeof(p->buf)) {
            p->buf[p->len++] = in[i];
        } else {
            /* overflow: drop the oldest byte to keep scanning */
            memmove(p->buf, p->buf + 1, p->len - 1);
            p->buf[p->len - 1] = in[i];
        }

        /* try to parse from the front, resyncing as needed */
        for (;;) {
            /* discard until a plausible SOF0 */
            size_t s = 0;
            while (s < p->len && p->buf[s] != APP_PROTO_SOF0) s++;
            if (s > 0) {
                memmove(p->buf, p->buf + s, p->len - s);
                p->len -= s;
            }
            if (p->len < APP_PROTO_HDR_LEN) break;         /* need full header */
            if (p->buf[1] != APP_PROTO_SOF1) {             /* bad second SOF */
                memmove(p->buf, p->buf + 1, --p->len);
                continue;
            }
            uint16_t plen = get_u16(&p->buf[5]);
            if (plen > APP_PROTO_MAX_PAYLOAD) {            /* implausible */
                memmove(p->buf, p->buf + 1, --p->len);
                continue;
            }
            size_t frame_len = APP_PROTO_HDR_LEN + plen + 2;
            if (p->len < frame_len) break;                 /* wait for more */

            uint16_t rx_crc = get_u16(&p->buf[frame_len - 2]);
            uint16_t calc = app_proto_crc16(&p->buf[2], (size_t)(APP_PROTO_HDR_LEN - 2) + plen);
            uint8_t  ver = p->buf[2];
            uint8_t  seq = p->buf[4];
            uint8_t  op  = plen >= 1 ? p->buf[APP_PROTO_HDR_LEN] : 0;

            if (calc != rx_crc) {
                /* LEN lives inside the CRC-covered region, so a failed CRC
                 * means we cannot trust frame_len. Resync one byte instead of
                 * consuming a possibly-bogus length (which could eat the next
                 * good frame). */
                ESP_LOGW(TAG, "CRC fail (op=0x%02x), resync", op);
                emit_error(seq, op, APP_PROTO_ERR_CRC);
                memmove(p->buf, p->buf + 1, --p->len);
                continue;
            }
            if (ver != APP_PROTO_VERSION) {
                emit_error(seq, op, APP_PROTO_ERR_VERSION);
            } else if (plen < 1) {
                emit_error(seq, op, APP_PROTO_ERR_OPCODE);
            } else {
                app_cmd_t cmd = { .seq = seq, .op = op, .transport = transport };
                decode_args(&cmd, &p->buf[APP_PROTO_HDR_LEN], plen);
                if (cb) cb(&cmd, ctx);
            }
            /* CRC valid => header (incl. LEN) is trusted => consume whole frame */
            memmove(p->buf, p->buf + frame_len, p->len - frame_len);
            p->len -= frame_len;
        }
    }
}
