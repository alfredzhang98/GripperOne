#include "motor_twai.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "motor_twai";

/* Damiao DM-J4310-2EC CAN framing (CAN 2.0A standard frame, 11-bit ID).
 *
 * ESC_ID (reg 0x08) = motor's receive ID, factory default 0x01.
 * MST_ID (reg 0x07) = feedback ID the motor transmits on, factory default 0x00.
 *
 * Command-frame ID depends on the control mode:
 *   MIT (mode 1)          -> ESC_ID
 *   position-velocity (2) -> 0x100 + ESC_ID
 *   velocity (3)          -> 0x200 + ESC_ID
 *   force-position (4)    -> 0x300 + ESC_ID
 * The feedback frame (ID = MST_ID) is identical in every mode.
 *
 * These are the built-in defaults; the actual IDs come from motor_twai_cfg_t
 * (set in main from app_config.h) and can be overridden per build. */
#define MOTOR_ESC_ID_DEFAULT    0x001   /* host -> motor: command frame ID     */
#define MOTOR_MST_ID_DEFAULT    0x000   /* motor -> host: feedback frame ID    */

/* Command-frame ID offsets per control mode (added to ESC_ID). */
#define DM_ID_OFFSET_POS_VEL    0x100
#define DM_ID_OFFSET_VELOCITY   0x200
#define DM_ID_OFFSET_FORCE_POS  0x300

/* Config frames (read/write/save registers) are sent to this fixed ID. */
#define DM_PARAM_FRAME_ID       0x7FF
#define DM_PARAM_CMD_READ       0x33
#define DM_PARAM_CMD_WRITE      0x55
#define DM_PARAM_CMD_SAVE       0xAA

/* MIT-mode value ranges (register defaults PMAX/VMAX/TMAX for the 4310). Keep
 * these in sync with the motor's PMAX(0x15)/VMAX(0x16)/TMAX(0x17) registers.
 * They also govern the fixed-point feedback decode, which is mode-independent. */
#define DM_P_MIN   (-12.5f)
#define DM_P_MAX   ( 12.5f)
#define DM_V_MIN   (-30.0f)
#define DM_V_MAX   ( 30.0f)
#define DM_T_MIN   (-10.0f)
#define DM_T_MAX   ( 10.0f)
#define DM_KP_MIN  ( 0.0f)
#define DM_KP_MAX  ( 500.0f)
#define DM_KD_MIN  ( 0.0f)
#define DM_KD_MAX  ( 5.0f)

/* Force-position mode scaling (manual: v_des x100 uint16, i_des x10000 uint16,
 * both clamped 0..10000). */
#define DM_FP_VEL_SCALE    100.0f
#define DM_FP_VEL_MAX_RAW  10000u    /* -> 100 rad/s */
#define DM_FP_CUR_SCALE    10000.0f
#define DM_FP_CUR_MAX_RAW  10000u    /* -> 1.0 pu   */

/* Damiao well-known 8-byte control frames (sent to CAN ID = ESC_ID). */
static const uint8_t DM_FRAME_ENABLE[8]    = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const uint8_t DM_FRAME_DISABLE[8]   = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const uint8_t DM_FRAME_SAVE_ZERO[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
static const uint8_t DM_FRAME_CLEAR_ERR[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB};

/* Standard MIT linear encode/decode (Ben Katz style). */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) * (float)((1u << bits) - 1) / span);
}

static float uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return (float)x * span / (float)((1u << bits) - 1) + x_min;
}

static twai_node_handle_t s_node = NULL;
static const app_bus_t   *s_bus  = NULL;

/* Runtime CAN IDs (overridable via cfg; default to the Damiao factory values). */
static uint32_t s_esc_id = MOTOR_ESC_ID_DEFAULT;
static uint32_t s_mst_id = MOTOR_MST_ID_DEFAULT;
static int      s_bitrate_kbps = 0;   /* remembered for diagnostics */

static volatile bool s_enabled = false;

/* Which command frame the periodic sender transmits (must match the motor's
 * CTRL_MODE register). Defaults to MIT. */
static volatile motor_ctrl_mode_t s_mode = MOTOR_MODE_MIT;

/* Setpoints. Position/velocity are shared across the modes that use them; the
 * per-mode extras are separate. All accessed only as whole 32-bit words, so
 * a plain volatile read/write is atomic enough for setpoint streaming. */
static volatile float s_pos = 0.0f;   /* position command, rad                */
static volatile float s_vel = 0.0f;   /* velocity command / limit, rad/s      */
static volatile float s_tff = 0.0f;   /* MIT torque feedforward, N·m          */
/* Default impedance gains. Kd must be non-zero for stable position control. */
static volatile float s_kp = 5.0f;
static volatile float s_kd = 1.0f;
static volatile float s_cur_limit = 0.0f;  /* force-pos current limit, 0..1 pu */

/* Read-param cache, indexed by register RID (see manual register table; max
 * used RID is 0x51). Filled from the async read reply in the RX callback. */
#define DM_PARAM_CACHE_LEN  0x52
static volatile uint32_t s_param_raw[DM_PARAM_CACHE_LEN];
static volatile bool     s_param_valid[DM_PARAM_CACHE_LEN];

/* ---- low-level TX -------------------------------------------------------- */

static void motor_tx(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    if (!s_node) {
        return;
    }
    twai_frame_t frame = {
        .header = { .id = can_id, .dlc = len },
        .buffer = (uint8_t *)data,
        .buffer_len = len,
    };
    esp_err_t err = twai_node_transmit(s_node, &frame, 10 /* ms */);
    if (err == ESP_OK) {
        return;
    }

    /* A CAN frame must be ACKed by at least one OTHER node. With the motor
     * unpowered, no transceiver, wrong bit rate or a broken H/L pair, every TX
     * errors, the TX error counter saturates and the controller latches BUS-OFF
     * — after which it refuses to transmit (ESP_ERR_INVALID_STATE) until it is
     * explicitly recovered. Retry recovery (and log) at most once a second so a
     * dead bus can't flood the console or wedge us until reboot. */
    static TickType_t next_tick = 0;
    TickType_t now = xTaskGetTickCount();
    if ((int32_t)(now - next_tick) < 0) {   /* signed diff: wrap-safe */
        return;
    }
    next_tick = now + pdMS_TO_TICKS(1000);

    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "CAN bus-off (no ACK on the bus) - recovering. Check: motor powered? "
                      "transceiver powered + STB pulled low? 120R termination? "
                      "bit rate %d kbps matches the motor? CANH/CANL not swapped?",
                 s_bitrate_kbps);
        twai_node_recover(s_node);
    } else {
        ESP_LOGW(TAG, "transmit (id=0x%03X) failed: %s", (unsigned)can_id, esp_err_to_name(err));
    }
}

/* ---- enable / fault control (any mode) ---------------------------------- */

void motor_twai_enable(void)
{
    s_enabled = true;
    motor_tx(s_esc_id, DM_FRAME_ENABLE, 8);
    ESP_LOGI(TAG, "enable requested");
}

void motor_twai_disable(void)
{
    s_enabled = false;
    motor_tx(s_esc_id, DM_FRAME_DISABLE, 8);
    ESP_LOGI(TAG, "disable requested");
}

void motor_twai_save_zero(void)
{
    motor_tx(s_esc_id, DM_FRAME_SAVE_ZERO, 8);
    ESP_LOGI(TAG, "save-zero");
}

void motor_twai_clear_error(void)
{
    motor_tx(s_esc_id, DM_FRAME_CLEAR_ERR, 8);
    ESP_LOGI(TAG, "clear-error");
}

/* ---- register access (config frames on CAN ID 0x7FF) -------------------- */

/* Register read/write frame: D0..D1 = motor CAN_ID (little-endian), D2 = cmd,
 * D3 = RID, D4..D7 = 32-bit payload (little-endian, LSB in D4). ESP32 is
 * little-endian so the 4-byte value maps straight into D4..D7 via memcpy. */
static void motor_param_write_raw(uint8_t rid, uint32_t raw)
{
    uint8_t d[8];
    d[0] = s_esc_id & 0xFF;
    d[1] = (s_esc_id >> 8) & 0xFF;
    d[2] = DM_PARAM_CMD_WRITE;
    d[3] = rid;
    memcpy(&d[4], &raw, 4);
    motor_tx(DM_PARAM_FRAME_ID, d, 8);
}

void motor_twai_write_param_f32(uint8_t rid, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, 4);
    motor_param_write_raw(rid, raw);
    ESP_LOGI(TAG, "write reg 0x%02X = %.4f", rid, value);
}

void motor_twai_write_param_u32(uint8_t rid, uint32_t value)
{
    motor_param_write_raw(rid, value);
    ESP_LOGI(TAG, "write reg 0x%02X = %u", rid, (unsigned)value);
}

void motor_twai_save_params(void)
{
    if (s_enabled) {
        ESP_LOGW(TAG, "save-params ignored by motor unless DISABLED first");
    }
    uint8_t d[4] = { s_esc_id & 0xFF, (s_esc_id >> 8) & 0xFF, DM_PARAM_CMD_SAVE, 0x01 };
    motor_tx(DM_PARAM_FRAME_ID, d, 4);
    ESP_LOGI(TAG, "save-params (flash)");
}

void motor_twai_set_baudrate(motor_baud_code_t code)
{
    motor_twai_write_param_u32(MOTOR_REG_CAN_BR, (uint32_t)code);
    ESP_LOGW(TAG, "baud code %d written; save + power-cycle to apply cleanly", (int)code);
}

void motor_twai_request_param(uint8_t rid)
{
    uint8_t d[4] = { s_esc_id & 0xFF, (s_esc_id >> 8) & 0xFF, DM_PARAM_CMD_READ, rid };
    motor_tx(DM_PARAM_FRAME_ID, d, 4);
}

bool motor_twai_get_param_u32(uint8_t rid, uint32_t *out)
{
    if (rid >= DM_PARAM_CACHE_LEN || !s_param_valid[rid]) {
        return false;
    }
    if (out) *out = s_param_raw[rid];
    return true;
}

bool motor_twai_get_param_f32(uint8_t rid, float *out)
{
    uint32_t raw;
    if (!motor_twai_get_param_u32(rid, &raw)) {
        return false;
    }
    if (out) memcpy(out, &raw, 4);
    return true;
}

/* ---- mode selection ----------------------------------------------------- */

void motor_twai_set_mode(motor_ctrl_mode_t mode)
{
    /* Writing CTRL_MODE zeroes the motor's command values; mirror that locally
     * so a stale setpoint from the previous mode isn't streamed on the first
     * frame of the new mode. */
    s_pos = 0.0f;
    s_vel = 0.0f;
    s_tff = 0.0f;
    s_cur_limit = 0.0f;
    s_mode = mode;
    motor_twai_write_param_u32(MOTOR_REG_CTRL_MODE, (uint32_t)mode);
    ESP_LOGI(TAG, "control mode -> %d", (int)mode);
}

motor_ctrl_mode_t motor_twai_get_mode(void)
{
    return s_mode;
}

/* ---- per-mode setpoint interfaces --------------------------------------- */

void motor_twai_set_mit(float pos, float vel, float kp, float kd, float tff)
{
    s_pos = pos;
    s_vel = vel;
    s_kp  = kp;
    s_kd  = kd;
    s_tff = tff;
    s_mode = MOTOR_MODE_MIT;
}

void motor_twai_set_target(float target)
{
    s_pos = target;
    s_vel = 0.0f;
    s_tff = 0.0f;
    s_mode = MOTOR_MODE_MIT;
    ESP_LOGI(TAG, "target=%.3f", target);
}

void motor_twai_set_gains(float kp, float kd)
{
    s_kp = kp;
    s_kd = kd;
    ESP_LOGI(TAG, "gains kp=%.2f kd=%.2f", kp, kd);
}

void motor_twai_set_pos_vel(float pos, float vel)
{
    s_pos = pos;
    s_vel = vel;
    s_mode = MOTOR_MODE_POS_VEL;
}

void motor_twai_set_velocity(float vel)
{
    s_vel = vel;
    s_mode = MOTOR_MODE_VELOCITY;
}

void motor_twai_set_force_pos(float pos, float vel_limit, float cur_limit)
{
    s_pos = pos;
    s_vel = vel_limit;
    s_cur_limit = cur_limit;
    s_mode = MOTOR_MODE_FORCE_POS;
}

/* ---- periodic command frame --------------------------------------------- */

/* MODE 1 (MIT), sent to ESC_ID. 8-byte packing (DM-J4310 MIT mode):
 *   D0 = p[15:8]              D1 = p[7:0]
 *   D2 = v[11:4]              D3 = v[3:0]<<4 | kp[11:8]
 *   D4 = kp[7:0]              D5 = kd[11:4]
 *   D6 = kd[3:0]<<4 | t[11:8] D7 = t[7:0] */
static void send_mit(void)
{
    uint16_t p  = float_to_uint(s_pos, DM_P_MIN, DM_P_MAX, 16);
    uint16_t v  = float_to_uint(s_vel, DM_V_MIN, DM_V_MAX, 12);
    uint16_t kp = float_to_uint(s_kp,  DM_KP_MIN, DM_KP_MAX, 12);
    uint16_t kd = float_to_uint(s_kd,  DM_KD_MIN, DM_KD_MAX, 12);
    uint16_t tq = float_to_uint(s_tff, DM_T_MIN, DM_T_MAX, 12);

    uint8_t d[8];
    d[0] = p >> 8;
    d[1] = p & 0xFF;
    d[2] = v >> 4;
    d[3] = ((v & 0xF) << 4) | (kp >> 8);
    d[4] = kp & 0xFF;
    d[5] = kd >> 4;
    d[6] = ((kd & 0xF) << 4) | (tq >> 8);
    d[7] = tq & 0xFF;
    motor_tx(s_esc_id, d, 8);
}

/* MODE 2 (position-velocity), sent to 0x100+ESC_ID.
 *   D0..D3 = p_des (float LE), D4..D7 = v_des (float LE). */
static void send_pos_vel(void)
{
    float p = s_pos, v = s_vel;
    uint8_t d[8];
    memcpy(&d[0], &p, 4);
    memcpy(&d[4], &v, 4);
    motor_tx(DM_ID_OFFSET_POS_VEL + s_esc_id, d, 8);
}

/* MODE 3 (velocity), sent to 0x200+ESC_ID. D0..D3 = v_des (float LE). */
static void send_velocity(void)
{
    float v = s_vel;
    uint8_t d[4];
    memcpy(&d[0], &v, 4);
    motor_tx(DM_ID_OFFSET_VELOCITY + s_esc_id, d, 4);
}

/* MODE 4 (force-position), sent to 0x300+ESC_ID.
 *   D0..D3 = p_des (float LE)
 *   D4..D5 = v_des = vel_limit*100 (uint16 LE, 0..10000)
 *   D6..D7 = i_des = cur_limit*10000 (uint16 LE, 0..10000). */
static void send_force_pos(void)
{
    float p = s_pos;
    float vl = s_vel;
    float il = s_cur_limit;
    if (vl < 0.0f) vl = 0.0f;
    if (il < 0.0f) il = 0.0f;

    uint32_t vraw = (uint32_t)(vl * DM_FP_VEL_SCALE);
    uint32_t iraw = (uint32_t)(il * DM_FP_CUR_SCALE);
    if (vraw > DM_FP_VEL_MAX_RAW) vraw = DM_FP_VEL_MAX_RAW;
    if (iraw > DM_FP_CUR_MAX_RAW) iraw = DM_FP_CUR_MAX_RAW;

    uint8_t d[8];
    memcpy(&d[0], &p, 4);
    d[4] = vraw & 0xFF;
    d[5] = (vraw >> 8) & 0xFF;
    d[6] = iraw & 0xFF;
    d[7] = (iraw >> 8) & 0xFF;
    motor_tx(DM_ID_OFFSET_FORCE_POS + s_esc_id, d, 8);
}

/**
 * @brief Transmit the command frame for the active control mode while enabled.
 * Re-sending every cycle also services the motor's comms-loss timeout (0x09).
 */
static void motor_send_command(void)
{
    if (!s_enabled) {
        return;
    }
    switch (s_mode) {
    case MOTOR_MODE_POS_VEL:   send_pos_vel();   break;
    case MOTOR_MODE_VELOCITY:  send_velocity();  break;
    case MOTOR_MODE_FORCE_POS: send_force_pos(); break;
    case MOTOR_MODE_MIT:
    default:                   send_mit();       break;
    }
}

/* ---- RX path: on_rx_done runs in ISR context ---------------------------- */

static bool motor_on_rx_done(twai_node_handle_t handle,
                             const twai_rx_done_event_data_t *edata,
                             void *user_ctx)
{
    (void)edata; (void)user_ctx;

    uint8_t d[8];
    twai_frame_t rx = { .buffer = d, .buffer_len = sizeof(d) };
    if (twai_node_receive_from_isr(handle, &rx) != ESP_OK) {
        return false;
    }

    /* Register read/write replies arrive on the feedback ID too. They carry the
     * motor CAN_ID in D0/D1 and the command byte (0x33 read, 0x55 write) in D2,
     * so distinguish them from a feedback frame before decoding. (Reads are done
     * at config time with the motor disabled, so there is no live feedback to
     * collide with.) */
    uint16_t echo_id = (uint16_t)(d[0] | (d[1] << 8));
    if ((d[2] == DM_PARAM_CMD_READ || d[2] == DM_PARAM_CMD_WRITE) &&
        echo_id == (uint16_t)s_esc_id) {
        uint8_t rid = d[3];
        if (rid < DM_PARAM_CACHE_LEN) {
            memcpy((void *)&s_param_raw[rid], &d[4], 4);
            s_param_valid[rid] = true;
        }
        return false;
    }

    /* DM-J4310 feedback frame (8 bytes, same in every mode):
     *   D0 = motor_id | (err << 4)
     *   D1 = pos[15:8]   D2 = pos[7:0]
     *   D3 = vel[11:4]   D4 = vel[3:0]<<4 | tq[11:8]   D5 = tq[7:0]
     *   D6 = T_MOS (°C)  D7 = T_Rotor (°C) */
    uint8_t  err = d[0] >> 4;
    uint16_t p_int = (d[1] << 8) | d[2];
    uint16_t v_int = (d[3] << 4) | (d[4] >> 4);
    uint16_t t_int = ((d[4] & 0xF) << 8) | d[5];

    static uint32_t seq = 0;
    motor_status_t st = {
        .seq        = ++seq,
        .enabled    = (err == 1),
        .position   = uint_to_float(p_int, DM_P_MIN, DM_P_MAX, 16),
        .velocity   = uint_to_float(v_int, DM_V_MIN, DM_V_MAX, 12),
        .torque     = uint_to_float(t_int, DM_T_MIN, DM_T_MAX, 12),
        .t_mos      = (float)d[6],
        .t_rotor    = (float)d[7],
        .error_code = (err <= 1) ? 0 : err,   /* 0/1 are healthy, >=8 are faults */
    };

    BaseType_t hp_task_woken = pdFALSE;
    if (s_bus && s_bus->motor_status_q) {
        xQueueSendFromISR(s_bus->motor_status_q, &st, &hp_task_woken);
    }
    return hp_task_woken == pdTRUE;
}

static void motor_twai_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "motor_twai_task running");

    const TickType_t period = pdMS_TO_TICKS(10);  /* 100 Hz command cadence */
    for (;;) {
        motor_send_command();
        vTaskDelay(period);
    }
}

int motor_twai_start(const motor_twai_cfg_t *cfg, const app_bus_t *bus)
{
    if (!cfg || !bus) {
        return ESP_ERR_INVALID_ARG;
    }
    s_bus = bus;
    s_esc_id = cfg->esc_id >= 0 ? (uint32_t)cfg->esc_id : MOTOR_ESC_ID_DEFAULT;
    s_mst_id = cfg->mst_id >= 0 ? (uint32_t)cfg->mst_id : MOTOR_MST_ID_DEFAULT;
    s_bitrate_kbps = cfg->bitrate_kbps;

    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx = cfg->tx_gpio,
            .rx = cfg->rx_gpio,
            .quanta_clk_out = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing = {
            .bitrate = (uint32_t)cfg->bitrate_kbps * 1000,
        },
        .tx_queue_depth = 8,
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &s_node));

    const twai_event_callbacks_t cbs = {
        .on_rx_done = motor_on_rx_done,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_node, &cbs, NULL));

    /* Only accept the motor's feedback frame (MST_ID). A full 11-bit mask means
     * every ID bit must match, so unrelated traffic on a shared bus can't be
     * mis-decoded as motor feedback and spuriously trip the ERROR/SAFE_STOP
     * path. Set MOTOR_TWAI_MST_ID (app_config.h) if the motor is reconfigured. */
    const twai_mask_filter_config_t filter = { .id = s_mst_id, .mask = 0x7FF, .is_ext = false };
    ESP_ERROR_CHECK(twai_node_config_mask_filter(s_node, 0, &filter));

    ESP_ERROR_CHECK(twai_node_enable(s_node));

    ESP_LOGI(TAG, "TWAI up: TX=GPIO%d RX=GPIO%d @ %d kbps (external transceiver required)",
             cfg->tx_gpio, cfg->rx_gpio, cfg->bitrate_kbps);

    uint32_t   stack = cfg->task_stack > 0 ? (uint32_t)cfg->task_stack : 4096;
    UBaseType_t prio = cfg->task_prio  > 0 ? (UBaseType_t)cfg->task_prio : 7;
    if (xTaskCreate(motor_twai_task, "motor_twai", stack, NULL, prio, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create motor_twai_task");
        return ESP_FAIL;
    }
    return ESP_OK;
}
