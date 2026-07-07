#include "motor_twai.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "motor_twai";

/* Damiao DM-J4310-2EC CAN framing (CAN 2.0A standard frame, 11-bit ID, DLC 8).
 *
 * ESC_ID (reg 0x08) = motor's receive ID, factory default 0x01.
 * MST_ID (reg 0x07) = feedback ID the motor transmits on, factory default 0x00.
 * In MIT control mode the command frame is sent to CAN ID = ESC_ID.
 * Update these to match your motor's configured IDs. */
#define MOTOR_ESC_ID    0x001   /* host -> motor: MIT command frame ID */
#define MOTOR_MST_ID    0x000   /* motor -> host: feedback frame ID   */

/* MIT-mode value ranges (register defaults PMAX/VMAX/TMAX for the 4310). Keep
 * these in sync with the motor's PMAX(0x15)/VMAX(0x16)/TMAX(0x17) registers. */
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

static volatile bool  s_enabled = false;
static volatile float s_target  = 0.0f;   /* MIT position command, rad     */
static volatile float s_vel     = 0.0f;   /* MIT velocity command, rad/s   */
static volatile float s_tff     = 0.0f;   /* MIT torque feedforward, N·m   */
/* Default impedance gains. Kd must be non-zero for stable position control. */
static volatile float s_kp = 5.0f;
static volatile float s_kd = 1.0f;

/* ---- public command API (callable from BLE / supervisor) ---------------- */

static void motor_tx_id(uint32_t can_id, const uint8_t data[8])
{
    twai_frame_t frame = {
        .header = { .id = can_id, .dlc = 8 },
        .buffer = (uint8_t *)data,
        .buffer_len = 8,
    };
    esp_err_t err = twai_node_transmit(s_node, &frame, 10 /* ms */);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "transmit failed: %s", esp_err_to_name(err));
    }
}

void motor_twai_enable(void)
{
    s_enabled = true;
    if (s_node) motor_tx_id(MOTOR_ESC_ID, DM_FRAME_ENABLE);
    ESP_LOGI(TAG, "enable requested");
}

void motor_twai_disable(void)
{
    s_enabled = false;
    if (s_node) motor_tx_id(MOTOR_ESC_ID, DM_FRAME_DISABLE);
    ESP_LOGI(TAG, "disable requested");
}

void motor_twai_set_target(float target)
{
    s_target = target;
    s_vel = 0.0f;
    s_tff = 0.0f;
    ESP_LOGI(TAG, "target=%.3f", target);
}

void motor_twai_set_mit(float pos, float vel, float kp, float kd, float tff)
{
    s_target = pos;
    s_vel    = vel;
    s_kp     = kp;
    s_kd     = kd;
    s_tff    = tff;
}

void motor_twai_set_gains(float kp, float kd)
{
    s_kp = kp;
    s_kd = kd;
    ESP_LOGI(TAG, "gains kp=%.2f kd=%.2f", kp, kd);
}

void motor_twai_save_zero(void)
{
    if (s_node) motor_tx_id(MOTOR_ESC_ID, DM_FRAME_SAVE_ZERO);
    ESP_LOGI(TAG, "save-zero");
}

void motor_twai_clear_error(void)
{
    if (s_node) motor_tx_id(MOTOR_ESC_ID, DM_FRAME_CLEAR_ERR);
    ESP_LOGI(TAG, "clear-error");
}

/* ---- periodic command frame --------------------------------------------- */

/**
 * @brief Transmit a MIT-mode command frame while enabled.
 *
 * 8-byte packing (DM-J4310 MIT mode):
 *   D0 = p[15:8]              D1 = p[7:0]
 *   D2 = v[11:4]             D3 = v[3:0]<<4 | kp[11:8]
 *   D4 = kp[7:0]            D5 = kd[11:4]
 *   D6 = kd[3:0]<<4 | t[11:8]  D7 = t[7:0]
 * Fields come from the last motor_twai_set_target()/set_mit() call, so the
 * gripper layer can do position, velocity, or torque(force) control.
 */
static void motor_send_command(void)
{
    if (!s_enabled) {
        return;
    }
    uint16_t p  = float_to_uint(s_target, DM_P_MIN, DM_P_MAX, 16);
    uint16_t v  = float_to_uint(s_vel,    DM_V_MIN, DM_V_MAX, 12);
    uint16_t kp = float_to_uint(s_kp,     DM_KP_MIN, DM_KP_MAX, 12);
    uint16_t kd = float_to_uint(s_kd,     DM_KD_MIN, DM_KD_MAX, 12);
    uint16_t tq = float_to_uint(s_tff,    DM_T_MIN, DM_T_MAX, 12);

    uint8_t data[8];
    data[0] = p >> 8;
    data[1] = p & 0xFF;
    data[2] = v >> 4;
    data[3] = ((v & 0xF) << 4) | (kp >> 8);
    data[4] = kp & 0xFF;
    data[5] = kd >> 4;
    data[6] = ((kd & 0xF) << 4) | (tq >> 8);
    data[7] = tq & 0xFF;
    motor_tx_id(MOTOR_ESC_ID, data);
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

    /* Accept all standard IDs (id=0, mask=0 -> every bit "don't care"). */
    const twai_mask_filter_config_t filter = { .id = 0, .mask = 0, .is_ext = false };
    ESP_ERROR_CHECK(twai_node_config_mask_filter(s_node, 0, &filter));

    ESP_ERROR_CHECK(twai_node_enable(s_node));

    ESP_LOGI(TAG, "TWAI up: TX=GPIO%d RX=GPIO%d @ %d kbps (external transceiver required)",
             cfg->tx_gpio, cfg->rx_gpio, cfg->bitrate_kbps);

    if (xTaskCreate(motor_twai_task, "motor_twai", 4096, NULL, 7, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create motor_twai_task");
        return ESP_FAIL;
    }
    return ESP_OK;
}
