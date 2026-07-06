#include "app_supervisor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "motor_twai.h"
#include "wireless_ble.h"

static const char *TAG = "supervisor";

static const app_bus_t   *s_bus = NULL;
static volatile app_state_t s_state = APP_STATE_INIT;

/* Liveness watchdog: if we stop hearing from a sensor for too long we drop to
 * SAFE_STOP. Counts are in supervisor loop ticks (see loop period below). */
#define LOOP_PERIOD_MS      20
#define HAPTIC_TIMEOUT_MS   500
#define IMU_TIMEOUT_MS      500

const volatile app_state_t *app_supervisor_state_ptr(void)
{
    return &s_state;
}

static void enter_state(app_state_t next)
{
    if (next != s_state) {
        ESP_LOGI(TAG, "state %s -> %s", app_state_name(s_state), app_state_name(next));
        s_state = next;
    }
}

/* Apply one control command, gated by the current state. This is the safety
 * choke point: e.g. we refuse to enable the motor while in ERROR/SAFE_STOP. */
static void handle_ctrl(const ctrl_msg_t *cmd)
{
    switch (cmd->id) {
    case CTRL_CMD_ENABLE_MOTOR:
        if (s_state == APP_STATE_ERROR || s_state == APP_STATE_SAFE_STOP) {
            ESP_LOGW(TAG, "refusing motor enable in state %s", app_state_name(s_state));
            break;
        }
        motor_twai_enable();
        enter_state(APP_STATE_ACTIVE);
        break;

    case CTRL_CMD_DISABLE_MOTOR:
        motor_twai_disable();
        enter_state(APP_STATE_IDLE);
        break;

    case CTRL_CMD_SET_TARGET:
        if (s_state == APP_STATE_ACTIVE) {
            motor_twai_set_target(cmd->target);
        } else {
            ESP_LOGW(TAG, "ignoring set_target in state %s", app_state_name(s_state));
        }
        break;

    case CTRL_CMD_REQUEST_STATUS:
        /* handled implicitly: status is published every time we get one */
        break;

    default:
        ESP_LOGW(TAG, "unknown ctrl id %d", cmd->id);
        break;
    }
}

static void supervisor_task(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "supervisor_task running");

    haptic_sample_t haptic;
    imu_sample_t    imu;
    motor_status_t  status;
    ctrl_msg_t      cmd;

    int haptic_silence_ms = 0;
    int imu_silence_ms    = 0;

    /* Bring-up complete: move out of INIT. */
    enter_state(APP_STATE_IDLE);

    for (;;) {
        bool got_haptic = false, got_imu = false;

        /* Drain sensor queues (non-blocking). */
        while (xQueueReceive(s_bus->haptic_q, &haptic, 0) == pdTRUE) {
            got_haptic = true;
            /* TODO: run safety checks on pressure (e.g. over-force -> SAFE_STOP). */
        }
        while (xQueueReceive(s_bus->imu_q, &imu, 0) == pdTRUE) {
            got_imu = true;
            /* TODO: use orientation for tip-over / limit detection. */
        }
        while (xQueueReceive(s_bus->motor_status_q, &status, 0) == pdTRUE) {
            if (status.error_code != 0) {
                ESP_LOGW(TAG, "motor error_code=%u", status.error_code);
                enter_state(APP_STATE_ERROR);
            }
            wireless_ble_publish_status(&status);
        }

        /* Control commands from BLE / Wi-Fi. */
        while (xQueueReceive(s_bus->ctrl_q, &cmd, 0) == pdTRUE) {
            handle_ctrl(&cmd);
        }

        /* Sensor-liveness watchdog. */
        haptic_silence_ms = got_haptic ? 0 : haptic_silence_ms + LOOP_PERIOD_MS;
        imu_silence_ms    = got_imu    ? 0 : imu_silence_ms + LOOP_PERIOD_MS;
        if (s_state == APP_STATE_ACTIVE &&
            (haptic_silence_ms > HAPTIC_TIMEOUT_MS || imu_silence_ms > IMU_TIMEOUT_MS)) {
            ESP_LOGE(TAG, "sensor timeout (haptic=%dms imu=%dms) -> SAFE_STOP",
                     haptic_silence_ms, imu_silence_ms);
            motor_twai_disable();
            enter_state(APP_STATE_SAFE_STOP);
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_PERIOD_MS));
    }
}

int app_supervisor_start(const app_bus_t *bus)
{
    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }
    s_bus = bus;

    if (xTaskCreate(supervisor_task, "supervisor", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create supervisor_task");
        return ESP_FAIL;
    }
    return ESP_OK;
}
