#include "app_protocol.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "app_protocol";

const char *app_state_name(app_state_t s)
{
    switch (s) {
    case APP_STATE_INIT:      return "INIT";
    case APP_STATE_IDLE:      return "IDLE";
    case APP_STATE_ACTIVE:    return "ACTIVE";
    case APP_STATE_ERROR:     return "ERROR";
    case APP_STATE_SAFE_STOP: return "SAFE_STOP";
    default:                  return "?";
    }
}

int app_bus_init(app_bus_t *bus)
{
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    bus->haptic_q       = xQueueCreate(QUEUE_LEN_HAPTIC,       sizeof(haptic_sample_t));
    bus->imu_q          = xQueueCreate(QUEUE_LEN_IMU,          sizeof(imu_sample_t));
    bus->motor_status_q = xQueueCreate(QUEUE_LEN_MOTOR_STATUS, sizeof(motor_status_t));
    bus->ctrl_q         = xQueueCreate(QUEUE_LEN_CTRL,         sizeof(app_cmd_t));

    if (!bus->haptic_q || !bus->imu_q || !bus->motor_status_q || !bus->ctrl_q) {
        ESP_LOGE(TAG, "failed to allocate one or more bus queues");
        if (bus->haptic_q)       vQueueDelete(bus->haptic_q);
        if (bus->imu_q)          vQueueDelete(bus->imu_q);
        if (bus->motor_status_q) vQueueDelete(bus->motor_status_q);
        if (bus->ctrl_q)         vQueueDelete(bus->ctrl_q);
        bus->haptic_q = bus->imu_q = bus->motor_status_q = bus->ctrl_q = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "message bus ready");
    return ESP_OK;
}

void app_bus_post_transport_closed(const app_bus_t *bus, uint8_t transport)
{
    if (!bus || !bus->ctrl_q) {
        return;
    }
    app_cmd_t cmd = { .op = OP_INTERNAL_TRANSPORT_CLOSED, .transport = transport };
    /* wait briefly: losing this message would leave the session unlocked */
    xQueueSend(bus->ctrl_q, &cmd, pdMS_TO_TICKS(50));
}
