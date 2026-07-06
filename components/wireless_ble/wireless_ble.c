#include "wireless_ble.h"

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "wireless_ble";

/* ---- 128-bit UUIDs (randomly chosen for this project) --------------------
 * Service : 6767-... "Gripper Control"
 * Control char (write)  : ...0002
 * Status  char (read/notify) : ...0003
 * ------------------------------------------------------------------------- */
static const ble_uuid128_t s_svc_uuid = BLE_UUID128_INIT(
    0x11, 0x11, 0x00, 0x00, 0x67, 0x67, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x70, 0x67);
static const ble_uuid128_t s_ctrl_uuid = BLE_UUID128_INIT(
    0x11, 0x11, 0x00, 0x00, 0x67, 0x67, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x70, 0x67);
static const ble_uuid128_t s_status_uuid = BLE_UUID128_INIT(
    0x11, 0x11, 0x00, 0x00, 0x67, 0x67, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x70, 0x67);

static const app_bus_t *s_bus = NULL;
static const char      *s_device_name = "GripperPlus";
static uint8_t          s_addr_type;
static uint16_t         s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t         s_status_val_handle;
static motor_status_t   s_last_status;

static void ble_advertise(void);

/* ---- GATT access callbacks ---------------------------------------------- */

static int gatt_ctrl_write(struct ble_gatt_access_ctxt *ctxt)
{
    uint8_t buf[5] = {0};
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len < 1 || len > sizeof(buf)) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len) != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    ctrl_msg_t msg = { .id = (ctrl_cmd_id_t)buf[0], .target = 0.0f };
    if (len >= 5) {
        memcpy(&msg.target, &buf[1], sizeof(float));
    }

    ESP_LOGI(TAG, "ctrl write: id=%d target=%.3f", msg.id, msg.target);
    if (s_bus && s_bus->ctrl_q) {
        xQueueSend(s_bus->ctrl_q, &msg, 0);
    }
    return 0;
}

static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        return gatt_ctrl_write(ctxt);
    case BLE_GATT_ACCESS_OP_READ_CHR:
        /* status characteristic read */
        return os_mbuf_append(ctxt->om, &s_last_status, sizeof(s_last_status))
               == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = &s_ctrl_uuid.u,
                .access_cb  = gatt_svc_access,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid       = &s_status_uuid.u,
                .access_cb  = gatt_svc_access,
                .val_handle = &s_status_val_handle,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 } /* no more characteristics */
        },
    },
    { 0 } /* no more services */
};

/* ---- GAP event handling -------------------------------------------------- */

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "connect %s", event->connect.status == 0 ? "OK" : "FAILED");
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
        } else {
            ble_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect; reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise();
        break;
    default:
        break;
    }
    return 0;
}

static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = (uint8_t *)s_device_name;
    fields.name_len = strlen(s_device_name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
        return;
    }
    rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start rc=%d", rc);
    }
}

/* ---- Host lifecycle ------------------------------------------------------ */

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ensure_addr rc=%d", rc);
        return;
    }
    ble_hs_id_infer_auto(0, &s_addr_type);
    ESP_LOGI(TAG, "BLE synced, advertising as \"%s\"", s_device_name);
    ble_advertise();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset; reason=%d", reason);
}

/* This is the "ble_control_task": the NimBLE host runs here. */
static void ble_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "ble_control_task (NimBLE host) started");
    nimble_port_run();            /* returns only on nimble_port_stop() */
    nimble_port_freertos_deinit();
}

/* ---- Public API ---------------------------------------------------------- */

void wireless_ble_publish_status(const motor_status_t *status)
{
    if (!status) {
        return;
    }
    s_last_status = *status;
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&s_last_status, sizeof(s_last_status));
        if (om) {
            ble_gatts_notify_custom(s_conn_handle, s_status_val_handle, om);
        }
    }
}

int wireless_ble_start(const app_bus_t *bus, const char *device_name)
{
    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }
    s_bus = bus;
    if (device_name) {
        s_device_name = device_name;
    }

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_count_cfg rc=%d", rc); return ESP_FAIL; }
    rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_add_svcs rc=%d", rc); return ESP_FAIL; }

    rc = ble_svc_gap_device_name_set(s_device_name);
    if (rc != 0) { ESP_LOGW(TAG, "device_name_set rc=%d", rc); }

    nimble_port_freertos_init(ble_host_task);
    return ESP_OK;
}
