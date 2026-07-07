#include "wireless_ble.h"
#include "app_proto.h"

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "wireless_ble";

/* 128-bit UUIDs (project-specific).
 *   Service           : ...0001
 *   Control  (write)  : ...0002   framed app_proto commands in
 *   Telemetry(rd/ntfy): ...0003   framed app_proto responses/telemetry out */
static const ble_uuid128_t s_svc_uuid = BLE_UUID128_INIT(
    0x11, 0x11, 0x00, 0x00, 0x67, 0x67, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x70, 0x67);
static const ble_uuid128_t s_ctrl_uuid = BLE_UUID128_INIT(
    0x11, 0x11, 0x00, 0x00, 0x67, 0x67, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x70, 0x67);
static const ble_uuid128_t s_telem_uuid = BLE_UUID128_INIT(
    0x11, 0x11, 0x00, 0x00, 0x67, 0x67, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x70, 0x67);

static const app_bus_t *s_bus = NULL;
static const char      *s_device_name = "GripperPlus";
static uint8_t          s_addr_type;
static volatile uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;  /* written on host task, read on supervisor task */
static uint16_t         s_telem_val_handle;

static app_proto_parser_t s_parser;                 /* per-connection reassembly */
static uint8_t          s_last_tx[APP_PROTO_MAX_FRAME];  /* last frame, for GATT read */
static size_t           s_last_tx_len;

static void ble_advertise(void);

/* ---- app_proto sink: deliver a built frame as a GATT notification -------- */
static void ble_sink(const uint8_t *frame, size_t len, void *ctx)
{
    (void)ctx;
    if (len <= sizeof(s_last_tx)) {
        memcpy(s_last_tx, frame, len);
        s_last_tx_len = len;
    }
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }
    struct os_mbuf *om = ble_hs_mbuf_from_flat(frame, len);
    if (om) {
        ble_gatts_notify_custom(s_conn_handle, s_telem_val_handle, om);
    }
}

/* ---- decoded command -> supervisor queue -------------------------------- */
static void ble_cmd_cb(const app_cmd_t *cmd, void *ctx)
{
    (void)ctx;
    if (s_bus && s_bus->ctrl_q) {
        xQueueSend(s_bus->ctrl_q, cmd, 0);
    }
}

/* ---- GATT access -------------------------------------------------------- */
static int gatt_access(uint16_t conn_handle, uint16_t attr_handle,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle; (void)attr_handle; (void)arg;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        uint8_t buf[APP_PROTO_MAX_FRAME];
        uint16_t len = 0;
        if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len) != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        app_proto_feed(&s_parser, 0 /* transport = BLE */, buf, len, ble_cmd_cb, NULL);
        return 0;
    }
    case BLE_GATT_ACCESS_OP_READ_CHR:
        /* return the most recent outbound frame (telemetry/response) */
        return os_mbuf_append(ctxt->om, s_last_tx, s_last_tx_len) == 0
               ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
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
                .uuid      = &s_ctrl_uuid.u,
                .access_cb = gatt_access,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid       = &s_telem_uuid.u,
                .access_cb  = gatt_access,
                .val_handle = &s_telem_val_handle,
                .flags      = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        },
    },
    { 0 }
};

/* ---- GAP ---------------------------------------------------------------- */
static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            s_parser.len = 0;                          /* fresh reassembly */
            app_proto_set_sink(ble_sink, NULL);        /* route telemetry here */
            ESP_LOGI(TAG, "connected");
        } else {
            ble_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected; reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        app_proto_set_sink(NULL, NULL);
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
    if (rc != 0) { ESP_LOGE(TAG, "adv_set_fields rc=%d", rc); return; }
    rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event, NULL);
    if (rc != 0) { ESP_LOGE(TAG, "adv_start rc=%d", rc); }
}

/* ---- host lifecycle ----------------------------------------------------- */
static void on_sync(void)
{
    if (ble_hs_util_ensure_addr(0) != 0) { ESP_LOGE(TAG, "ensure_addr failed"); return; }
    ble_hs_id_infer_auto(0, &s_addr_type);
    ESP_LOGI(TAG, "BLE synced, advertising as \"%s\"", s_device_name);
    ble_advertise();
}

static void on_reset(int reason) { ESP_LOGW(TAG, "BLE host reset; reason=%d", reason); }

static void ble_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "ble_control_task (NimBLE host) started");
    nimble_port_run();
    nimble_port_freertos_deinit();
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

    /* Telemetry frames can exceed the 23-byte default MTU; ask for more. The
     * central must accept the negotiation for large notifications to fit. */
    ble_att_set_preferred_mtu(247);

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
