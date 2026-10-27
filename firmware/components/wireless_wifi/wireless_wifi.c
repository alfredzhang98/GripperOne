#include "wireless_wifi.h"
#include "app_proto.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "lwip/sockets.h"

static const char *TAG = "wireless_wifi";

#define NVS_NS         "wifi"
#define TX_QUEUE_LEN   16          /* frames waiting for the socket            */
#define RX_CHUNK       256
#define STA_RETRY_US   (5 * 1000 * 1000)
#define SELECT_WAIT_US 5000        /* the server loop drains TX at least this often */
#define SEND_TIMEOUT_MS 200        /* a peer that stops reading is dropped      */
#define SERVER_STACK   6144
#define SERVER_PRIO    4           /* below the supervisor (5): control comes first */

typedef struct {
    uint16_t len;
    uint8_t  data[APP_PROTO_MAX_FRAME];
} tx_frame_t;

static const app_bus_t    *s_bus;
static wireless_wifi_cfg_t s_cfg;
static bool                s_started;
static esp_netif_t        *s_ap_netif;
static char                s_ap_ssid[33];
static char                s_sta_ssid[33];
static volatile uint8_t    s_sta_state;    /* 0 none, 1 connecting, 2 connected */
static volatile uint32_t   s_sta_ip;
static volatile bool       s_client;
static volatile uint32_t   s_client_ip;
static uint32_t            s_sta_fail_n;
static QueueHandle_t       s_tx_q;
static esp_timer_handle_t  s_retry_timer;
static app_proto_parser_t  s_parser;        /* reassembly for the TCP byte stream */
static tx_frame_t          s_tx_buf;        /* server task only                   */

/* ---- app_proto sink: called from the supervisor task, must not block ------ */
static void wifi_sink(const uint8_t *frame, size_t len, void *ctx)
{
    (void)ctx;
    if (!s_tx_q || len > APP_PROTO_MAX_FRAME) {
        return;
    }
    tx_frame_t f;
    f.len = (uint16_t)len;
    memcpy(f.data, frame, len);
    /* queue full (peer not reading): drop the frame; the host's parser skips
     * the gap and its request times out */
    (void)xQueueSend(s_tx_q, &f, 0);
}

static void wifi_cmd_cb(const app_cmd_t *cmd, void *ctx)
{
    (void)ctx;
    if (s_bus && s_bus->ctrl_q) {
        xQueueSend(s_bus->ctrl_q, cmd, 0);
    }
}

/* ---- station: stored network, reconnect every 5 s ------------------------- */
static void retry_cb(void *arg)
{
    (void)arg;
    if (s_sta_ssid[0] && s_sta_state != 2) {
        s_sta_state = 1;
        esp_wifi_connect();
    }
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            if (s_sta_ssid[0]) {
                s_sta_state = 1;
                esp_wifi_connect();
            }
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            const wifi_event_sta_disconnected_t *e = data;
            s_sta_ip = 0;
            if (s_sta_ssid[0]) {
                s_sta_state = 1;
                if (s_sta_fail_n++ % 12 == 0) {   /* once a minute while it keeps failing */
                    ESP_LOGW(TAG, "network \"%s\" not joined (reason %d); retrying every 5 s",
                             s_sta_ssid, e ? e->reason : -1);
                }
                esp_timer_stop(s_retry_timer);
                esp_timer_start_once(s_retry_timer, STA_RETRY_US);
            } else {
                s_sta_state = 0;
            }
            break;
        }
        case WIFI_EVENT_AP_STACONNECTED: {
            const wifi_event_ap_staconnected_t *e = data;
            ESP_LOGI(TAG, "device " MACSTR " joined the access point", MAC2STR(e->mac));
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED: {
            const wifi_event_ap_stadisconnected_t *e = data;
            ESP_LOGI(TAG, "device " MACSTR " left the access point", MAC2STR(e->mac));
            break;
        }
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        s_sta_ip = e->ip_info.ip.addr;
        s_sta_state = 2;
        s_sta_fail_n = 0;
        ESP_LOGW(TAG, "joined network \"%s\": address " IPSTR ", TCP port %u", s_sta_ssid,
                 IP2STR(&e->ip_info.ip), (unsigned)s_cfg.tcp_port);
    }
}

static void load_sta(char *ssid, size_t ssid_cap, char *pass, size_t pass_cap)
{
    ssid[0] = pass[0] = '\0';
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    size_t n = ssid_cap;
    if (nvs_get_str(h, "ssid", ssid, &n) != ESP_OK) ssid[0] = '\0';
    n = pass_cap;
    if (nvs_get_str(h, "pass", pass, &n) != ESP_OK) pass[0] = '\0';
    nvs_close(h);
}

static void apply_sta(const char *ssid, const char *pass)
{
    wifi_config_t sta;
    memset(&sta, 0, sizeof(sta));
    size_t sl = strnlen(ssid, sizeof(sta.sta.ssid));
    size_t pl = strnlen(pass, sizeof(sta.sta.password));
    memcpy(sta.sta.ssid, ssid, sl);
    memcpy(sta.sta.password, pass, pl);
    sta.sta.threshold.authmode = pl ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    esp_wifi_set_config(WIFI_IF_STA, &sta);
}

/* ---- TCP server: one client; a new connection replaces the old one -------- */
static bool send_all(int sock, const uint8_t *p, size_t n)
{
    while (n > 0) {
        int w = send(sock, p, n, 0);
        if (w <= 0) {
            return false;
        }
        p += w;
        n -= (size_t)w;
    }
    return true;
}

static void close_client(int *sock)
{
    if (*sock < 0) {
        return;
    }
    shutdown(*sock, SHUT_RDWR);
    close(*sock);
    *sock = -1;
    s_client = false;
    s_client_ip = 0;
    app_proto_set_sink(APP_TR_WIFI, NULL, NULL);
    xQueueReset(s_tx_q);
    app_bus_post_transport_closed(s_bus, APP_TR_WIFI);   /* relock, stop its streams */
    ESP_LOGI(TAG, "TCP client closed");
}

static void set_client_opts(int sock)
{
    int one = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
#ifdef TCP_KEEPIDLE
    int idle = 5, intvl = 2, cnt = 3;   /* a vanished peer is dropped after about 11 s */
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#endif
    struct timeval tv = { .tv_sec = 0, .tv_usec = SEND_TIMEOUT_MS * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

static void server_task(void *arg)
{
    (void)arg;
    int lsock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (lsock < 0) {
        ESP_LOGE(TAG, "socket() failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int one = 1;
    setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(s_cfg.tcp_port),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(lsock, (struct sockaddr *)&addr, sizeof(addr)) != 0 || listen(lsock, 1) != 0) {
        ESP_LOGE(TAG, "bind/listen on port %u failed: errno %d", (unsigned)s_cfg.tcp_port, errno);
        close(lsock);
        vTaskDelete(NULL);
        return;
    }

    int csock = -1;
    uint8_t rx[RX_CHUNK];
    for (;;) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(lsock, &rfds);
        int maxfd = lsock;
        if (csock >= 0) {
            FD_SET(csock, &rfds);
            if (csock > maxfd) maxfd = csock;
        }
        struct timeval tv = { .tv_sec = 0, .tv_usec = SELECT_WAIT_US };
        int r = select(maxfd + 1, &rfds, NULL, NULL, &tv);

        if (r > 0 && FD_ISSET(lsock, &rfds)) {
            struct sockaddr_in peer;
            socklen_t plen = sizeof(peer);
            int ns = accept(lsock, (struct sockaddr *)&peer, &plen);
            if (ns >= 0) {
                if (csock >= 0) {
                    ESP_LOGW(TAG, "a new TCP client replaces the connected one");
                    close_client(&csock);
                }
                set_client_opts(ns);
                csock = ns;
                s_parser.len = 0;
                s_client = true;
                s_client_ip = peer.sin_addr.s_addr;
                xQueueReset(s_tx_q);
                app_proto_set_sink(APP_TR_WIFI, wifi_sink, NULL);
                ESP_LOGW(TAG, "TCP client %s connected", inet_ntoa(peer.sin_addr));
            }
        }
        if (r > 0 && csock >= 0 && FD_ISSET(csock, &rfds)) {
            int n = recv(csock, rx, sizeof(rx), 0);
            if (n > 0) {
                app_proto_feed(&s_parser, APP_TR_WIFI, rx, (size_t)n, wifi_cmd_cb, NULL);
            } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
                close_client(&csock);
            }
        }
        while (csock >= 0 && xQueueReceive(s_tx_q, &s_tx_buf, 0) == pdTRUE) {
            if (!send_all(csock, s_tx_buf.data, s_tx_buf.len)) {
                close_client(&csock);
            }
        }
    }
}

/* ---- public ---------------------------------------------------------------- */
esp_err_t wireless_wifi_start(const app_bus_t *bus, const wireless_wifi_cfg_t *cfg)
{
    if (!bus || !cfg || !cfg->ap_ssid_prefix || !cfg->ap_password) {
        return ESP_ERR_INVALID_ARG;
    }
    s_bus = bus;
    s_cfg = *cfg;

    s_tx_q = xQueueCreate(TX_QUEUE_LEN, sizeof(tx_frame_t));
    const esp_timer_create_args_t targs = { .callback = retry_cb, .name = "wifi_retry" };
    if (!s_tx_q || esp_timer_create(&targs, &s_retry_timer) != ESP_OK) {
        ESP_LOGE(TAG, "out of memory");
        return ESP_ERR_NO_MEM;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    s_ap_netif = esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wcfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init: %s", esp_err_to_name(err));
        return err;
    }
    esp_wifi_set_storage(WIFI_STORAGE_RAM);   /* the stored network lives in our own NVS keys */
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL, NULL);

    /* access point: "<prefix>-XXXX", XXXX = last two bytes of its MAC */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s-%02X%02X", cfg->ap_ssid_prefix, mac[4], mac[5]);
    wifi_config_t ap;
    memset(&ap, 0, sizeof(ap));
    size_t sl = strnlen(s_ap_ssid, sizeof(ap.ap.ssid));
    memcpy(ap.ap.ssid, s_ap_ssid, sl);
    ap.ap.ssid_len = (uint8_t)sl;
    size_t pl = strnlen(cfg->ap_password, sizeof(ap.ap.password) - 1);
    memcpy(ap.ap.password, cfg->ap_password, pl);
    ap.ap.authmode = pl >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    ap.ap.channel = cfg->ap_channel;
    ap.ap.max_connection = 2;

    char ssid[33], pass[65];
    load_sta(ssid, sizeof(ssid), pass, sizeof(pass));
    strncpy(s_sta_ssid, ssid, sizeof(s_sta_ssid) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    if (ssid[0]) {
        apply_sta(ssid, pass);
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    s_started = true;

    if (xTaskCreate(server_task, "wifi_srv", SERVER_STACK, NULL, SERVER_PRIO, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create the TCP server task");
        return ESP_FAIL;
    }
    ESP_LOGW(TAG, "access point \"%s\" (password \"%s\"): connect to 192.168.4.1, TCP port %u%s%s%s",
             s_ap_ssid, pl >= 8 ? cfg->ap_password : "none", (unsigned)cfg->tcp_port,
             ssid[0] ? "; also joining network \"" : "", ssid, ssid[0] ? "\"" : "");
    return ESP_OK;
}

esp_err_t wireless_wifi_set_sta(const char *ssid, const char *password)
{
    if (!s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!ssid || !password || strlen(ssid) > 32 || strlen(password) > 64) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    if (ssid[0]) {
        nvs_set_str(h, "ssid", ssid);
        nvs_set_str(h, "pass", password);
    } else {
        nvs_erase_key(h, "ssid");
        nvs_erase_key(h, "pass");
    }
    nvs_commit(h);
    nvs_close(h);

    esp_timer_stop(s_retry_timer);
    memset(s_sta_ssid, 0, sizeof(s_sta_ssid));
    strncpy(s_sta_ssid, ssid, sizeof(s_sta_ssid) - 1);
    s_sta_fail_n = 0;
    esp_wifi_disconnect();
    apply_sta(ssid, password);
    if (ssid[0]) {
        s_sta_state = 1;
        esp_wifi_connect();
        ESP_LOGW(TAG, "joining network \"%s\" (stored)", ssid);
    } else {
        s_sta_state = 0;
        s_sta_ip = 0;
        ESP_LOGW(TAG, "stored network forgotten; access point only");
    }
    return ESP_OK;
}

bool wireless_wifi_get_info(wireless_wifi_info_t *out)
{
    if (!out) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (!s_started) {
        return false;
    }
    out->started = true;
    strncpy(out->ap_ssid, s_ap_ssid, sizeof(out->ap_ssid) - 1);
    esp_netif_ip_info_t ip;
    if (s_ap_netif && esp_netif_get_ip_info(s_ap_netif, &ip) == ESP_OK) {
        out->ap_ip = ip.ip.addr;
    }
    strncpy(out->sta_ssid, s_sta_ssid, sizeof(out->sta_ssid) - 1);
    out->sta_state = s_sta_state;
    out->sta_ip = s_sta_ip;
    out->client = s_client;
    out->client_ip = s_client_ip;
    return true;
}
