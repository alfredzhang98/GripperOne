#include "oled_sh1106.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "oled_sh1106";

/* SH1106 is 132x64 controller RAM; the visible panel is 128x64 with a 2-px
 * column offset. Framebuffer is 8 pages of 128 columns. */
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_PAGES      (OLED_HEIGHT / 8)
#define OLED_COL_OFFSET 2

#define OLED_CTRL_CMD   0x00
#define OLED_CTRL_DATA  0x40

typedef struct {
    oled_cfg_t                 cfg;
    const app_bus_t           *bus;
    const volatile app_state_t *state;
    i2c_master_bus_handle_t    bus_handle;
    i2c_master_dev_handle_t    dev_handle;
    uint8_t                    fb[OLED_PAGES][OLED_WIDTH];
} oled_ctx_t;

static esp_err_t oled_write_cmd(oled_ctx_t *c, uint8_t cmd)
{
    uint8_t buf[2] = { OLED_CTRL_CMD, cmd };
    return i2c_master_transmit(c->dev_handle, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

/* SH1106 power-on init sequence (standard). */
static esp_err_t oled_panel_init(oled_ctx_t *c)
{
    static const uint8_t init_seq[] = {
        0xAE,             /* display off */
        0xD5, 0x80,       /* clock div */
        0xA8, 0x3F,       /* multiplex = 63 */
        0xD3, 0x00,       /* display offset */
        0x40,             /* start line 0 */
        0xAD, 0x8B,       /* charge pump on (SH1106) */
        0xA1,             /* segment remap */
        0xC8,             /* COM scan dec */
        0xDA, 0x12,       /* COM pins */
        0x81, 0x80,       /* contrast */
        0xD9, 0x22,       /* pre-charge */
        0xDB, 0x40,       /* VCOM detect */
        0xA4,             /* resume from RAM */
        0xA6,             /* normal (not inverted) */
        0xAF,             /* display on */
    };
    for (size_t i = 0; i < sizeof(init_seq); i++) {
        esp_err_t err = oled_write_cmd(c, init_seq[i]);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

/* Push the framebuffer to the panel, page by page (SH1106 has no
 * horizontal-addressing auto-wrap, so we set the column each page). */
static void oled_flush(oled_ctx_t *c)
{
    for (int page = 0; page < OLED_PAGES; page++) {
        oled_write_cmd(c, 0xB0 | page);                       /* page address    */
        oled_write_cmd(c, 0x00 | (OLED_COL_OFFSET & 0x0F));   /* lower col        */
        oled_write_cmd(c, 0x10 | (OLED_COL_OFFSET >> 4));     /* higher col       */

        uint8_t line[1 + OLED_WIDTH];
        line[0] = OLED_CTRL_DATA;
        memcpy(&line[1], c->fb[page], OLED_WIDTH);
        i2c_master_transmit(c->dev_handle, line, sizeof(line), pdMS_TO_TICKS(100));
    }
}

/*
 * TODO: add a small 5x7 font and a text renderer so the status page shows
 * readable strings. For now the framebuffer is cleared/striped as a liveness
 * indicator and the human-readable status is logged instead.
 */
static void oled_render_status(oled_ctx_t *c,
                               app_state_t st,
                               const haptic_sample_t *h,
                               const imu_sample_t *imu)
{
    memset(c->fb, 0x00, sizeof(c->fb));
    /* Simple visual heartbeat: top page filled so we can see the panel is
     * alive without a font. */
    memset(c->fb[0], 0xFF, OLED_WIDTH);

    float fz0 = (h && h->module_count > 0) ? h->force[0].fz : 0.0f;
    ESP_LOGI(TAG, "OLED page: state=%s modules=%u fz0=%.2fN yaw=%.1f",
             app_state_name(st),
             h ? h->module_count : 0,
             fz0,
             imu ? imu->yaw : 0.0f);
}

static void oled_task(void *arg)
{
    oled_ctx_t *c = (oled_ctx_t *)arg;
    ESP_LOGI(TAG, "oled_task running (I2C%d addr 0x%02X)", c->cfg.i2c_port, c->cfg.addr);

    haptic_sample_t last_haptic = {0};
    imu_sample_t    last_imu    = {0};

    for (;;) {
        /* Peek latest values without consuming the supervisor's copies:
         * a real design might use a shared snapshot; here we just try to read
         * whatever is queued so the display reflects recent data. */
        xQueuePeek(c->bus->haptic_q, &last_haptic, 0);
        xQueuePeek(c->bus->imu_q, &last_imu, 0);

        app_state_t st = c->state ? *c->state : APP_STATE_INIT;
        oled_render_status(c, st, &last_haptic, &last_imu);
        oled_flush(c);

        vTaskDelay(pdMS_TO_TICKS(250));  /* 4 Hz refresh */
    }
}

int oled_start(const oled_cfg_t *cfg, const app_bus_t *bus,
               const volatile app_state_t *state)
{
    if (!cfg || !bus) {
        return ESP_ERR_INVALID_ARG;
    }

    oled_ctx_t *c = calloc(1, sizeof(oled_ctx_t));
    if (!c) {
        return ESP_ERR_NO_MEM;
    }
    c->cfg   = *cfg;
    c->bus   = bus;
    c->state = state;

    i2c_master_bus_config_t bus_cfg = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .i2c_port                     = cfg->i2c_port,
        .sda_io_num                   = cfg->sda_gpio,
        .scl_io_num                   = cfg->scl_gpio,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &c->bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->addr,
        .scl_speed_hz    = cfg->freq_hz,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(c->bus_handle, &dev_cfg, &c->dev_handle));

    esp_err_t err = oled_panel_init(c);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SH1106 init failed (%s) - is the panel connected?",
                 esp_err_to_name(err));
        /* Do not abort the whole firmware over an optional display. */
    }

    uint32_t   stack = cfg->task_stack > 0 ? (uint32_t)cfg->task_stack : 3072;
    UBaseType_t prio = cfg->task_prio  > 0 ? (UBaseType_t)cfg->task_prio : 3;
    if (xTaskCreate(oled_task, "oled", stack, c, prio, NULL) != pdPASS) {
        ESP_LOGE(TAG, "failed to create oled_task");
        free(c);
        return ESP_FAIL;
    }
    return ESP_OK;
}
