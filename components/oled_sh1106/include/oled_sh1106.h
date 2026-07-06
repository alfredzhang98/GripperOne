/**
 * @file oled_sh1106.h
 * @brief Optional SH1106 1.3" OLED over I2C.
 *
 * This whole component is only started when the caller enables it
 * (APP_ENABLE_OLED in main/app_config.h). It owns the I2C master bus, a small
 * SH1106 command set, and oled_task, which periodically renders a status page
 * (system state + connection + a couple of sensor values) pulled from the bus.
 *
 * The display driver here is a *skeleton*: it initialises the panel and can
 * push a full framebuffer, but the text/rendering helpers are stubs. See TODO
 * markers in oled_sh1106.c.
 */
#ifndef OLED_SH1106_H
#define OLED_SH1106_H

#include <stdint.h>
#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      i2c_port;
    int      sda_gpio;
    int      scl_gpio;
    uint32_t freq_hz;
    uint8_t  addr;      /* 7-bit I2C address, typically 0x3C */
} oled_cfg_t;

/**
 * @brief Bring up the I2C bus + SH1106 panel and spawn oled_task.
 *
 * @param cfg    display configuration
 * @param bus    message bus (peeked read-only for status values)
 * @param state  pointer to the supervisor's current state (read-only).
 *               May be NULL to show "?".
 * @return ESP_OK on success.
 */
int oled_start(const oled_cfg_t *cfg, const app_bus_t *bus,
               const volatile app_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* OLED_SH1106_H */
