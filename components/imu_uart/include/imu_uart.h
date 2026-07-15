/**
 * @file imu_uart.h
 * @brief UART link to the IMU (JY901P / MPU9250+ with on-board fusion).
 *
 * Owns one UART controller and a task that decodes IMU frames into
 * imu_sample_t messages posted to bus->imu_q.
 *
 * The JY901P (WitMotion) protocol uses 11-byte frames beginning with 0x55 and
 * a type byte (0x53 = angle, 0x59 = quaternion). This scaffold provides a
 * placeholder decoder; see TODO markers in imu_uart.c.
 */
#ifndef IMU_UART_H
#define IMU_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int uart_port;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;
    int rx_buf_size;
    int task_stack;   /* task stack depth, bytes (0 = built-in default) */
    int task_prio;    /* FreeRTOS priority       (0 = built-in default) */
} imu_uart_cfg_t;

/**
 * @brief Install the UART driver and spawn imu_uart_task.
 * @return ESP_OK on success.
 */
int imu_uart_start(const imu_uart_cfg_t *cfg, const app_bus_t *bus);

/**
 * @brief Low-level WIT register write: auto-unlock -> write reg -> optional SAVE.
 * @param addr  register address (e.g. 0x02 RSW, 0x03 RRATE, 0x04 BAUD).
 * @param val   16-bit value (sent little-endian as dataL, dataH).
 * @param save  true to persist via the SAVE register afterwards.
 *
 * Unlock is handled internally (the unit auto-relocks after ~10 s idle), so the
 * caller never manages it. Use this for any register the named helpers below do
 * not cover.
 */
void imu_uart_write_reg(uint8_t addr, uint16_t val, bool save);

/* --- calibration (CALSW register 0x01) --- */
/** @brief Accelerometer calibration (keep still and level); saved. */
void imu_uart_cal_accel(void);
/** @brief Yaw/Z-axis zero. Only valid under the 6-axis algorithm; refused (no-op)
 *  in 9-axis mode. Call imu_uart_set_axis(true) first. */
void imu_uart_set_zero(void);
/** @brief Set current roll/pitch as the angle reference (relative zero). */
void imu_uart_set_angle_ref(void);
/** @brief Enter spherical magnetic calibration; rotate, then cal_end() (persists). */
void imu_uart_cal_mag(void);
/** @brief Exit any calibration mode (normal working mode). */
void imu_uart_cal_end(void);

/* --- gyro calibration --- */
/** @brief Auto gyro-zeroing on stillness: true=on (default), false=off. */
void imu_uart_gyro_autocal(bool enable);
/** @brief Manual gyro cal (disable auto first): begin acquiring zero bias, hold
 *  still a while, then gyro_cal_end(). */
void imu_uart_gyro_cal_begin(void);
/** @brief End manual gyro cal and persist. */
void imu_uart_gyro_cal_end(void);

/* --- algorithm / mounting --- */
/** @brief 6-axis (accel+gyro, magnetic-interference-immune) vs 9-axis (adds mag). */
void imu_uart_set_axis(bool six_axis);
/** @brief Last-set algorithm mode (true=6-axis); see imu_uart_refresh_axis(). */
bool imu_uart_is_six_axis(void);
/** @brief Read the unit's actual AXIS6 mode, cache and return it (true=6-axis). */
bool imu_uart_refresh_axis(void);
/** @brief Mounting: false=horizontal, true=vertical (Y axis up). */
void imu_uart_set_orient(bool vertical);

/* --- output / link configuration (persisted) --- */
/** @brief Set RSW (0x02): which 0x5x frames the unit streams (bit3 angle, bit9 quat). */
void imu_uart_set_content(uint16_t rsw);
/** @brief Set RRATE (0x03): output rate code (0x06=10Hz 0x07=20Hz 0x08=50Hz 0x09=100Hz). */
void imu_uart_set_rate(uint8_t code);
/** @brief Set BAUD (0x04): 0x02=9600 ... 0x06=115200. Queued and run by the IMU
 *  task, which also switches this host UART to match — no caller action needed. */
void imu_uart_set_baud(uint8_t code);

/* --- filtering / bandwidth (persisted) --- */
/** @brief Set BANDWIDTH (0x1F): 0=256Hz 1=188 2=98 3=42 4=20 5=10 6=5 Hz. */
void imu_uart_set_bandwidth(uint8_t code);
/** @brief Set FILTK (0x25) angle filter: 1..10000 (default 30; smaller=trust gyro). */
void imu_uart_set_filtk(uint16_t k);
/** @brief Set ACCFILT (0x2A) accel filter: 1..10000 (default 500; smaller=steadier). */
void imu_uart_set_accfilt(uint16_t v);

/* --- power / misc --- */
/** @brief Power-on auto data output (POWONSEND 0x2D). */
void imu_uart_set_powon_output(bool on);
/** @brief Enter sleep (any serial byte wakes it). Session-only. */
void imu_uart_sleep(bool enable);
/** @brief LED on/off (LEDOFF 0x1B; some newer units ignore it). */
void imu_uart_led(bool on);

/* --- register read (call from a task OTHER than the IMU task) --- */
/** @brief Read up to 4 consecutive regs from addr into out[]; 0 ok, -1 timeout. */
int imu_uart_read_reg(uint8_t addr, uint16_t *out, int count);

/* --- system (SAVE register 0x00) ---
 * Unlock and plain "save current config" are internal primitives (static in the
 * .c, applied via write_reg's `save` flag); they are deliberately not exposed. */
/** @brief Soft-reboot the unit. */
void imu_uart_reboot(void);
/** @brief Restore the unit to factory defaults. */
void imu_uart_factory_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_UART_H */
