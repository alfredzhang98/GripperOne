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
} imu_uart_cfg_t;

/**
 * @brief Install the UART driver and spawn imu_uart_task.
 * @return ESP_OK on success.
 */
int imu_uart_start(const imu_uart_cfg_t *cfg, const app_bus_t *bus);

#ifdef __cplusplus
}
#endif

#endif /* IMU_UART_H */
