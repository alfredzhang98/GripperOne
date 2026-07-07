/**
 * @file haptic_uart.h
 * @brief High-speed UART link to the haptic / pressure sensor board.
 *
 * Owns one UART controller, its RX ring buffer, and a task that turns raw
 * bytes into haptic_sample_t messages posted to bus->haptic_q.
 *
 * The real wire format is the vendor "通信盒通信协议 V1.0.5" (see
 * source_files/hand_sensor_uart/). This scaffold ships a *placeholder* parser
 * that recognises the frame heads and validates the LRC checksum, but does
 * not yet decode the payload. Search for TODO markers in haptic_uart.c.
 */
#ifndef HAPTIC_UART_H
#define HAPTIC_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int      uart_port;    /* UART_NUM_x */
    int      tx_gpio;
    int      rx_gpio;
    int      baud_rate;
    int      rx_buf_size;  /* driver RX ring buffer, bytes */
    int      tx_buf_size;  /* driver TX ring buffer, bytes (0 = blocking TX) */
} haptic_uart_cfg_t;

/**
 * @brief Install the UART driver and spawn haptic_uart_task.
 * @param cfg  hardware configuration (copied internally)
 * @param bus  message bus; decoded samples are posted to bus->haptic_q
 * @return ESP_OK on success.
 */
int haptic_uart_start(const haptic_uart_cfg_t *cfg, const app_bus_t *bus);

/** @brief Select auto-push content (bit0 = resultant force, bit1 = distributed). */
void haptic_uart_set_content(uint8_t content_bits);

/** @brief Enable/disable the sensor's auto-push data stream (register 0x0017). */
void haptic_uart_stream(bool enable);

/** @brief Zero/tare the tactile modules (vendor calibration command). */
void haptic_uart_calibrate(void);

#ifdef __cplusplus
}
#endif

#endif /* HAPTIC_UART_H */
