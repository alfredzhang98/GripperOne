/**
 * @file haptic_uart.h
 * @brief Driver for two PaXini CP-M3025-Core tactile modules read through the
 *        PX-6AX GEN3 high-speed communication integration board (高速通信集成板).
 *
 * The ESP is wired to the board's UART header, so it speaks the board's
 * register protocol (通信盒通信协议 section 4.4), not the sensor-direct
 * protocol from section 5.4 of the sensor manual. A module is identified by
 * its slot index in the board's fixed 28-module ordering, not by a bus
 * address: thumb / index / middle / ring / little, each as
 * proximal, middle, fingertip, nail. A CP-M3025 is a proximal module.
 */
#ifndef HAPTIC_UART_H
#define HAPTIC_UART_H

#include <stdbool.h>
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
    int tx_buf_size;
    int task_stack;
    int task_prio;
    int poll_hz;           /* complete samples requested per sensor/s        */
    uint8_t ring_module;   /* board module index, left GUI panel  (12 = 无名指近节) */
    uint8_t little_module; /* board module index, right GUI panel (16 = 小拇指近节) */
} haptic_uart_cfg_t;

/** Install UART1, run board discovery, and start polling both 7 x 11 sensors. */
int haptic_uart_start(const haptic_uart_cfg_t *cfg, const app_bus_t *bus);

/** Independently control board -> ESP polling; does not change USB reporting. */
bool haptic_uart_set_acquisition(bool enable);
bool haptic_uart_acquisition_enabled(void);

/** Schedule a board-wide tare (function 0x17, register 0x0002 = 1). */
bool haptic_uart_calibrate(void);

#ifdef __cplusplus
}
#endif

#endif /* HAPTIC_UART_H */
