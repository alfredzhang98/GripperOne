/**
 * @file transport_uart.h
 * @brief UART (USB-Serial/JTAG) transport for the app_proto user layer.
 *
 * One of the three interchangeable transports (UART / BLE / Wi-Fi). Each does
 * the same two things: feed received bytes into app_proto_feed() (decoded
 * commands go to bus->ctrl_q) and register an app_proto sink so replies and
 * telemetry go back out. On the XIAO ESP32-C6 this uses the native USB-C
 * (USB-Serial/JTAG), which the PC sees as a COM port.
 */
#ifndef TRANSPORT_UART_H
#define TRANSPORT_UART_H

#include "app_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Bring up the USB-Serial/JTAG transport and its RX task. */
int transport_uart_start(const app_bus_t *bus);

#ifdef __cplusplus
}
#endif

#endif /* TRANSPORT_UART_H */
