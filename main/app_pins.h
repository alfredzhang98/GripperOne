/**
 * @file app_pins.h
 * @brief Central pin map for the Seeed Studio XIAO ESP32-C6 controller.
 *
 * Every hardware GPIO used by the firmware is declared here and ONLY here.
 * Component code must never hard-code a GPIO number; it includes this header
 * instead. This keeps the board wiring in a single, reviewable place.
 *
 * Verified against the project schematic (esp32_c6_xiao / Sub1). "UART1" and
 * "UART2" below are the schematic's logical link names, NOT ESP32 controller
 * numbers (the C6 has two HP controllers, UART0 and UART1; either can drive
 * any pins through the GPIO matrix). Mapping: haptic link = schematic UART1,
 * IMU link = schematic UART2. See app_config.h for the controller assignment.
 *
 * Reference (XIAO ESP32-C6 silkscreen -> chip GPIO, from the schematic):
 *
 *   Schematic net   XIAO pin   GPIO   Peripheral / link
 *   --------------  ---------  -----  -----------------------------
 *   (USB-C)         USB-C      inty   USB Serial/JTAG (debug/download)
 *   UART2_rx        D0          0     IMU link RX  (XIAO <- IMU TX)
 *   UART2_tx        D1          1     IMU link TX  (XIAO -> IMU RX)
 *   TWAI TX         D2          2     Motor CAN TX (-> transceiver)
 *   TWAI RX         D3         21     Motor CAN RX (<- transceiver)
 *   (free)          D4         22     (was OLED I2C SDA, removed)
 *   (free)          D5         23     (was OLED I2C SCL, removed)
 *   UART1_tx        D6         16     Haptic link TX (XIAO -> board RX)
 *   UART1_rx        D7         17     Haptic link RX (XIAO <- board TX)
 */
#ifndef APP_PINS_H
#define APP_PINS_H

/* -------------------------------------------------------------------------
 * Haptic / pressure sensor board  (high-speed UART link)
 * TX/RX are named from the XIAO's point of view:
 *   XIAO TX (D6, GPIO16) -> sensor RX
 *   XIAO RX (D7, GPIO17) <- sensor TX
 * ------------------------------------------------------------------------- */
#define PIN_HAPTIC_UART_TX      16  /* XIAO D6 */
#define PIN_HAPTIC_UART_RX      17  /* XIAO D7 */

/* -------------------------------------------------------------------------
 * IMU board (JY901P / MPU9250+ with on-board fusion), UART link.
 *   XIAO TX (D1, GPIO1) -> IMU RX
 *   XIAO RX (D0, GPIO0) <- IMU TX
 * Routed through the GPIO matrix like every ESP32 UART.
 * ------------------------------------------------------------------------- */
#define PIN_IMU_UART_TX          1  /* XIAO D1 */
#define PIN_IMU_UART_RX          0  /* XIAO D0 */

/* -------------------------------------------------------------------------
 * D4 (GPIO22) / D5 (GPIO23) are free (were the SH1106 OLED I2C, now removed).
 * ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * Motor control via TWAI (CAN 2.0).
 * NOTE: the ESP32-C6 has NO on-chip CAN transceiver. An external 3.3 V CAN
 * transceiver (e.g. TCAN332 / SN65HVD230) MUST sit between these pins and
 * the bus.
 *   TWAI TX -> transceiver TXD  (XIAO D2, GPIO2)
 *   TWAI RX <- transceiver RXD  (XIAO D3, GPIO21)
 * ------------------------------------------------------------------------- */
#define PIN_MOTOR_TWAI_TX        2  /* XIAO D2 */
#define PIN_MOTOR_TWAI_RX       21  /* XIAO D3 */

#endif /* APP_PINS_H */
