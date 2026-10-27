/**
 * @file board.h
 * @brief Board bring-up for the Seeed Studio XIAO ESP32-C6.
 *
 * Handles the one-time, board-level initialisation that is not tied to any
 * single sensor: NVS flash (needed by BLE/Wi-Fi) and a boot banner. Keeping
 * this in a component means main/ stays free of chip-specific boilerplate.
 */
#ifndef BOARD_H
#define BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise board-level services (NVS flash, log banner).
 * @return ESP_OK on success.
 */
int board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
