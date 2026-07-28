/**
 * @file app_main.c
 * @brief Firmware entry point for the gripper_plus controller (XIAO ESP32-C6).
 *
 * Responsibilities kept deliberately thin:
 *   1. Bring up the board (NVS, banner).
 *   2. Create the inter-task message bus.
 *   3. Hand each driver its pin/baud config (from app_pins.h / app_config.h)
 *      and start it.
 *   4. Start the supervisor state machine and the selected user transport.
 *
 * All hardware logic lives in components/. This file only owns configuration
 * and wiring.
 *
 * Data flow:
 *   haptic_uart_task --\
 *   imu_uart_task -----+--> [bus queues] --> supervisor_task --> gripper --> motor_twai
 *   motor_twai_task --/                           ^
 *   transport (UART/BLE/WiFi) --> ctrl_q ---------+
 */
#include "esp_log.h"
#include "driver/uart.h"          /* for UART_NUM_x used in app_config.h */

#include "app_pins.h"
#include "app_config.h"

#include "board.h"
#include "app_protocol.h"
#include "haptic_uart.h"
#include "imu_uart.h"
#include "motor_twai.h"
#include "gripper.h"
#include "app_supervisor.h"

#include "transport_uart.h"
#include "wireless_ble.h"
#include "wireless_wifi.h"

static const char *TAG = "app_main";

/* The message bus is process-wide and long-lived. */
static app_bus_t s_bus;

void app_main(void)
{
    ESP_LOGI(TAG, "=== gripper_plus firmware starting ===");

    ESP_ERROR_CHECK(board_init());
    ESP_ERROR_CHECK(app_bus_init(&s_bus));

    /* ---- Haptic / pressure high-speed UART ---- */
    const haptic_uart_cfg_t haptic_cfg = {
        .uart_port   = HAPTIC_UART_PORT,
        .tx_gpio     = PIN_HAPTIC_UART_TX,
        .rx_gpio     = PIN_HAPTIC_UART_RX,
        .baud_rate   = HAPTIC_UART_BAUD,
        .rx_buf_size = UART_RX_BUFFER_SIZE,
        .tx_buf_size = UART_TX_BUFFER_SIZE,
        .task_stack  = TASK_STACK_HAPTIC,
        .task_prio   = TASK_PRIO_HAPTIC,
    };
    ESP_ERROR_CHECK(haptic_uart_start(&haptic_cfg, &s_bus));

    /* ---- IMU UART ---- */
    const imu_uart_cfg_t imu_cfg = {
        .uart_port   = IMU_UART_PORT,
        .tx_gpio     = PIN_IMU_UART_TX,
        .rx_gpio     = PIN_IMU_UART_RX,
        .baud_rate   = IMU_UART_BAUD,
        .rx_buf_size = UART_RX_BUFFER_SIZE,
        .task_stack  = TASK_STACK_IMU,
        .task_prio   = TASK_PRIO_IMU,
    };
    ESP_ERROR_CHECK(imu_uart_start(&imu_cfg, &s_bus));

    /* ---- Motor control over TWAI (needs external CAN transceiver) ---- */
    const motor_twai_cfg_t motor_cfg = {
        .tx_gpio      = PIN_MOTOR_TWAI_TX,
        .rx_gpio      = PIN_MOTOR_TWAI_RX,
        .bitrate_kbps = MOTOR_TWAI_BITRATE_KBPS,
        .esc_id       = MOTOR_TWAI_ESC_ID,
        .mst_id       = MOTOR_TWAI_MST_ID,
        .task_stack   = TASK_STACK_MOTOR,
        .task_prio    = TASK_PRIO_MOTOR,
    };
    ESP_ERROR_CHECK(motor_twai_start(&motor_cfg, &s_bus));

    /* ---- High-level gripper app (force-position mode, loads calib from flash) ---- */
    ESP_ERROR_CHECK(gripper_init());

    /* ---- Supervisor state machine (drains sensors, ticks the gripper) ---- */
    const app_supervisor_cfg_t supervisor_cfg = {
        .task_stack  = TASK_STACK_SUPERVISOR,
        .task_prio   = TASK_PRIO_SUPERVISOR,
        .overforce_n = GRIPPER_OVERFORCE_N,
    };
    ESP_ERROR_CHECK(app_supervisor_start(&s_bus, &supervisor_cfg));

    /* ---- User-layer transport (one of UART / BLE / Wi-Fi; see APP_TRANSPORT) ---- */
#if   APP_TRANSPORT == TRANSPORT_BLE
    ESP_ERROR_CHECK(wireless_ble_start(&s_bus, BLE_DEVICE_NAME));
#elif APP_TRANSPORT == TRANSPORT_WIFI
    ESP_ERROR_CHECK(wireless_wifi_start(&s_bus));
#else  /* TRANSPORT_UART (default): app_proto over the native USB-Serial/JTAG */
    ESP_ERROR_CHECK(transport_uart_start(&s_bus));
#endif

    ESP_LOGI(TAG, "=== all modules started ===");
    /* app_main returns; FreeRTOS tasks keep running. */
}
