/**
 * @file app_main.c
 * @brief Firmware entry point — IMU + two-sensor haptic bring-up build.
 *
 * The remaining motor/gripper subsystems sit in
 * components/ but are not started and have no command cases here. ESP-IDF may
 * still compile auto-discovered components into libraries; because nothing
 * references those libraries, they add no tasks or runtime output. That is the
 * useful isolation for bring-up: every application action here belongs to a
 * sensor path. Restoring a subsystem means restoring its start call, supervisor
 * cases, and component dependencies together.
 *
 * Data flow in this build:
 *
 *   IMU --UART--> imu_uart_task --(imu_sample_t)--> imu_q --\
 *                                                            > supervisor_task
 *   host --USB--> transport_uart --(app_cmd_t)-----> ctrl_q -/       |
 *                       ^                                            |
 *                       \------- ACK / telemetry (app_proto) --------/
 *
 * supervisor_task is the sole consumer of imu_q — a queue hands each message
 * to exactly one reader, so nothing here may drain it as well — and the sole
 * place a decoded command becomes a driver call. This file only owns
 * configuration and start-up order.
 *
 * Nothing calls imu_uart_set_angle_ref() at boot. That is the roll/pitch zero
 * (WIT CALSW 0x08, persisted in the unit's own flash), and it matters here:
 * the module is mounted inverted, so without a reference roll sits at ±180°,
 * right on the wrap-around. Send OP_IMU_SET_REF (0x25) once from the host with
 * the gripper held in its home pose and still for ~3.2 s.
 */
#include "driver/uart.h"                /* UART_NUM_x, used by app_config.h */
#include "esp_err.h"
#include "esp_log.h"

#include "app_pins.h"
#include "app_config.h"

#include "board.h"
#include "app_protocol.h"
#include "imu_uart.h"
#include "haptic_uart.h"
#include "app_supervisor.h"
#include "transport_uart.h"

static const char *TAG = "app_main";

/* The message bus is process-wide and long-lived. This build uses imu_q,
 * haptic_q and ctrl_q; motor_status_q remains allocated but idle. */
static app_bus_t s_bus;

void app_main(void)
{
    ESP_LOGI(TAG, "=== gripper_plus firmware starting (IMU + haptic bring-up) ===");

    ESP_ERROR_CHECK(board_init());
    ESP_ERROR_CHECK(app_bus_init(&s_bus));   /* must precede every start below */

    /* ---- IMU UART: produces imu_sample_t onto bus->imu_q ---- */
    const imu_uart_cfg_t imu_cfg = {
        .uart_port   = IMU_UART_PORT,
        .tx_gpio     = PIN_IMU_UART_TX,
        .rx_gpio     = PIN_IMU_UART_RX,
        .baud_rate   = IMU_UART_BOOT_BAUD,
        .run_baud    = IMU_UART_RUN_BAUD,
        .rate_hz     = IMU_OUT_RATE_HZ,
        .rx_buf_size = UART_RX_BUFFER_SIZE,
        .task_stack  = TASK_STACK_IMU,
        .task_prio   = TASK_PRIO_IMU,
    };
    ESP_ERROR_CHECK(imu_uart_start(&imu_cfg, &s_bus));

    /* ---- Two CP-M3025 sensors on one device-addressed UART bus. ---- */
    const haptic_uart_cfg_t haptic_cfg = {
        .uart_port = HAPTIC_UART_PORT,
        .tx_gpio = PIN_HAPTIC_UART_TX,
        .rx_gpio = PIN_HAPTIC_UART_RX,
        .baud_rate = HAPTIC_UART_BAUD,
        .rx_buf_size = UART_RX_BUFFER_SIZE,
        .tx_buf_size = UART_TX_BUFFER_SIZE,
        .task_stack = TASK_STACK_HAPTIC,
        .task_prio = TASK_PRIO_HAPTIC,
        .poll_hz = HAPTIC_POLL_HZ,
        .ring_module = HAPTIC_RING_MODULE,
        .little_module = HAPTIC_LITTLE_MODULE,
    };
    ESP_ERROR_CHECK(haptic_uart_start(&haptic_cfg, &s_bus));

    /* ---- Supervisor: drains sensor queues, executes commands off ctrl_q ---- */
    const app_supervisor_cfg_t supervisor_cfg = {
        .task_stack = TASK_STACK_SUPERVISOR,
        .task_prio  = TASK_PRIO_SUPERVISOR,
    };
    ESP_ERROR_CHECK(app_supervisor_start(&s_bus, &supervisor_cfg));

    /* ---- User transport: app_proto over the native USB-Serial/JTAG.
     * Started last so the supervisor is already draining ctrl_q by the time a
     * host can send anything. ---- */
    ESP_ERROR_CHECK(transport_uart_start(&s_bus));

    ESP_LOGI(TAG, "=== IMU %d Hz; tactile 2 x 7x11 @ %d Hz; app_proto on USB ===",
             IMU_OUT_RATE_HZ, HAPTIC_POLL_HZ);
    /* app_main returns; FreeRTOS tasks keep running. */
}
