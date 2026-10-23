/**
 * @file app_main.c
 * @brief Firmware entry point — IMU, two tactile pads, DM-J4310 gripper motor.
 *
 * Data flow:
 *
 *   IMU   --UART--> imu_uart_task    --(imu_sample_t)----> imu_q ---------\
 *   pads  --UART--> haptic_uart_task --(haptic_sample_t)-> haptic_q ------+
 *   motor --TWAI--> motor_twai RX    --(motor_status_t)--> motor_status_q +--> supervisor_task
 *   host  --USB---> transport_uart   --(app_cmd_t)-------> ctrl_q --------/     |
 *                        ^                                       gripper_tick() |
 *                        \--- ACK / telemetry (app_proto) ---   force-position  v
 *                                                               setpoint -> motor_twai_task
 *
 * supervisor_task is the sole consumer of every queue — a queue hands each
 * message to exactly one reader — and the sole place a decoded command becomes
 * a driver call. This file only owns configuration and start-up order.
 * wireless_ble / wireless_wifi are not started (USB transport only).
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
#include "motor_twai.h"
#include "gripper.h"
#include "app_supervisor.h"
#include "transport_uart.h"

static const char *TAG = "app_main";

/* The message bus is process-wide and long-lived. */
static app_bus_t s_bus;

void app_main(void)
{
    ESP_LOGI(TAG, "=== gripper_plus firmware starting (IMU + haptic + gripper) ===");

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

    /* ---- Motor: TWAI driver + probe task. It commands no motion until the
     * probe has verified the motor's registers, and only the gripper (via
     * the supervisor) ever sets a setpoint. ---- */
    const motor_twai_cfg_t motor_cfg = {
        .tx_gpio      = PIN_MOTOR_TWAI_TX,
        .rx_gpio      = PIN_MOTOR_TWAI_RX,
        .bitrate_kbps = MOTOR_TWAI_BITRATE_KBPS,
        .esc_id       = MOTOR_TWAI_ESC_ID,
        .mst_id       = MOTOR_TWAI_MST_ID,
        .timeout_ms   = MOTOR_TWAI_TIMEOUT_MS,
        .task_stack   = TASK_STACK_MOTOR,
        .task_prio    = TASK_PRIO_MOTOR,
    };
    ESP_ERROR_CHECK(motor_twai_start(&motor_cfg, &s_bus));

    /* ---- Gripper state machine (ticked by the supervisor). ---- */
    ESP_ERROR_CHECK(gripper_init(GRIPPER_AUTOCAL_ON_BOOT != 0));

    /* ---- Supervisor: drains every queue, executes commands off ctrl_q,
     * runs the gripper control loop ---- */
    const app_supervisor_cfg_t supervisor_cfg = {
        .task_stack  = TASK_STACK_SUPERVISOR,
        .task_prio   = TASK_PRIO_SUPERVISOR,
        .overforce_n = GRIPPER_OVERFORCE_N,
    };
    ESP_ERROR_CHECK(app_supervisor_start(&s_bus, &supervisor_cfg));

    /* ---- User transport: app_proto over the native USB-Serial/JTAG.
     * Started last so the supervisor is already draining ctrl_q by the time a
     * host can send anything. ---- */
    ESP_ERROR_CHECK(transport_uart_start(&s_bus));

    ESP_LOGI(TAG, "=== IMU %d Hz; tactile 2 x 7x11 @ %d Hz; motor TWAI %d kbps; app_proto on USB ===",
             IMU_OUT_RATE_HZ, HAPTIC_POLL_HZ, MOTOR_TWAI_BITRATE_KBPS);
    /* app_main returns; FreeRTOS tasks keep running. */
}
