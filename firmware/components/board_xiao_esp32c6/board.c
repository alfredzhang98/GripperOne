#include "board.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "board";

int board_init(void)
{
    ESP_LOGI(TAG, "XIAO ESP32-C6 board init");

    /* NVS is required by the BLE/Wi-Fi stacks and is generally useful for
     * storing calibration and user settings later. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "erasing and re-initialising NVS (%s)", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    return ESP_OK;
}
