#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "lv_demos.h"

static const char *TAG = "lvgl_with_ram";

void app_main(void)
{
    const bsp_board_variant_t variant = bsp_board_detect();
    ESP_LOGI(TAG, "Starting LVGL on %s", bsp_board_variant_to_name(variant));

    lv_display_t *display = bsp_display_start();
    if (display == NULL) {
        ESP_LOGE(TAG, "Display initialization failed");
        return;
    }
    ESP_ERROR_CHECK(bsp_display_backlight_on());

    if (!bsp_display_lock(0)) {
        ESP_LOGE(TAG, "Failed to lock LVGL");
        return;
    }
    lv_demo_music();
    bsp_display_unlock();

    ESP_LOGI(TAG, "LVGL music demo is ready");
}
