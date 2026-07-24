#include "bsp/esp-bsp.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "lv_demos.h"

static const char *TAG = "lvgl_with_ram";

void app_main(void)
{
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    esp_log_level_t i2c_log_level = esp_log_level_get("i2c.master");
    esp_log_level_set("i2c.master", ESP_LOG_NONE);
#endif
    const bsp_board_variant_t variant = bsp_board_detect();
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    esp_log_level_set("i2c.master", i2c_log_level);
#endif
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
