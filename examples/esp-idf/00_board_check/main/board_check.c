#include <stdint.h>

#include "bsp/esp-bsp.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char *TAG = "board_check";

void app_main(void)
{
    esp_chip_info_t chip_info = {0};
    uint32_t flash_size = 0;

    esp_chip_info(&chip_info);
    ESP_ERROR_CHECK(esp_flash_get_size(NULL, &flash_size));

    ESP_LOGI(TAG, "ESP32-C6-Touch-AMOLED-1.8");
    ESP_LOGI(TAG, "Chip: %d core(s), revision %d", chip_info.cores, chip_info.revision);
    ESP_LOGI(TAG, "Flash: %lu MB", (unsigned long)(flash_size / (1024 * 1024)));
    ESP_LOGI(TAG, "PSRAM: %lu bytes", (unsigned long)heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Display: %d x %d", BSP_LCD_H_RES, BSP_LCD_V_RES);

    const bsp_board_variant_t variant = bsp_board_detect();
    ESP_LOGI(TAG, "Board variant: %s", bsp_board_variant_to_name(variant));
    ESP_LOGI(TAG, "I2C: port %d, SDA GPIO %d, SCL GPIO %d",
             BSP_I2C_NUM, BSP_I2C_SDA, BSP_I2C_SCL);
    ESP_LOGI(TAG, "Capabilities: display=%d touch=%d audio=%d SD=%d IMU=%d",
             BSP_CAPS_DISPLAY, BSP_CAPS_TOUCH, BSP_CAPS_AUDIO,
             BSP_CAPS_SDCARD, BSP_CAPS_IMU);

    if (flash_size != 16 * 1024 * 1024) {
        ESP_LOGW(TAG, "Expected 16 MB flash for this board");
    }
    if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) != 0) {
        ESP_LOGW(TAG, "This board is expected to operate without PSRAM");
    }
    if (variant == BSP_BOARD_VARIANT_UNKNOWN) {
        ESP_LOGE(TAG, "No supported touch controller was detected");
    } else {
        ESP_LOGI(TAG, "Board check complete");
    }
}
