#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "lvgl.h"

static const char *TAG = "bsp_quickstart";

static void touch_event_cb(lv_event_t *event)
{
    lv_obj_t *status = lv_event_get_user_data(event);
    lv_label_set_text(status, "Touch input detected");
    ESP_LOGI(TAG, "Touch input detected");
}

void app_main(void)
{
    const bsp_board_variant_t variant = bsp_board_detect();
    ESP_LOGI(TAG, "Starting %s", bsp_board_variant_to_name(variant));

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

    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101418), LV_PART_MAIN);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "ESP32-C6 Touch AMOLED 1.8");
    lv_obj_set_style_text_color(title, lv_color_hex(0xF4F7F8), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 54);

    lv_obj_t *variant_label = lv_label_create(screen);
    lv_label_set_text_fmt(variant_label, "%s\n368 x 448", bsp_board_variant_to_name(variant));
    lv_obj_set_style_text_align(variant_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_set_style_text_color(variant_label, lv_color_hex(0x8FD3C7), LV_PART_MAIN);
    lv_obj_align(variant_label, LV_ALIGN_CENTER, 0, -42);

    lv_obj_t *status = lv_label_create(screen);
    lv_label_set_text(status, bsp_display_get_input_dev() != NULL ? "Touch is ready" : "Touch is unavailable");
    lv_obj_set_style_text_color(status, lv_color_hex(0xD8DEE1), LV_PART_MAIN);
    lv_obj_align(status, LV_ALIGN_CENTER, 0, 42);

    lv_obj_t *button = lv_button_create(screen);
    lv_obj_set_size(button, 180, 58);
    lv_obj_align(button, LV_ALIGN_BOTTOM_MID, 0, -62);
    lv_obj_add_event_cb(button, touch_event_cb, LV_EVENT_CLICKED, status);

    lv_obj_t *button_label = lv_label_create(button);
    lv_label_set_text(button_label, "Touch");
    lv_obj_center(button_label);

    bsp_display_unlock();
    ESP_LOGI(TAG, "BSP quick start is ready");
}
