# LVGL with internal RAM

This example starts LVGL through the managed
waveshare/esp32_c6_touch_amoled_1_8 BSP and opens the LVGL music demo.

The board has no PSRAM. Display buffers are allocated from internal RAM, and
the BSP automatically selects SH8601/FT5x06 for V1 or CO5300/CST820 for V2.
Compilation and firmware packaging are validated by the repository CI.
