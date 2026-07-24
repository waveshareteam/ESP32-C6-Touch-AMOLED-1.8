# Example index

## ESP-IDF

Start with 00_board_check for serial output, then use 00_bsp_quickstart for
display and touch validation. Both board revisions use the same projects
because the BSP detects V1 or V2 automatically.

| Path | Purpose |
| --- | --- |
| esp-idf/00_board_check | Board, memory, capabilities, and hardware revision |
| esp-idf/00_bsp_quickstart | Display, touch, brightness, and LVGL |
| esp-idf/01_AXP2101 | AXP2101 PMU |
| esp-idf/02_PCF85063 | PCF85063A RTC |
| esp-idf/03_esp-brookesia | ESP-Brookesia UI |
| esp-idf/04_QMI8658 | QMI8658 IMU |
| esp-idf/05_LVGL_WITH_RAM | LVGL music demo in internal RAM |

All seven projects are included in ESP-IDF v5.5.5 and v6.0.2 CI.

## Arduino V1

The [arduino](arduino/) set targets SH8601 with FT3168 / FT6146 and contains:

- 01_HelloWorld
- 02_Drawing_board
- 03_GFX_AsciiTable
- 04_GFX_FT3168_Image
- 05_GFX_PCF85063_simpleTime
- 06_GFX_ESPWiFiAnalyzer
- 07_GFX_Clock
- 08_LVGL_Animation
- 09_LVGL_change_background
- 10_LVGL_PCF85063_simpleTime
- 11_LVGL_QMI8658_ui
- 13_LVGL_Widgets
- 15_ES8311
- 16_LVGL_Sqprj

## Arduino V2

The [arduino-v2](arduino-v2/) set targets CO5300 with CST820 and contains:

- 01_HelloWorld
- 02_Drawing_board
- 03_GFX_AsciiTable
- 04_GFX_FT3168_Image
- 05_GFX_PCF85063_simpleTime
- 09_LVGL_change_background
- 11_LVGL_QMI8658_ui
- 13_LVGL_Widgets
- 15_ES8311

The retained 04_GFX_FT3168_Image directory name matches the reference example
layout; its V2 source uses CST820 through the compatible
<code>Arduino_CST816x</code> driver. All V2 touch sketches use periodic
interrupt mode.

Arduino CI builds only the sketches under each first-party examples directory.
Sketches nested under bundled libraries are intentionally excluded.
