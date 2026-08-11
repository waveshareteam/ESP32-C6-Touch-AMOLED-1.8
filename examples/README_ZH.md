# 示例索引

[English](README.md)

## ESP-IDF

先使用 `00_board_check` 查看串口输出，再使用 `00_bsp_quickstart` 验证显示和触摸。BSP 会自动检测 V1 或 V2。

| 路径 | 用途 |
| --- | --- |
| esp-idf/00_board_check | 开发板、内存、能力和硬件版本 |
| esp-idf/00_bsp_quickstart | 显示、触摸、亮度与 LVGL |
| esp-idf/01_AXP2101 | AXP2101 PMU |
| esp-idf/02_PCF85063 | PCF85063A RTC |
| esp-idf/03_esp-brookesia | ESP-Brookesia UI |
| esp-idf/04_QMI8658 | QMI8658 IMU |
| esp-idf/05_LVGL_WITH_RAM | 内部 RAM 中的 LVGL 音乐演示 |

全部 7 个工程均在 ESP-IDF v5.5.5 与 v6.0.2 CI 中验证。

## Arduino

Arduino V1 位于 [arduino](arduino/)，V2 位于 [arduino-v2](arduino-v2/)。CI 仅构建两个第一方 `examples` 目录中的草图，不构建捆绑库中嵌套的示例。
