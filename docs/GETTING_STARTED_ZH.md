# 入门指南

[English](GETTING_STARTED.md)

## 确认开发板版本

V1 使用 SH8601 与 FT3168 / FT6146，V2 使用 CO5300 与 CST820；两者使用相同板级引脚。

Arduino 请打开对应的 `examples/arduino` 或 `examples/arduino-v2` 草图。ESP-IDF 两个版本使用同一工程，BSP 会探测触摸地址。

## ESP-IDF

CI 支持 ESP-IDF v5.5.5 与 v6.0.2，目标为 `esp32c6`。先运行 `00_board_check`，再运行 `00_bsp_quickstart`。工程从 Component Registry 解析 `waveshare/esp32_c6_touch_amoled_1_8` 的 `^1.0.0`；1.x 用于非破坏性 BSP 更新。

## Arduino

CI 使用 Arduino-ESP32 3.3.11，并使用示例旁的捆绑库。
