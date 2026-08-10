# 使用内部 RAM 的 LVGL

[English](README.md)

此示例通过受管 `waveshare/esp32_c6_touch_amoled_1_8` BSP 启动 LVGL 音乐演示。

开发板不带 PSRAM，显示缓冲区从内部 RAM 分配；BSP 会自动为 V1 选择 SH8601/FT5x06，或为 V2 选择 CO5300/CST820。仓库 CI 会验证编译与固件打包。
