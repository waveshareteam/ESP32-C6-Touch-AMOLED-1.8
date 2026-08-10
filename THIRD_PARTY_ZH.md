# 第三方软件

[English](THIRD_PARTY.md)

本仓库包括源码示例、固件资源、受管组件引用和捆绑第三方库。

## 受管 ESP-IDF 组件

多数 ESP-IDF 示例使用 Component Manager 解析的 `waveshare/esp32_c6_touch_amoled_1_8`、`waveshare/pcf85063a` 和 `waveshare/qmi8658`。精确版本在各示例的 `main/idf_component.yml` 中声明。

## 捆绑库

Arduino 示例树含 LVGL、Adafruit BusIO、SensorLib、GFX Library for Arduino 等捆绑库，其上游许可证保留在各自目录中。`examples/esp-idf/01_AXP2101` 诊断工程含本地 XPowersLib 端口以支持低层 PMU 启动，文件保留上游 MIT 声明。Brookesia app 是产品示例功能，不是通用 BSP。

## 固件二进制

`Firmware/` 中的文件是本板预编译工厂固件，请参阅 [固件说明](Firmware/README_ZH.md)。
