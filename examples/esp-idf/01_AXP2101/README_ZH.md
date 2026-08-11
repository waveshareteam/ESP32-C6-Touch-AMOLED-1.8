# AXP2101 电源管理

[English](README.md)

此诊断工程包含用于低层 PMU 初始化的本地上游 XPowersLib 端口；该端口保留上游 MIT 许可证声明。

## 使用方法

配置并编译前将目标设置为 `esp32c6`。在 `XPowers Configuration` 中选择 PMU 类型；SCL 默认为 GPIO 7，SDA 默认为 GPIO 8，开发板未连接 PMU 中断。

运行 `idf.py -p PORT flash monitor` 进行编译、烧录和监视。完整 ESP-IDF 设置步骤见 Espressif 入门文档。
