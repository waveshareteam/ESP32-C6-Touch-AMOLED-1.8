# 仓库结构

[English](PROJECT_STRUCTURE.md)

| 路径 | 用途 |
| --- | --- |
| examples/esp-idf | 第一方 ESP-IDF 工程 |
| examples/arduino | V1 Arduino 草图及捆绑库 |
| examples/arduino-v2 | V2 Arduino 草图及捆绑库 |
| config | 共享 ESP-IDF 配置说明 |
| Firmware | 工厂/恢复镜像；保留大小写的公共兼容路径和不可变交付边界 |
| releases | CI 固件打包与下载工具 |
| Schematic | 开发板原理图；保留大小写的公共兼容路径 |
| scripts | CI 示例发现与审计工具 |
| docs | 仓库文档 |
| assets | 仓库文档使用的产品图片 |

可复用显示、触摸和板级支持代码属于 Waveshare-ESP32-components；产品 ESP-IDF 示例通过受管 registry BSP 依赖使用它们，不保留本地可复用 BSP 副本。
