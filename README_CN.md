# ESP32-C6-Touch-AMOLED-1.8

[English](README.md)

本仓库提供 ESP32-C6-Touch-AMOLED-1.8 的 ESP-IDF、Arduino 示例、CI 固件产物工具、
工厂恢复镜像和开发文档。开发板搭载 1.8 英寸 368 x 448 QSPI AMOLED 触摸屏、
16 MB Flash，不带 PSRAM。

## 硬件版本

| 版本 | 显示控制器 | 触摸控制器 | Arduino 示例 |
| --- | --- | --- | --- |
| V1 | SH8601 | FT3168 / FT6146 | [examples/arduino](examples/arduino/) |
| V2 | CO5300 | CST820 | [examples/arduino-v2](examples/arduino-v2/) |

V1 与 V2 共用板级引脚。Arduino 示例会直接创建显示和触摸驱动，因此按版本保留
两个独立目录。ESP-IDF 示例统一使用 waveshare/esp32_c6_touch_amoled_1_8 BSP；
BSP 会探测触摸地址并自动选择对应的显示和触摸驱动，不需要修改示例源码。

## 硬件概览

| 功能 | 器件 / 接口 |
| --- | --- |
| MCU | ESP32-C6，目标 esp32c6 |
| 存储 | 16 MB Flash，无 PSRAM |
| 显示 | 1.8 英寸 368 x 448 QSPI AMOLED |
| 电源管理 | AXP2101 |
| 实时时钟 | PCF85063A |
| 运动传感器 | QMI8658 六轴 IMU |
| 音频 | ES8311 编解码器、麦克风输入和扬声器输出 |
| 扩展存储 | SPI microSD |
| 原理图 | [Schematic](Schematic/) |

## ESP-IDF 示例

| 示例 | 用途 |
| --- | --- |
| [00_board_check](examples/esp-idf/00_board_check/) | 串口输出板卡、内存、BSP 能力和硬件版本 |
| [00_bsp_quickstart](examples/esp-idf/00_bsp_quickstart/) | V1/V2 显示与触摸快速验证 |
| [01_AXP2101](examples/esp-idf/01_AXP2101/) | AXP2101 电源管理诊断 |
| [02_PCF85063](examples/esp-idf/02_PCF85063/) | PCF85063A RTC 读写 |
| [03_esp-brookesia](examples/esp-idf/03_esp-brookesia/) | ESP-Brookesia Phone UI |
| [04_QMI8658](examples/esp-idf/04_QMI8658/) | QMI8658 加速度与陀螺仪数据 |
| [05_LVGL_WITH_RAM](examples/esp-idf/05_LVGL_WITH_RAM/) | 使用内部 RAM 的 LVGL 音乐演示 |

Arduino V1 有 14 个一方示例，V2 有 9 个一方示例。完整清单见
[examples/README.md](examples/README.md)。各目录自带的 library examples 不进入产品 CI。

## CI 工具链

| 开发框架 | 版本 | 目标 / 板卡选项 |
| --- | --- | --- |
| ESP-IDF | v5.5.5 | esp32c6 |
| ESP-IDF | v6.0.2 | esp32c6 |
| Arduino-ESP32 | 3.3.11 | esp32:esp32:esp32c6，16 MB Flash，app3M_fat9M_16MB |

Build Examples 工作流会发现并构建全部一方示例，并为每个成功任务上传可刷写固件包。
完整 all 或 tag 运行包含 37 个固件构建任务。说明见 [docs/CI.md](docs/CI.md)。

## BSP 依赖

BSP 正式发布到组件注册表之前，ESP-IDF 工程使用固定到 BSP 提交的 Git 路径依赖：

~~~yaml
waveshare/esp32_c6_touch_amoled_1_8:
  git: https://github.com/waveshareteam/Waveshare-ESP32-components.git
  path: bsp/esp32_c6_touch_amoled_1_8
  version: "d75c3e72be9e2248f525bcdbf9ca31f1fe8d357b"
~~~

BSP 源码 manifest 的版本为 1.0.0。完整 commit SHA 可确保注册表发布前的 CI 依赖解析可复现；注册表版本发布后，可切换为兼容版本范围，例如 ^1.0.0。

## CI 固件与工厂固件

下载指定 GitHub Actions 运行的固件产物：

~~~bash
python3 releases/download_artifacts.py --run-id RUN_ID --clean
~~~

CI 固件包包含 manifest、烧录参数、二进制文件以及 Windows/Linux 烧录脚本。
[Firmware](Firmware/) 中的文件是工厂/恢复镜像，不是 CI 构建产物，二者不要混用。
详见 [docs/FIRMWARE.md](docs/FIRMWARE.md)。

## 文档与支持

- [入门指南](docs/GETTING_STARTED.md)
- [示例清单](examples/README.md)
- [CI 说明](docs/CI.md)
- [固件说明](docs/FIRMWARE.md)
- [仓库结构](docs/PROJECT_STRUCTURE.md)
- [Release 工具](releases/README.md)
- [产品 Wiki](https://www.waveshare.com/wiki/ESP32-C6-Touch-AMOLED-1.8)
- [支持说明](SUPPORT.md)
- [贡献指南](CONTRIBUTING.md)

除子目录另有说明外，本仓库使用 Apache License 2.0。第三方代码保留其原有许可证和声明。
