# ESP32-C6-Touch-AMOLED-1.8

[简体中文](README_CN.md)

ESP32-C6 development board with a 1.8-inch 368 x 448 QSPI AMOLED display,
capacitive touch, 16 MB Flash, power management, RTC, IMU, audio, and microSD.
The board has no PSRAM.

## Hardware revisions

| Revision | Display | Touch | Arduino examples |
| --- | --- | --- | --- |
| V1 | SH8601 | FT3168 / FT6146 | [examples/arduino](examples/arduino/) |
| V2 | CO5300 | CST820 | [examples/arduino-v2](examples/arduino-v2/) |

Both revisions use the same board-level pins. Arduino keeps separate example
sets because each sketch instantiates its display and touch drivers directly.
ESP-IDF examples use the managed waveshare/esp32_c6_touch_amoled_1_8 BSP,
which detects the touch controller and selects the matching display and touch
drivers automatically.

## Hardware overview

| Feature | Device / interface |
| --- | --- |
| MCU | ESP32-C6, target esp32c6 |
| Memory | 16 MB Flash, no PSRAM |
| Display | 1.8-inch 368 x 448 QSPI AMOLED |
| V1 display / touch | SH8601 with FT3168 / FT6146 |
| V2 display / touch | CO5300 with CST820 |
| Power management | AXP2101 |
| Real-time clock | PCF85063A |
| Motion sensor | QMI8658 six-axis IMU |
| Audio | ES8311 codec, microphone input, speaker output |
| Storage | microSD over SPI |
| Schematics | [Schematic](Schematic/) |

## Examples

ESP-IDF projects are under [examples/esp-idf](examples/esp-idf/):

| Example | Purpose |
| --- | --- |
| [00_board_check](examples/esp-idf/00_board_check/) | Serial board, memory, BSP capability, and revision check |
| [00_bsp_quickstart](examples/esp-idf/00_bsp_quickstart/) | V1/V2 display and touch quick start |
| [01_AXP2101](examples/esp-idf/01_AXP2101/) | AXP2101 power management diagnostics |
| [02_PCF85063](examples/esp-idf/02_PCF85063/) | PCF85063A RTC read/write example |
| [03_esp-brookesia](examples/esp-idf/03_esp-brookesia/) | ESP-Brookesia phone UI |
| [04_QMI8658](examples/esp-idf/04_QMI8658/) | QMI8658 acceleration and gyro readings |
| [05_LVGL_WITH_RAM](examples/esp-idf/05_LVGL_WITH_RAM/) | LVGL music demo using internal RAM |

Arduino provides 14 V1 sketches and 9 V2 sketches. Bundled libraries remain
next to each set and their upstream library examples are excluded from product
CI. See [examples/README.md](examples/README.md) for the complete index.

## Supported toolchains

| Surface | Version | Target / board options |
| --- | --- | --- |
| ESP-IDF | v5.5.5 | esp32c6 |
| ESP-IDF | v6.0.2 | esp32c6 |
| Arduino-ESP32 | 3.3.11 | esp32:esp32:esp32c6, 16 MB Flash, app3M_fat9M_16MB |

GitHub Actions discovers and builds every first-party project, then uploads a
flashable firmware package for each successful build. See
[docs/CI.md](docs/CI.md).

## BSP dependency

Before the component registry release, ESP-IDF projects use this Git
dependency pinned to the BSP commit:

~~~yaml
waveshare/esp32_c6_touch_amoled_1_8:
  git: https://github.com/waveshareteam/Waveshare-ESP32-components.git
  path: bsp/esp32_c6_touch_amoled_1_8
  version: "d75c3e72be9e2248f525bcdbf9ca31f1fe8d357b"
~~~

The BSP source manifest declares version 1.0.0. The full commit SHA keeps
CI dependency resolution reproducible before the registry release. After the
component is published, projects can move to a compatible registry constraint
such as ^1.0.0.

## Firmware

CI firmware packages contain a manifest, binaries, flash arguments, and flash
helpers. Download them with:

~~~bash
python3 releases/download_artifacts.py --run-id RUN_ID --clean
~~~

The checked-in image under [Firmware](Firmware/) is a factory/recovery asset,
not a CI build output. See [docs/FIRMWARE.md](docs/FIRMWARE.md).

## Documentation

- [Getting started](docs/GETTING_STARTED.md)
- [Example index](examples/README.md)
- [Continuous integration](docs/CI.md)
- [Firmware artifacts](docs/FIRMWARE.md)
- [Repository structure](docs/PROJECT_STRUCTURE.md)
- [Release tools](releases/README.md)
- [Product Wiki](https://www.waveshare.com/wiki/ESP32-C6-Touch-AMOLED-1.8)

## Support and license

Use [GitHub Issues](https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-1.8/issues)
for reproducible reports, and include the board revision, example path,
framework version, and relevant logs. See [SUPPORT.md](SUPPORT.md),
[CONTRIBUTING.md](CONTRIBUTING.md), and [SECURITY.md](SECURITY.md).

Unless noted otherwise in a subdirectory, this repository is licensed under
the Apache License 2.0. Third-party code keeps its own license and notices.
