<div align="center">
  <h1>ESP32-C6-Touch-AMOLED-1.8</h1>
  <p><strong>ESP32-C6 1.8-inch 368 x 448 QSPI AMOLED touch development board</strong></p>
  <p>
    <a href="https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-1.8/actions/workflows/examples.yml"><img alt="Build Examples" src="https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-1.8/actions/workflows/examples.yml/badge.svg"></a>
    <a href="LICENSE"><img alt="License" src="https://img.shields.io/github/license/waveshareteam/ESP32-C6-Touch-AMOLED-1.8"></a>
  </p>
  <p>
    <a href="README_CN.md">简体中文</a> ·
    <a href="https://www.waveshare.com/esp32-c6-touch-amoled-1.8.htm">Product Page</a> ·
    <a href="https://docs.waveshare.com/ESP32-C6-Touch-AMOLED-1.8">Product Documentation</a> ·
    <a href="https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-1.8/releases">GitHub Releases</a> ·
    <a href="https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-1.8/actions/workflows/examples.yml">CI Firmware Artifacts</a> ·
    <a href="examples/esp-idf/">ESP-IDF Examples</a> ·
    <a href="examples/arduino/">Arduino V1</a> ·
    <a href="examples/arduino-v2/">Arduino V2</a> ·
    <a href="docs/">Repository Documentation</a>
  </p>
  <a href="https://www.waveshare.com/esp32-c6-touch-amoled-1.8.htm">
    <img src="assets/ESP32-C6-Touch-AMOLED-1.8.jpg" alt="Waveshare ESP32-C6-Touch-AMOLED-1.8 V2 product image" width="520">
  </a>
</div>

---

## Overview

This repository provides first-party ESP-IDF projects, separate Arduino V1
and V2 example sets, source-built firmware packages, factory recovery images,
and development documentation for the Waveshare ESP32-C6-Touch-AMOLED-1.8.

The board combines an ESP32-C6 with a 1.8-inch QSPI AMOLED display,
capacitive touch, 16 MB Flash, power management, RTC, IMU, audio, and microSD.
The board has no PSRAM.

## Hardware Revisions

| Revision | Display | Touch | Arduino examples |
| --- | --- | --- | --- |
| V1 | SH8601 | FT3168 / FT6146 | [examples/arduino](examples/arduino/) |
| V2 | CO5300 | CST820 | [examples/arduino-v2](examples/arduino-v2/) |

Both revisions use the same board-level pins. Arduino keeps separate example
sets because each sketch instantiates its display and touch drivers directly.
ESP-IDF examples use the managed waveshare/esp32_c6_touch_amoled_1_8 BSP,
which detects the touch controller and selects the matching display and touch
drivers automatically.

> [!IMPORTANT]
> The physical touch controller on V2 boards is CST820. The software names
> <code>Arduino_CST816x</code> and <code>esp_lcd_touch_cst816s</code> are
> retained because those drivers are compatible with CST820; they are not the
> onboard controller model.

## Hardware Overview

| Feature | Device / interface |
| --- | --- |
| MCU | ESP32-C6, single-core 32-bit RISC-V up to 160 MHz (target esp32c6) |
| Wireless | 2.4 GHz Wi-Fi 6, Bluetooth 5 LE, and IEEE 802.15.4 (Zigbee 3.0 / Thread) |
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

## Quick Start

1. Identify the board revision in the table above. If it is unknown, run
   [00_board_check](examples/esp-idf/00_board_check/) to let the BSP report it.
2. For ESP-IDF, start with
   [00_bsp_quickstart](examples/esp-idf/00_bsp_quickstart/); the same BSP-based
   projects support V1 and V2.
3. For Arduino, use [examples/arduino](examples/arduino/) for V1 or
   [examples/arduino-v2](examples/arduino-v2/) for V2.
4. To flash without building locally, download the matching package from a
   successful [Build Examples run](https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-1.8/actions/workflows/examples.yml).

See [Getting Started](docs/GETTING_STARTED.md) for toolchain and flashing details.

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

## Supported Toolchains

| Surface | Version | Firmware builds |
| --- | --- | ---: |
| ESP-IDF | <code>v5.5.5</code> | 7 |
| ESP-IDF | <code>v6.0.2</code> | 7 |
| Arduino-ESP32 V1 | <code>3.3.11</code> | 14 |
| Arduino-ESP32 V2 | <code>3.3.11</code> | 9 |

The [Build Examples workflow](https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-1.8/actions/workflows/examples.yml)
runs two discovery jobs and 37 firmware builds for a full <code>all</code> or
tag matrix. ESP-IDF targets <code>esp32c6</code>; Arduino uses
<code>esp32:esp32:esp32c6:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB</code>.
Every successful build uploads a flashable firmware package. See
[Continuous Integration](docs/CI.md) for discovery and artifact details.

## BSP Dependency

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
such as <code>^1.0.0</code>.

## Firmware Artifacts

Each successful CI build is packaged as a flashable archive and uploaded to
the [Build Examples workflow](https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-1.8/actions/workflows/examples.yml).
Download and extract all artifacts from a completed run with:

~~~bash
python3 releases/download_artifacts.py --run-id RUN_ID --clean
~~~

Open the matching directory under <code>releases/downloads/run-RUN_ID/</code>,
install esptool with <code>python -m pip install esptool</code>, and flash with:

~~~bash
./flash.sh /dev/ttyACM0
~~~

On Windows:

~~~bat
flash.bat COMx
~~~

Each package contains <code>manifest.json</code>, <code>flash_args.txt</code>,
platform-specific flash helpers, and the required binaries under
<code>bin/</code>. The checked-in image under [Firmware](Firmware/) is a
separate factory/recovery asset, not a CI build output. See
[Firmware Artifacts](docs/FIRMWARE.md).

## Repository Layout

| Path | Purpose |
| --- | --- |
| [examples/esp-idf/](examples/esp-idf/) | First-party ESP-IDF projects |
| [examples/arduino/](examples/arduino/) | V1 Arduino sketches and bundled libraries |
| [examples/arduino-v2/](examples/arduino-v2/) | V2 Arduino sketches and bundled libraries |
| [Firmware/](Firmware/) | Factory flashing and recovery image |
| [releases/](releases/) | Firmware packaging and artifact download tools |
| [Schematic/](Schematic/) | Board schematics |
| [config/](config/) | Shared ESP-IDF configuration notes and overlays |
| [docs/](docs/) | Setup, examples, CI, structure, and firmware documentation |
| [assets/](assets/) | Product images used by repository documentation |

## Documentation

- [Getting started](docs/GETTING_STARTED.md)
- [Example index](examples/README.md)
- [Continuous integration](docs/CI.md)
- [Firmware artifacts](docs/FIRMWARE.md)
- [Repository structure](docs/PROJECT_STRUCTURE.md)
- [Release tools](releases/README.md)
- [Product page](https://www.waveshare.com/esp32-c6-touch-amoled-1.8.htm)
- [Product documentation](https://docs.waveshare.com/ESP32-C6-Touch-AMOLED-1.8)

## Support and Contributions

Use [GitHub Issues](https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-1.8/issues)
for reproducible reports, and include the board revision, example path,
framework version, reproduction steps, expected behavior, actual behavior,
and relevant serial or build logs.

- [Contributing Guide](CONTRIBUTING.md)
- [Support](SUPPORT.md)
- [Security Policy](SECURITY.md)
- [Code of Conduct](CODE_OF_CONDUCT.md)
- [Third-party Notices](THIRD_PARTY.md)
- [Open an Issue](https://github.com/waveshareteam/ESP32-C6-Touch-AMOLED-1.8/issues/new/choose)

## License

Unless noted otherwise in a subdirectory, this repository is licensed under
the Apache License 2.0. Third-party code keeps its own license and notices.
See [LICENSE](LICENSE) and [Third-party Notices](THIRD_PARTY.md).
