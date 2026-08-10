# Getting started

[简体中文](GETTING_STARTED_ZH.md)

## Identify the board revision

V1 uses SH8601 with FT3168 / FT6146. V2 uses CO5300 with CST820. The two
revisions share the board-level pin map.

For Arduino, open the matching examples/arduino or examples/arduino-v2
sketch. For ESP-IDF, use the same project for either revision; the BSP probes
touch address 0x15 first for V2 and 0x38 for V1.

## ESP-IDF

Supported CI versions are v5.5.5 and v6.0.2 with target esp32c6.

~~~bash
cd examples/esp-idf/00_board_check
idf.py set-target esp32c6
idf.py build
idf.py -p PORT flash monitor
~~~

Run 00_bsp_quickstart next to check the detected display and touch path. The
project manifest resolves `waveshare/esp32_c6_touch_amoled_1_8` from the
Component Registry with `^1.0.0`. CI covers ESP-IDF v5.5.5 and v6.0.2; the
1.x range is intended for non-breaking BSP updates.

## Arduino

CI uses Arduino-ESP32 3.3.11 and this FQBN:

~~~text
esp32:esp32:esp32c6:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB
~~~

Use the bundled libraries beside the selected V1 or V2 sketch set.

## CI firmware

Every successful source build uploads a flashable package. This is the
repository build validation path. Download a completed run with:

~~~bash
python3 releases/download_artifacts.py --run-id RUN_ID --clean
~~~

Each extracted package includes its exact flash command and platform helpers.
