# Repository structure

[简体中文](PROJECT_STRUCTURE_ZH.md)

| Path | Purpose |
| --- | --- |
| examples/esp-idf | First-party ESP-IDF projects |
| examples/arduino | V1 Arduino sketches and bundled libraries |
| examples/arduino-v2 | V2 Arduino sketches and bundled libraries |
| config | Shared ESP-IDF configuration notes |
| Firmware | Checked-in factory/recovery image; public case-sensitive compatibility path and immutable delivery boundary |
| releases | CI firmware packaging and download tools |
| Schematic | Board schematic revisions; public case-sensitive compatibility path |
| scripts | CI example discovery |
| docs | Repository documentation |
| assets | Product images used by repository documentation |

Reusable display, touch, and board support code belongs in
Waveshare-ESP32-components. Product ESP-IDF examples consume the BSP as a
managed registry dependency and do not keep local reusable BSP copies.

Generated sdkconfig files, component locks, managed_components directories,
build trees, packaged archives, and downloaded CI artifacts are ignored.
