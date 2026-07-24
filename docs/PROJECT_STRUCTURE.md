# Repository structure

| Path | Purpose |
| --- | --- |
| examples/esp-idf | First-party ESP-IDF projects |
| examples/arduino | V1 Arduino sketches and bundled libraries |
| examples/arduino-v2 | V2 Arduino sketches and bundled libraries |
| config | Shared ESP-IDF configuration notes |
| Firmware | Checked-in factory/recovery image |
| releases | CI firmware packaging and download tools |
| Schematic | Board schematic revisions |
| scripts | CI example discovery |
| docs | Repository documentation |
| assets | Product images used by repository documentation |

Reusable display, touch, and board support code belongs in
Waveshare-ESP32-components. Product ESP-IDF examples consume the BSP as a
managed Git or registry dependency and do not keep local reusable BSP copies.

Generated sdkconfig files, component locks, managed_components directories,
build trees, packaged archives, and downloaded CI artifacts are ignored.
