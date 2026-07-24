# Continuous integration

The Build Examples workflow is the compilation and firmware packaging gate for
this repository.

## Matrix

| Surface | Version | First-party projects | Build jobs |
| --- | --- | ---: | ---: |
| ESP-IDF | v5.5.5 | 7 | 7 |
| ESP-IDF | v6.0.2 | 7 | 7 |
| Arduino V1 | 3.3.11 | 14 | 14 |
| Arduino V2 | 3.3.11 | 9 | 9 |

A full all or tag run therefore schedules 37 firmware builds.

ESP-IDF uses target esp32c6. Arduino uses:

~~~text
esp32:esp32:esp32c6:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB
~~~

## Discovery

scripts/discover_examples.py accepts all, an example name, or a repo-relative
path. Pull requests and pushes select affected examples. Workflow, discovery,
configuration, and release-packaging changes rebuild the relevant complete
surface. Bundled Arduino library changes rebuild the sketches that consume
that library set.

Library-owned example sketches, checked-in factory binaries, and nested
component test applications are not part of the product matrix.

## Firmware artifacts

Each successful job runs releases/package_firmware.py and uploads one archive.
ESP-IDF packages are generated from flasher_args.json. Arduino packages prefer
the merged binary and otherwise include the bootloader, partition table,
boot_app0, and application images at their inferred offsets.

Every archive contains:

- manifest.json
- flash_args.txt
- flash.sh and flash.bat
- firmware binaries under bin/

Compilation success does not prove V1/V2 display output, touch coordinates,
audio routing, sensor readings, SD access, or battery behavior. Validate those
items on hardware by flashing CI artifacts.
