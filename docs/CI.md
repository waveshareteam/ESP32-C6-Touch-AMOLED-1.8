# Continuous integration

[简体中文](CI_ZH.md)

The Build Examples workflow has an always-visible lightweight routing and
Markdown gate on every pull request. Product compilation and firmware packaging
run only after that gate and only for selected examples.

## Matrix

| Surface | Version | First-party projects | Build jobs |
| --- | --- | ---: | ---: |
| ESP-IDF | v5.5.5 | 7 | 7 |
| ESP-IDF | v6.0.2 | 7 | 7 |
| Arduino V1 | 3.3.11 | 14 | 14 |
| Arduino V2 | 3.3.11 | 9 | 9 |

A full `all`, tag, or workflow-dispatch run schedules 37 firmware builds.

ESP-IDF uses target esp32c6. Arduino uses:

~~~text
esp32:esp32:esp32c6:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB
~~~

## Discovery

`scripts/discover_examples.py` accepts `all`, an example name, or a
repo-relative path. Pull requests select affected examples; documentation-only
changes run the lightweight gate and select zero product builds. Direct example
source selects only that project or sketch, while workflow/configuration/
packaging/discovery inputs select the applicable full surface. Bundled Arduino
library source selects its sketches; bundled-library Markdown does not.

The router includes rename/deletion old paths. Firmware Markdown, source,
binaries, and archives are reported as the independent firmware surface and do
not enter the examples matrix. An unavailable or empty pull-request diff fails
the gate; it never silently falls back to all builds. An unfamiliar complete
non-document path conservatively selects the full matrix.

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
items on hardware by flashing CI artifacts. Local product builds are not a
repository CI contract; the workflow is the build-validation surface.
