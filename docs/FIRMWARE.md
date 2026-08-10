# Firmware artifacts

[简体中文](FIRMWARE_ZH.md)

This repository contains two different firmware sources.

## CI source builds

GitHub Actions builds first-party ESP-IDF and Arduino examples and packages
the result as flashable artifacts. Download and extract a run with:

~~~bash
python3 releases/download_artifacts.py --run-id RUN_ID --clean
~~~

Open the matching artifact directory under releases/downloads/run-RUN_ID and
flash with:

~~~bash
./flash.sh /dev/ttyACM0
~~~

On Windows:

~~~bat
flash.bat COMx
~~~

Install esptool when required:

~~~bash
python -m pip install esptool
~~~

Inspect manifest.json before flashing to confirm the framework, example,
target, source revision, offsets, and command.

## Factory and recovery image

[Firmware](../Firmware/) contains a checked-in factory/recovery image. It is
not rebuilt, repackaged, or uploaded by example CI. Use the product Wiki or
the established manufacturing/recovery procedure for that image.

Factory firmware source and build instructions are not included in this
repository yet and may be added in a later update.
