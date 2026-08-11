# AXP2101-PMIC

[简体中文](README_ZH.md)
AXP2101 Power management IC（AXP2101电源管理芯片）

### 硬件开源链接：https://oshwhub.com/mondraker/axp2101_2023-11-18_20-15-19
### 库来自: https://github.com/lewisxhe/XPowersLib
### 我只是把IDF工程独立出来并且整理编译了一遍，这里只是留个记录方便大家直接测试。
***
# XPowersLib Example
### Prerequisites

XPowersLib is bundled in this project under `components/XPowersLib`.


### Configure the Project

Open the project configuration menu (`idf.py menuconfig`).

In the `XPowers Configuration` menu:

* Select the PMU Type in the `PMU_Type` option.
* `PMU SCL GPIO Num` defaults to GPIO 7.
* `PMU SDA GPIO Num` defaults to GPIO 8.
* `PMU Interrupt Pin` defaults to `-1` because the board does not route the PMU interrupt.

## How to Use Example

Set the target to `esp32c6` before configuring the project.


### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

The output information is to configure the output voltage and enable status of the PMU

```
I (345) mian: I2C initialized successfully
I (355) AXP2101: Init PMU SUCCESS!
I (385) AXP2101: DCDC=======================================================================
I (385) AXP2101: DC1  :ENABLE    Voltage:3300 mV
I (385) AXP2101: DC2  :DISABLE   Voltage:900 mV
I (395) AXP2101: DC3  :ENABLE    Voltage:3300 mV
I (395) AXP2101: DC4  :DISABLE   Voltage:1100 mV
I (405) AXP2101: DC5  :DISABLE   Voltage:1200 mV
I (405) AXP2101: ALDO=======================================================================
I (415) AXP2101: ALDO1:ENABLE    Voltage:1800 mV
I (425) AXP2101: ALDO2:ENABLE    Voltage:2800 mV
I (425) AXP2101: ALDO3:ENABLE    Voltage:3300 mV
I (435) AXP2101: ALDO4:ENABLE    Voltage:3000 mV
I (435) AXP2101: BLDO=======================================================================
I (445) AXP2101: BLDO1:ENABLE    Voltage:3300 mV
```

## Upstream XPowersLib setup reference

This standalone upstream setup is not required for this repository, which uses the bundled component.

```
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
git clone https://github.com/lewisxhe/XPowersLib.git
cd esp-idf
./install.sh
. ./export.sh
cd ..
cd XPowersLib/examples/ESP_IDF_Example
idf.py menuconfig
idf.py build
idf.py -b 921600 flash
idf.py monitor

```
