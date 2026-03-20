# Introduction

VESICOLOS is a small fluorescence microscope in a 2U CubeSat unit,
designed for the flight on the sounding rocket [MAPHEUS®](https://www.dlr.de/en/research-and-transfer/projects-and-missions/mapheus).
It is a sub-payload of the MOSAIC module, which handles power and signals
for its CubeSat-sized inserts.

The setup was designed for the observation of giant unilamellar vesicles (GUV)
in microgravity, hence the name.

The MAPHEUS sounding rocket is a program run by the German Aerospace Center
(DLR), in collaboration between the [Institute of Frontier Materials
on Earth an in Space](https://www.dlr.de/en/fm), the [Institute of
Aerospace Medicine](https://www.dlr.de/en/me), and the [Mobile Rocket
Base MORABA](https://moraba.de/en/). VESICOLOS is part of a collaboration
between DLR and the University of Bordeaux.

## Contributors

VESICOLOS was created by

- Jörg Drescher - hardware engineering (DLR-FM)
- Jens Hauslage - optics (DLR-ME)
- Thomas Voigtmann - software (DLR-FM)
- Laura Alvarez - science (U Bordeaux)
- Paulina Blair - science (DLR-FM / U Düsseldorf)
- Christian Kahlo - software revision, spare parts ([VX4](https://sites.vx4.de/imprint))

# Hardware Setup

For the MAPHEUS-16 flight, we used a Raspberry 5, and the following
hardware setup:

- Temperature sensor on SPI, 2-wire setup using Adafruit MAX31865 board.
- LED and Heater on PWM channels.
- Servo motors [Waveshare ST3020](https://www.waveshare.com/wiki/ST3020_Servo)
  controlled by a Waveshare driver.
  - Stage movement (X/Y): 2.3mm per turn of 4096 steps
  - Focus movement (Z): 0.1mm per turn of 4096 steps

Raspberry GPIO pin layout (using GPIO numbers, not physical pin numbers):

| GPIO    | Function  | Remark                                  |
|---------|-----------|-----------------------------------------|
| GPIO 5  | SPI CS1   | temperature sensor                      |
| GPIO 6  | SPI CS2   | not yet connected on the clip connector |
| GPIO 9  | SPI MISO  | standard                                |
| GPIO 10 | SPI MOSI  | standard                                |
| GPIO 11 | SPI CLK   | standard                                |
| GPIO 12 | PWM1      | LED                                     |
| GPIO 13 | PWM2      | heater                                  |
| GPIO 17 | LO        | lift off from MOSAIC                    |
| GPIO 23 | mug       | microgravity detection from MOSAIC      |


# Raspberry Configuration

- Make sure it boots into graphical desktop, using LXDE
- The `/boot/firmware/config.txt` should have the following entries:
  ```bash
  usb_max_current_enable=1
  dtparam=uart0=on
  dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
  dtoverlay=w1-gpio
  dtoverlay=disable-wifi
  dtoverlay=disable-bt
  ```
  These should have the effect that UART is enabled, 2 hardware PWM channels
  are enabled, and WiFi is off. Also the Raspberry 5 should be forced to
  ignore the underpower warning if it thinks that our power supply cannot
  handle 5V/5A (it can).
- The EEPROM config should be adapted with the `rpi-eeprom-config` tool.
  The flight configuration (MAPHEUS-16) was
  ```bash
  BOOT_UART=1
  BOOT_ORDER=0xf461
  NET_INSTALL_AT_POWER_ON=0
  PSU_MAX_CURRENT=5000
  ```


# Software Installation

The contents of this repository should be placed into
`~/Desktop/vesicolos` such that the script `~/Desktop/vesicolos/start.sh`
can be found.

Inside this folder, a python virtual environment needs to be created
as follows:
```bash
python3 -m venv --system-site-packages venv
. venv/bin/activate
```
Then install the required python libraries found in `requirements.txt`.
For the `lgpio` library on the Raspberry 5, we found it necessary to first
install `lg` from https://abyz.me.uk/lg/download.html (follow instructions
there). The important packages that do not come with the raspberry install
should be
```bash
adafruit-circuitpython-max31865 gpiozero lgpio rpi-lgpio pyserial
```
(TODO: check whether `lgpio` is still used)

The file in `autostart` needs to be copied into `~/.config/autostart/`.
This should make the vesicolos python program automatically start once
the LXDE desktop is up.

# Code Options

The `vesciolos.py` uses a number of hard-coded defaults that can be
found in the top part of the source code - those are mostly pin numbers
and things related to the hardware.

The code respects the following environment variables:

| env variable   | function                 | Remark                   |
|----------------|--------------------------|--------------------------|
| `RASPI_MODEL`  | model version (4 or 5)   | switches UART device     |
| `ST_DEVICE`    | UART device name         | overrides auto-detect    |
| `DEBUG`        | debugging on             | increases log verbosity  |
| `NO_GPIO`      | don't import GPIO lib    | for testing on non-RPi   |

The code also looks for a JSON configuration file in `flight_config`
in order to set per-flight defaults. The first command-line argument, if
given, can explicitly specify the name of a JSON file, e.g.
`vesicolos.py M16_HCD`.
If no file name is given, the file in `flight_config` with the latest
modification-time stamp will be used. Currently, we set

| config variable | function                                   |
|-----------------|--------------------------------------------|
| `SOE_TIMEOUT`   | SOE timeout in seconds, if no signal comes |
| `EXP_TIMEOUT`   | EXP timeout in seconds, if no signal comes |


