# Hardware Setup

For the MAPHEUS-16 flight, we used a Raspberry 5, and the following
hardware setup:

- Temperature sensor on SPI, 2-wire setup using Adafruit MAX31865 board.
- LED and Heater on PWM channels.
- Servo motors controlled by a Waveshare driver.

Raspberry GPIO pin layout (using GPIO numbers, not physical pin numbers):

| GPIO    | Function | Remark |
|---------|----------|--------|
| GPIO 5  | SPI CS1   | temperature sensor |
| GPIO 6  | SPI CS2   | not yet connected on the clip connector |
| GPIO 9  | SPI MISO  | standard 
| GPIO 10 | SPI MOSI  | standard
| GPIO 11 | SPI CLK   | standard
| GPIO 12 | PWM1      | LED
| GPIO 13 | PWM2      | heater
| GPIO 17 | LO        | lift off from MOSAIC |
| GPIO 23 | mug       | microgravity detection from MOSAIC


# Raspberry Configuration

- Make sure it boots into graphical desktop, using LXDE
- The `/boot/firmware/config.txt` should have the following entries:
  ```bash
  usb_max_current_enable=1
  dtparam=uart0=on
  dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
  dtoverlay=w1-gpio
  dtoverlay=disable-wifi
  ```
  These should have the effect that UART is enabled, 2 hardware PWM channels
  are enabled, and WiFi is off. Also the Raspberry 5 should be forced to
  ignored the underpower warning if it things that our power supply cannot
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
python3 -m venv venv
. venv/bin/activate
```
Then install the required python libraries found in `requirements.txt`.
For the `lgpio` library on the Raspberry 5, we found it necessary to first
install `lg` from https://abyz.me.uk/lg/download.html (follow instructions
there).

The file in `autostart` needs to be copied into `~/.config/autostart/`.
This should make the vesicolos python program automatically start once
the LXDE desktop is up.
