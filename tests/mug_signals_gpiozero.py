import gpiozero
import time

status = { 'LO': None, 'SOE': None, 'SODS': None, 'mug': None }
status_pins = { 'LO': 17, 'mug': 27 }

from gpiozero.pins.lgpio import LGPIOFactory
gpiozero.Device.pin_factory = LGPIOFactory(chip=15)
gpio_input = {}
for pin in status_pins.values():
    gpio_input[pin] = gpiozero.InputDevice(pin)

while True:
    for signal,pin in status_pins.items():
        if gpio_input[pin].is_active:
            status[signal] = True
        else:
            status[signal] = False
    print(status)
    time.sleep(0.2)
