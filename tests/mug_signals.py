import RPi.GPIO as gpio
import time

status = { 'LO': False, 'SOE': False, 'SODS': False, 'mug': False, '27': False }
status_pins = { 'LO': 17, 'mug': 23, '27': 27 }

gpio.setmode(gpio.BCM)
for pin in status_pins.values():
    gpio.setup(pin, gpio.IN) #, pull_up_down=gpio.PUD_DOWN)

while True:
    for signal,pin in status_pins.items():
        if gpio.input(pin):
            status[signal] = True
        else:
            status[signal] = False
    print(status)
    time.sleep(1)
