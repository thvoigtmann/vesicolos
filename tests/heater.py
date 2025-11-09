import RPi.GPIO as GPIO
import gpiozero
import time

GPIO_HEATER = 12
GPIO_LED = 13

led = gpiozero.LED(GPIO_LED)
led.off()

heater = gpiozero.PWMOutputDevice(GPIO_HEATER)
heater.off()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
