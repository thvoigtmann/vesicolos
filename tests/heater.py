import RPi.GPIO as GPIO
import gpiozero
import time

#from gpiozero.pins.lgpio import LGPIOFactory
#gpiozero.Device.pin_factory = LGPIOFactory(chip=0)

GPIO_HEATER = 12
GPIO_LED = 13

led = gpiozero.LED(GPIO_LED)
#led.off()
led.on()

heater = gpiozero.PWMOutputDevice(GPIO_HEATER)
heater.off()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
