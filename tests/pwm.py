import RPi.GPIO as GPIO
import gpiozero
import time

led = gpiozero.LED(13)

led.on()
led.value = 1.
time.sleep(15)
led.off()
led.value = 0
