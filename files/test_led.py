#Most of this hour was spent on flashing the pico, since i encountered some issues with it.
from machine import Pin
import time
led = Pin("LED", Pin.OUT)
while True:
  led.toggle()
  time.sleep(1)
