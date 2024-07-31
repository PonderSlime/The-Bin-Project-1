import time
import machine, neopixel

np = neopixel.NeoPixel(machine.Pin(28), 1)

def neo(r,g,b):
    np[0] = (r, g, b) # set to red, full brightness
    np.write()

