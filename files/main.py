from machine import Pin, SPI
import max7219
from screen import main
from keypad import loop_keypad
from joypad import joy
from speaker_machine import d_note, c_note, d2_note
from pico_dht_22 import PicoDHT22
from rotary_irq_rp2 import RotaryIRQ
import utime
import time
from utime import sleep
import machine, neopixel

CLOCK_PIN = 10
DATA_PIN = 11
CS_PIN = 13
d_note()
#spi0=SPI(0,baudrate=10000000, polarity=1, phase=0, sck=Pin(CLOCK_PIN), mosi=Pin(DATA_PIN))
c_note()
cs = Pin(CS_PIN, Pin.OUT)
d2_note()

#matrix = max7219.Matrix8x8(spi0, cs , 1)

#matrix.text('A', 0, 0, 1)
#matrix.show()
#sleep(delay_time)


np = neopixel.NeoPixel(machine.Pin(0), 1)
np[0] = (255, 0, 0) # set to red, full brightness
np.write()
dht_sensor=PicoDHT22(Pin(15,Pin.IN,Pin.PULL_UP),dht11=True)

SW = Pin(22,Pin.IN,Pin.PULL_UP)  

r = RotaryIRQ(pin_num_clk=20,
              pin_num_dt=17,
              min_val=0,
              reverse=False,
              range_mode=RotaryIRQ.RANGE_UNBOUNDED)
val_old = r.value()
c_note()
main()
while True:
  loop_keypad()
  joy()
  T,H = dht_sensor.read()
  if T is None:
      print(" sensor error")
      d_note()
  else:
      print("{}'C  {}%".format(T,H))
      #d2_note()
  #DHT22 not responsive if delay to short
  try:  
    val_new = r.value()  
    if SW.value()==0 and n==0:  
        print("Button Pressed")  
        print("Selected Number is : ",val_new)
        c_note()
        np[0] = (255, 255, 0) # set to red, full brightness
        np.write()
        n=1 
        while SW.value()==0:  
            continue  
    n=0  
    if val_old != val_new:  
        val_old = val_new  
        print('result =', val_new)
        c_note()
        np[0] = (0, 255, 255) # set to red, full brightness
        np.write()
    time.sleep_ms(50)
  except KeyboardInterrupt:  
    break

CLOCK_PIN = 10
DATA_PIN = 11
CS_PIN = 13
d_note()
#spi0=SPI(0,baudrate=10000000, polarity=1, phase=0, sck=Pin(CLOCK_PIN), mosi=Pin(DATA_PIN))
c_note()
cs = Pin(CS_PIN, Pin.OUT)
d2_note()

#matrix = max7219.Matrix8x8(spi0, cs , 1)

#matrix.text('A', 0, 0, 1)
#matrix.show()
#sleep(delay_time)