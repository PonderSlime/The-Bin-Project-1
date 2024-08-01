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


#Intialize the SPI
spi = SPI(0, baudrate=10000000, polarity=1, phase=0, sck=Pin(2), mosi=Pin(3))
ss = Pin(5, Pin.OUT)

# Create matrix display instant, which has four MAX7219 devices.
display = max7219.Matrix8x8(spi, ss, 1)

#Set the display brightness. Value is 1 to 15.
display.brightness(10)

#Define the scrolling message
scrolling_message = "RASPBERRY PI PICO AND MAX7219 -- 8x8 DOT MATRIX SCROLLING DISPLAY"

#Get the message length
length = len(scrolling_message)

#Calculate number of columns of the message
column = (length * 8)

#Clear the display.
display.fill(0)
display.show()

#sleep for one one seconds
time.sleep(1)

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
  
  for x in range(32, -column, -1):     
    #Clear the display
    display.fill(0)

    # Write the scrolling text in to frame buffer
    display.text(scrolling_message ,x,0,1)
        
    #Show the display
    display.show()
      
    #Set the Scrolling speed. Here it is 50mS.
    time.sleep(0.05) 
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


''' # Import MicroPython libraries of PIN and SPI
from machine import Pin, SPI

# Import MicoPython max7219 library
import max7219

# Import time
import time

#Intialize the SPI
spi = SPI(0, baudrate=10000000, polarity=1, phase=0, sck=Pin(2), mosi=Pin(3))
ss = Pin(5, Pin.OUT)

# Create matrix display instant, which has four MAX7219 devices.
display = max7219.Matrix8x8(spi, ss, 1)

#Set the display brightness. Value is 1 to 15.
display.brightness(10)

#Define the scrolling message
scrolling_message = "RASPBERRY PI PICO AND MAX7219 -- 8x8 DOT MATRIX SCROLLING DISPLAY"

#Get the message length
length = len(scrolling_message)

#Calculate number of columns of the message
column = (length * 8)

#Clear the display.
display.fill(0)
display.show()

#sleep for one one seconds
time.sleep(1)

# Unconditionally execute the loop
while True:
  #display.fill(0)
  #display.text(message,1,0,1)
  #display.show()
  for x in range(32, -column, -1):     
    #Clear the display
    display.fill(0)

    # Write the scrolling text in to frame buffer
    display.text(scrolling_message ,x,0,1)
        
    #Show the display
    display.show()
      
    #Set the Scrolling speed. Here it is 50mS.
    time.sleep(0.1) '''