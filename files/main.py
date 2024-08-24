from machine import Pin, SPI, SoftI2C, ADC
from speaker_machine import c_note, d_note
import max7219
from pico_dht_22 import PicoDHT22
import ssd1306
from keypad import loop_keypad
from buzzer_music import music
#import joypad
#from joypad import joy, write_joy_vars
from rotary_irq_rp2 import RotaryIRQ
import time
import machine, neopixel

from machine import Pin, Timer
led = Pin("LED", Pin.OUT)
timer = Timer()

def blink(timer):
    led.toggle()

timer.init(freq=2.5, mode=Timer.PERIODIC, callback=blink)

song = '4 F#5 1 1;0 D5 1 1;2 E5 1 1;8 A5 1 1;12 F#5 1 1;16 E5 1 1;20 D5 1 1;32 F#5 1 1;34 A5 1 1;36 B5 1 1;40 D6 1 1;44 C#6 1 1;48 A5 1 1;52 F#5 1 1;60 E5 1 1;64 D5 1 1;66 E5 1 1;68 F#5 1 1;72 A5 1 1;76 F#5 1 1;80 E5 1 1;84 D5 1 1;96 F#5 1 1;98 A5 1 1;100 B5 1 1;116 E5 1 1;110 B5 1 1;112 A5 1 1;114 F#5 1 1;4 D3 1 5;12 F#3 1 5;20 G3 1 5;28 F#3 1 5;36 G3 1 5;44 A3 1 5;52 F#3 1 5;60 E3 1 5;68 D3 1 5;76 F#3 1 5;84 G3 1 5;92 F#3 1 5;100 G3 1 5;108 F#3 1 5;116 E3 1 5;4 D5 1 1;4 A4 1 1;12 C#5 1 1;12 A4 1 1;20 B4 1 1;20 G4 1 1;28 A4 1 1;28 F#4 1 1;36 D5 1 1;36 G5 1 1;44 E5 1 1;44 A5 1 1;52 D5 1 1;52 A4 1 1;60 C#5 1 1;60 A4 1 1;68 A4 1 1;68 D5 1 1;76 A4 1 1;76 C#5 1 1;84 B4 1 1;84 G4 1 1;92 F#4 1 1;92 A4 1 1;100 D5 1 1;100 G5 1 1;108 D5 1 1;108 F#5 1 1;124 E4 1 1;116 A4 1 1;116 C#5 1 1;124 C5 1 1;124 G4 1 1;132 D5 1 1;134 E5 1 1;136 F#5 1 1;132 D4 1 1;132 F#4 1 1;132 B4 1 1;140 E4 1 1;140 A4 1 1;140 C#5 1 1;156 C#5 1 1;156 A4 1 1;156 E4 1 1;148 D5 1 1;148 B4 1 1;148 F#4 1 1;162 C#6 1 1;158 F#5 1 1;160 B5 1 1;164 D6 1 1;164 G3 1 5;172 A3 1 5;188 E3 1 5;164 B5 1 1;164 G5 1 1;172 C#6 1 1;172 E5 1 1;172 A5 1 1;176 A5 1 1;180 D3 1 5;180 D5 1 1;180 F#5 1 1;180 A4 1 1;188 A4 1 1;188 C#5 1 1;188 E5 1 1;194 E5 1 1;192 D5 1 1;196 F#5 1 1;196 D3 1 5;204 E3 1 5;212 G3 1 5;196 F#4 1 1;196 B4 1 1;204 A4 1 1;204 C#5 1 1;220 F#3 1 5;212 B4 1 1;212 D5 1 1;220 C#5 1 1;220 A4 1 1;222 F#5 1 1;224 B5 1 1;226 C#6 1 1;228 D6 1 1;228 G3 1 5;236 A3 1 5;244 B3 1 5;228 B5 1 1;228 G5 1 1;236 E6 1 1;236 C#6 1 1;236 A5 1 1;242 D6 1 1;243 E6 1 1;244 F#6 1 1;244 B5 1 1;244 D6 1 1;252 G3 1 5;260 D4 1 1;256 B5 1 1;254 D6 1 1;258 D6 1 1;264 D4 1 1;268 D4 1 1;272 D4 1 1;262 A4 1 1;262 F#4 1 1;266 A4 1 1;266 F#4 1 1;270 A4 1 1;270 F#4 1 1;274 A4 1 1;274 F#4 1 1;260 D6 1 11;261 E6 1 11;262 F#6 1 11;270 F#6 1 11;274 F#6 1 11;278 A4 1 1;278 C#5 1 1;276 C#4 1 1;280 C#4 1 1;284 C#4 1 1;288 C#4 1 1;282 A4 1 1;286 A4 1 1;290 A4 1 1;290 C#5 1 1;286 C#5 1 1;282 C#5 1 1;276 A6 1 11;278 E6 1 11;280 D6 1 11;282 E6 1 11;277 B6 1 11;277 A6 1 11;294 C#6 1 11;292 A5 1 11;293 B5 1 11;292 B3 1 1;296 B3 1 1;300 B3 1 1;304 B3 1 1;308 G3 1 1;312 G3 1 1;316 A3 1 1;320 A3 1 1;294 D4 1 1;298 D4 1 1;302 D4 1 1;306 D4 1 1;310 D4 1 1;314 D4 1 1;318 E4 1 1;322 E4 1 1;294 F#4 1 1;298 F#4 1 1;302 F#4 1 1;306 F#4 1 1;310 B3 1 1;314 B3 1 1;322 C#4 1 1;298 C#6 1 11;302 D6 1 11;306 B5 1 11;308 F#5 1 11;314 A5 1 11;316 E5 1 11;318 C#4 1 1;294 C#6 1 11;292 A5 1 11;293 B5 1 11;298 C#6 1 11;302 D6 1 11;306 B5 1 11;308 F#5 1 11;314 A5 1 11;316 E5 1 11;260 D6 1 11;261 E6 1 11;262 F#6 1 11;270 F#6 1 11;274 F#6 1 11;276 A6 1 11;278 E6 1 11;280 D6 1 11;282 E6 1 11;277 B6 1 11;277 A6 1 11;132 D3 1 5;140 E3 1 5;156 E3 1 5;124 C3 1 5;148 F#3 1 5;324 A3 1 1;328 A5 1 11;330 C#6 1 11;333 E6 1 11;334 F#6 1 11;332 D4 1 1;336 D4 1 1;340 D4 1 1;344 D4 1 1;334 A4 1 1;334 F#4 1 1;338 F#4 1 1;342 F#4 1 1;346 F#4 1 1;346 A4 1 1;342 A4 1 1;338 A4 1 1;338 F#6 1 11;346 A6 1 11;348 F#6 1 11;348 G3 1 1;352 G3 1 1;356 A3 1 1;360 A3 1 1;350 D4 1 1;354 D4 1 1;358 E4 1 1;362 E4 1 1;354 B3 1 1;350 B3 1 1;358 C#4 1 1;362 C#4 1 1;350 E6 1 11;354 D6 1 11;356 E6 1 11;349 G6 1 11;349 F#6 1 11;358 C#6 1 11;332 D6 1 11;328 A5 1 11;330 C#6 1 11;333 E6 1 11;334 F#6 1 11;338 F#6 1 11;346 A6 1 11;348 F#6 1 11;350 E6 1 11;354 D6 1 11;356 E6 1 11;349 G6 1 11;349 F#6 1 11;358 C#6 1 11;332 D6 1 11;362 C#6 1 11;363 D6 1 11;364 C#6 1 11;364 D6 1 11;365 E6 1 11;366 F#6 1 11;364 D5 1 10;365 E5 1 10;366 F#5 1 10;364 B3 1 1;366 D4 1 1;366 F#4 1 1;368 B3 1 1;372 B3 1 1;376 B3 1 1;378 D4 1 1;374 D4 1 1;370 D4 1 1;370 F#4 1 1;374 F#4 1 1;378 F#4 1 1;377 F#6 1 11;377 F#5 1 10;378 B6 1 11;379 C#7 1 11;380 D7 1 11;380 D6 1 10;378 B5 1 10;379 C#6 1 10;380 G3 1 1;384 G3 1 1;388 G3 1 1;392 G3 1 1;362 C#6 1 11;363 D6 1 11;364 C#6 1 11;364 D6 1 11;365 E6 1 11;366 F#6 1 11;364 D5 1 10;365 E5 1 10;366 F#5 1 10;377 F#6 1 11;377 F#5 1 10;378 B6 1 11;379 C#7 1 11;380 D7 1 11;380 D6 1 10;378 B5 1 10;379 C#6 1 10;382 B3 1 1;382 D4 1 1;386 D4 1 1;390 D4 1 1;394 D4 1 1;394 B3 1 1;390 B3 1 1;386 B3 1 1;386 C#7 1 11;386 C#6 1 10;386 C#6 1 10;386 C#7 1 11;394 A6 1 11;394 A5 1 10;394 A5 1 10;394 A6 1 11;396 A3 1 1;398 C#4 1 1;398 E4 1 1;402 E4 1 1;406 E4 1 1;410 E4 1 1;410 C#4 1 1;406 C#4 1 1;402 C#4 1 1;400 A3 1 1;404 A3 1 1;408 A3 1 1;396 F#6 1 11;402 G6 1 11;403 F#6 1 11;404 E6 1 11;396 F#6 1 11;402 G6 1 11;403 F#6 1 11;404 E6 1 11;396 F#5 1 10;396 F#5 1 10;402 G5 1 10;402 G5 1 10;403 F#5 1 10;403 F#5 1 10;404 E5 1 10;404 E5 1 10;410 D6 1 11;411 E6 1 11;410 D6 1 11;411 E6 1 11;410 D5 1 10;410 D5 1 10;411 E5 1 10;411 E5 1 10;412 F#6 1 11;412 F#5 1 10;412 F#5 1 10;412 F#6 1 11;412 B3 1 1;416 B3 1 1;420 B3 1 1;424 B3 1 1;414 D4 1 1;414 F#4 1 1;418 F#4 1 1;422 F#4 1 1;426 F#4 1 1;426 D4 1 1;422 D4 1 1;418 D4 1 1;424 F#6 1 11;424 F#5 1 10;424 F#5 1 10;424 F#6 1 11;426 F#6 1 11;427 G6 1 11;426 F#5 1 10;427 G5 1 10;426 F#5 1 10;427 G5 1 10;426 F#6 1 11;427 G6 1 11;428 A3 1 1;432 A3 1 1;436 A3 1 1;440 A3 1 1;430 C#4 1 1;430 E4 1 1;434 E4 1 1;438 E4 1 1;442 E4 1 1;442 C#4 1 1;438 C#4 1 1;434 C#4 1 1;428 A6 1 11;428 A5 1 10;428 E6 1 11;428 E6 1 11;428 A6 1 11;428 A5 1 10;436 E6 1 11;436 C#6 1 11;436 E5 1 10;436 E5 1 10;436 C#6 1 11;436 E6 1 11;444 D5 1 10;444 D6 1 11;444 D4 1 1;444 D5 1 10;444 D6 1 11;445 E6 1 11;446 F#6 1 11;445 E5 1 10;446 F#5 1 10;445 E5 1 10;446 F#5 1 10;445 E6 1 11;446 F#6 1 11;448 D4 1 1;452 D4 1 1;456 D4 1 1;446 F#4 1 1;446 A4 1 1;450 A4 1 1;454 A4 1 1;458 A4 1 1;458 F#4 1 1;454 F#4 1 1;450 F#4 1 1;454 F#6 1 11;454 F#5 1 10;454 F#5 1 10;454 D6 1 11;454 D6 1 11;454 F#6 1 11;458 F#6 1 11;458 D6 1 11;458 F#5 1 10;458 D6 1 11;458 F#6 1 11;460 E6 1 11;461 F#6 1 11;462 G#6 1 11;460 E6 1 11;461 F#6 1 11;462 G#6 1 11;460 E5 1 10;461 F#5 1 10;462 G#5 1 10;461 F#5 1 10;462 G#5 1 10;460 E4 1 1;462 G#4 1 1;462 B4 1 1;466 B4 1 1;470 B4 1 1;474 B4 1 1;474 G#4 1 1;470 G#4 1 1;466 G#4 1 1;464 E4 1 1;468 E4 1 1;472 E4 1 1;466 G#6 1 11;470 G#6 1 11;474 G#6 1 11;466 G#5 1 10;470 G#5 1 10;474 G#5 1 10;466 E6 1 11;470 E6 1 11;474 E6 1 11;466 G#6 1 11;470 G#6 1 11;474 G#6 1 11;466 G#5 1 10;474 G#5 1 10;466 E6 1 11;470 E6 1 11;474 E6 1 11;476 F#4 1 1;476 F#5 1 10;476 F#6 1 11;477 G#6 1 11;478 A#6 1 11;477 G#5 1 10;478 A#5 1 10;476 F#5 1 10;477 G#5 1 10;478 A#5 1 10;476 F#6 1 11;477 G#6 1 11;478 A#6 1 11;478 A#4 1 1;478 C#5 1 1;480 F#4 1 1;482 A#4 1 1;482 C#5 1 1;484 F#4 1 1;484 A#4 1 1;484 C#5 1 1;484 F#5 1 1;484 A#5 1 10;484 F#6 1 11;484 A#6 1 11;486 A#6 1 11;486 F#6 1 11;486 A#5 1 10;486 F#5 1 1;486 C#5 1 1;486 A#4 1 1;486 F#4 1 1;490 A#6 1 11;491 A#6 1 11;492 A#6 1 11;490 F#6 1 11;491 F#6 1 11;492 F#6 1 11;490 A#5 1 10;491 A#5 1 10;492 A#5 1 10;490 F#5 1 1;491 F#5 1 1;492 F#5 1 1;490 C#5 1 1;491 C#5 1 1;492 C#5 1 1;490 A#4 1 1;491 A#4 1 1;492 A#4 1 1;490 F#4 1 1;491 F#4 1 1;492 F#4 1 1;484 F#4 1 1;484 A#4 1 1;484 C#5 1 1;484 F#5 1 1;484 A#5 1 10;484 F#6 1 11;484 A#6 1 11;486 A#6 1 11;486 F#6 1 11;486 A#5 1 10;486 F#5 1 1;486 C#5 1 1;486 A#4 1 1;486 F#4 1 1;490 A#6 1 11;491 A#6 1 11;492 A#6 1 11;490 F#6 1 11;491 F#6 1 11;492 F#6 1 11;490 A#5 1 10;491 A#5 1 10;492 A#5 1 10;490 F#5 1 1;491 F#5 1 1;492 F#5 1 1;490 C#5 1 1;491 C#5 1 1;492 C#5 1 1;490 A#4 1 1;491 A#4 1 1;492 A#4 1 1;490 F#4 1 1;491 F#4 1 1;492 F#4 1 1;256 B5 1 11;254 D6 1 11;258 D6 1 11;260 D3 1 5;264 D3 1 5;268 D3 1 5;272 D3 1 5;276 C#3 1 5;280 C#3 1 5;284 C#3 1 5;288 C#3 1 5;292 B2 1 5;296 B2 1 5;300 B2 1 5;304 B2 1 5;308 G2 1 5;312 G2 1 5;316 A2 1 5;320 A2 1 5;332 D3 1 5;336 D3 1 5;340 D3 1 5;344 D3 1 5;348 G2 1 5;352 G2 1 5;356 A2 1 5;360 A2 1 5;364 B2 1 5;368 B2 1 5;372 B2 1 5;376 B2 1 5;380 G2 1 5;384 G2 1 5;388 G2 1 5;392 G2 1 5;396 A2 1 5;400 A2 1 5;404 A2 1 5;408 A2 1 5;412 B2 1 5;416 B2 1 5;420 B2 1 5;424 B2 1 5;428 A2 1 5;432 A2 1 5;436 A2 1 5;440 A2 1 5;444 D3 1 5;448 D3 1 5;452 D3 1 5;456 D3 1 5;460 E3 1 5;464 E3 1 5;468 E3 1 5;472 E3 1 5;476 F#3 1 5;480 F#3 1 5;484 F#3 1 5;486 F#3 1 5;490 F#3 1 5;491 F#3 1 5;492 F#3 1 5'
# creating a Speaker object

# continuously beep at 1 sec interval while the board has power
# note: a passive buzzer can also be used to play different tones
# 
dht_sensor=PicoDHT22(Pin(15,Pin.IN,Pin.PULL_UP),dht11=True)
# #You can choose any other combination of I2C pins
i2c = SoftI2C(scl=Pin(19), sda=Pin(18))
 
oled_width = 128
oled_height = 64
oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)
 
#Find a piece of music on onlinesequencer.net, click edit,
#then select all notes with CTRL+A and copy them with CTRL+C
 
#Paste string as shown above after removing ";:" from
#the end and "Online Sequencer:120233:" from the start

#One buzzer on pin 14
mySong = music(song, pins=[Pin(14)])

#Intialize the SPI
spi = SPI(0, baudrate=10000000, polarity=1, phase=0, sck=Pin(2), mosi=Pin(3))
ss = Pin(5, Pin.OUT)

# Create matrix display instant, which has four MAX7219 devices.
display = max7219.Matrix8x8(spi, ss, 1)
 
#Set the display brightness. Value is 1 to 15.
display.brightness(5)

#Define the scrolling message
scrolling_message = "RASPBERRY PI PICO AND MAX7219 -- 8x8 DOT MATRIX SCROLLING DISPLAY"

#Get the message length
length = len(scrolling_message)

#Calculate number of columns of the message
column = (length * 8)

# #Clear the display.
display.fill(0)
display.show()

#sleep for one one seconds
time.sleep(1)
scroll_ammt = 32
 
# np = neopixel.NeoPixel(machine.Pin(0), 1)
# np[0] = (255, 0, 0) # set to red, full brightness
# np.write()
# 
# 
# SW = Pin(22,Pin.IN,Pin.PULL_UP)  
# 
# r = RotaryIRQ(pin_num_clk=20,
#               pin_num_dt=17,
#               min_val=0,
#               reverse=False,
#               range_mode=RotaryIRQ.RANGE_UNBOUNDED)
# val_old = r.value()
# 
# 
# #c_note()
# #main()
# xAxis = ADC(Pin(27))
# yAxis = ADC(Pin(26))
# 
# button = Pin(1,Pin.IN, Pin.PULL_UP)
# def joy():
#     xValue = xAxis.read_u16()
#     yValue = yAxis.read_u16()
#     buttonValue= button.value()
#     #print(str(xValue) +", " + str(yValue) + " -- " + str(buttonValue))
#     #utime.sleep(0.1)
# 
#     xStatus = "middle"
#     yStatus = "middle"
#     buttonStatus = "not pressed"
# 
#     if xValue <= 600:
#         xStatus = "left"
#         np[0] = (68, 122, 133) # set to my favorite shade of blue, full brightness
#         np.write()
#         #c_note()
#     elif xValue >= 60000:
#         xStatus = "right"
#         np[0] = (191, 73, 146) # set to deep pink, full brightness
#         np.write()
#         #c_note()
#     if yValue <= 600:
#         yStatus = "up"
#         np[0] = (191, 163, 73) # set to pale orange, full brightness
#         np.write()
#         #c_note()
#     elif yValue >= 60000:
#         yStatus = "down"
#         np[0] = (79, 191, 73) # set to light green, full brightness
#         np.write()
#         #c_note()
#     if buttonValue == 0:
#         buttonStatus = "pressed"
#         np[0] = (166, 73, 191) # set to purple, full brightness
#         np.write()
#         d_note()
# 
#     #print("X: " + xStatus + ", Y: " + yStatus + " -- button " + buttonStatus)
#
oled.fill(0)
oled.text("Hello, World!", 10, 10)
oled.show()
temp_tick = 0
lasttime = time.ticks_ms()
lasttimesong = time.ticks_ms()
lasttimematrix = time.ticks_ms()

while True:
#     
    
#   loop_keypad()
#   joy()
    devices = i2c.scan()

    if len(devices) == 0:
      print("No i2c device !")
    else:
      if time.ticks_ms() > 60000 + lasttime:
        
        T,H = dht_sensor.read()
        if T is None:
            continue
            print(" sensor error")
            #d_note()
        else:
            print("{}'C  {}%".format(T,H))
            oled.fill(0)
            oled.text(str('H: ' +"{:0.2f}".format(H)+ "  %",2),40,5)
            oled.text(str('T: ' +"{:0.2f}".format(T)+ "  C",2),40,19)
            oled.show()
        lasttime = time.ticks_ms()
    if time.ticks_ms() > 40 + lasttimesong:
        lasttimesong = time.ticks_ms()
        print(mySong.tick())
        
        
    
        
   # time.sleep(0.04)
   
    if time.ticks_ms() > 75 + lasttimematrix:
        lasttimematrix = time.ticks_ms()  
        scroll_ammt -= 1
        
    if scroll_ammt in range(32, -column, -1):
        display.fill(0)
 
           # Write the scrolling text in to frame buffer
        display.text(scrolling_message ,scroll_ammt,0,1)
           
           #Show the display
        display.show()
       
           #Set the Scrolling speed. Here it is 50mS.
       #time.sleep(0.0)
    else:
        scroll_ammt = 32
           
#   try:  
#     val_new = r.value()  
#     if SW.value()==0 and n==0:  
#         print("Button Pressed")  
#         print("Selected Number is : ",val_new) 
#         #c_note()
#         np[0] = (255, 255, 0) # set to yellow, full brightness
#         np.write()
#         n=1 
#         while SW.value()==0:  
#             continue  
#     n=0  
#     if val_old != val_new:  
#         val_old = val_new  
#         print('result =', val_new)
#         #c_note()
#         np[0] = (0, 255, 255) # set to teal, full brightness
#         np.write()
#   
#   except KeyboardInterrupt:  
#     break
#   #time.sleep_ms(50)'''
