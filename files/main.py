from machine import Pin, SPI, SoftI2C, ADC
#from speaker_machine import c_note, d_note
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
song = '0 C4 1 35;4 A#3 1 35;6 A#3 1 35;10 C4 1 35;8 C4 1 35;12 G3 1 35;16 C4 1 35;20 A#3 1 35;22 A#3 1 35;26 C4 1 35;24 C4 1 35;28 G3 1 35;32 C4 1 35;36 A#3 1 35;38 A#3 1 35;42 C4 1 35;40 C4 1 35;44 G3 1 35;48 C4 1 35;52 A#3 1 35;54 A#3 1 35;58 C4 1 35;56 C4 1 35;60 G3 1 35;32 C5 1 0;36 D#5 1 0;38 G5 1 0;48 F#5 1 0;52 D#5 1 0;54 C5 1 0;64 C5 1 0;68 D#5 1 0;70 G5 1 0;74 G#5 1 0;76 G5 1 0;80 F#5 1 0;84 D#5 1 0;86 C5 1 0;64 C4 1 35;68 A#3 1 35;70 A#3 1 35;74 C4 1 35;72 C4 1 35;76 G3 1 35;80 C4 1 35;84 A#3 1 35;86 A#3 1 35;90 C4 1 35;88 C4 1 35;92 G3 1 35;96 F4 1 35;100 D#4 1 35;102 D#4 1 35;106 F4 1 35;104 F4 1 35;108 C4 1 35;112 F4 1 35;116 D#4 1 35;118 D#4 1 35;122 F4 1 35;120 F4 1 35;124 C4 1 35;32 C4 1 0;36 D#4 1 0;38 G4 1 0;48 F#4 1 0;52 D#4 1 0;54 C4 1 0;64 C4 1 0;68 D#4 1 0;70 G4 1 0;74 G#4 1 0;76 G4 1 0;80 F#4 1 0;84 D#4 1 0;86 C4 1 0;128 C4 1 35;132 A#3 1 35;134 A#3 1 35;138 C4 1 35;136 C4 1 35;140 G3 1 35;144 C4 1 35;148 A#3 1 35;150 A#3 1 35;154 C4 1 35;152 C4 1 35;156 G3 1 35;96 F5 1 0;100 G#5 1 0;102 C6 1 0;112 B5 1 0;116 G#5 1 0;118 F5 1 0;128 C5 1 0;132 D#5 1 0;134 G5 1 0;138 G#5 1 0;140 G5 1 0;144 F#5 1 0;148 D#5 1 0;150 C5 1 0;96 F4 1 0;100 G#4 1 0;102 C5 1 0;112 B4 1 0;116 G#4 1 0;118 F4 1 0;128 C4 1 0;132 D#4 1 0;134 G4 1 0;138 G#4 1 0;140 G4 1 0;144 F#4 1 0;148 D#4 1 0;150 C4 1 0;156 G#5 1 0;158 G5 1 0;160 G4 1 35;164 F4 1 35;166 F4 1 35;168 G4 1 35;170 G4 1 35;172 D4 1 35;176 G4 1 35;180 F4 1 35;182 F4 1 35;184 G4 1 35;186 G4 1 35;188 D4 1 35;192 C4 1 35;196 A#3 1 35;198 A#3 1 35;200 C4 1 35;202 C4 1 35;204 G3 1 35;208 C4 1 35;212 A#3 1 35;214 A#3 1 35;216 C4 1 35;218 C4 1 35;220 G3 1 35;176 F5 1 0;178 F5 1 0;182 D#5 1 0;184 F5 1 0;188 D#5 1 0;190 C5 1 0;156 G#4 1 0;158 G4 1 0;176 F4 1 0;178 F4 1 0;182 D#4 1 0;184 F4 1 0;188 D#4 1 0;190 C4 1 0;200 D7 1 0;200 B6 1 0;200 G#6 1 0;200 G6 1 0;200 D6 1 0;200 B5 1 0;200 C6 1 0;208 D7 1 0;208 B6 1 0;208 G#6 1 0;208 G6 1 0;208 D6 1 0;208 C6 1 0;208 B5 1 0;216 D7 1 0;216 B6 1 0;216 G#6 1 0;216 G6 1 0;216 D6 1 0;216 C6 1 0;216 B5 1 0;224 C4 1 35;228 A#3 1 35;230 A#3 1 35;234 C4 1 35;232 C4 1 35;236 G3 1 35;240 C4 1 35;244 A#3 1 35;246 A#3 1 35;250 C4 1 35;248 C4 1 35;252 G3 1 35;224 C5 1 0;228 D#5 1 0;230 G5 1 0;240 F#5 1 0;244 D#5 1 0;246 C5 1 0;256 C5 1 0;260 D#5 1 0;262 G5 1 0;266 G#5 1 0;268 G5 1 0;272 F#5 1 0;276 D#5 1 0;278 C5 1 0;256 C4 1 35;260 A#3 1 35;262 A#3 1 35;266 C4 1 35;264 C4 1 35;268 G3 1 35;272 C4 1 35;276 A#3 1 35;278 A#3 1 35;282 C4 1 35;280 C4 1 35;284 G3 1 35;288 F4 1 35;292 D#4 1 35;294 D#4 1 35;298 F4 1 35;296 F4 1 35;300 C4 1 35;304 F4 1 35;308 D#4 1 35;310 D#4 1 35;314 F4 1 35;312 F4 1 35;316 C4 1 35;224 C4 1 0;228 D#4 1 0;230 G4 1 0;240 F#4 1 0;244 D#4 1 0;246 C4 1 0;256 C4 1 0;260 D#4 1 0;262 G4 1 0;266 G#4 1 0;268 G4 1 0;272 F#4 1 0;276 D#4 1 0;278 C4 1 0;320 C4 1 35;324 A#3 1 35;326 A#3 1 35;330 C4 1 35;328 C4 1 35;332 G3 1 35;336 C4 1 35;340 A#3 1 35;342 A#3 1 35;346 C4 1 35;344 C4 1 35;348 G3 1 35;288 F5 1 0;292 G#5 1 0;294 C6 1 0;304 B5 1 0;308 G#5 1 0;310 F5 1 0;320 C5 1 0;324 D#5 1 0;326 G5 1 0;330 G#5 1 0;332 G5 1 0;336 F#5 1 0;340 D#5 1 0;342 C5 1 0;288 F4 1 0;292 G#4 1 0;294 C5 1 0;304 B4 1 0;308 G#4 1 0;310 F4 1 0;320 C4 1 0;324 D#4 1 0;326 G4 1 0;330 G#4 1 0;332 G4 1 0;336 F#4 1 0;340 D#4 1 0;342 C4 1 0;348 G#5 1 0;350 G5 1 0;352 G4 1 35;356 F4 1 35;358 F4 1 35;360 G4 1 35;362 G4 1 35;364 D4 1 35;368 G4 1 35;372 F4 1 35;374 F4 1 35;376 G4 1 35;378 G4 1 35;380 D4 1 35;384 C4 1 35;388 A#3 1 35;390 A#3 1 35;392 C4 1 35;394 C4 1 35;396 G3 1 35;400 C4 1 35;404 A#3 1 35;406 A#3 1 35;408 C4 1 35;410 C4 1 35;412 G3 1 35;368 F5 1 0;370 F5 1 0;374 D#5 1 0;376 F5 1 0;380 D#5 1 0;382 C5 1 0;348 G#4 1 0;350 G4 1 0;368 F4 1 0;370 F4 1 0;374 D#4 1 0;376 F4 1 0;380 D#4 1 0;382 C4 1 0;392 D7 1 0;392 B6 1 0;392 G#6 1 0;392 G6 1 0;392 D6 1 0;392 B5 1 0;392 C6 1 0;400 D7 1 0;400 B6 1 0;400 G#6 1 0;400 G6 1 0;400 D6 1 0;400 C6 1 0;400 B5 1 0;408 G5 1 0;412 G#5 1 0;416 A#5 1 0;416 A#3 1 35;420 G#3 1 35;422 G#3 1 35;424 A#3 1 35;426 A#3 1 35;428 D3 1 35;432 D#3 1 35;436 F3 1 35;438 F3 1 35;440 G3 1 35;442 G3 1 35;444 D#3 1 35;448 D3 1 35;452 F3 1 35;454 F3 1 35;456 G3 1 35;458 G3 1 35;460 D3 1 35;464 C3 1 35;468 D#3 1 35;472 G3 1 35;476 D#3 1 35;480 A#3 1 35;484 G#3 1 35;486 G#3 1 35;488 A#3 1 35;490 A#3 1 35;492 D3 1 35;496 D#3 1 35;500 F3 1 35;502 F3 1 35;504 G3 1 35;506 G3 1 35;508 D#3 1 35;512 D3 1 35;516 F3 1 35;518 F3 1 35;520 G3 1 35;522 G3 1 35;524 C4 1 35;528 B3 1 35;532 G3 1 35;536 G3 1 35;540 G3 1 35;428 G#5 1 0;430 G5 1 0;440 F5 1 0;444 D#5 1 0;448 F5 1 0;452 F5 1 0;456 D#5 1 0;460 D5 1 0;462 C5 1 0;472 G5 1 0;476 G#5 1 0;480 A#5 1 0;492 G#5 1 0;494 G5 1 0;504 F5 1 0;508 D#5 1 0;512 F5 1 0;516 F5 1 0;520 D#5 1 0;524 F5 1 0;526 G5 1 0;408 D#5 1 0;412 C5 1 0;416 D5 1 0;428 C5 1 0;430 A#4 1 0;440 G#4 1 0;444 G4 1 0;448 G#4 1 0;452 G#4 1 0;456 G4 1 0;460 F4 1 0;462 D#4 1 0;472 D#5 1 0;476 C5 1 0;480 D5 1 0;492 C5 1 0;494 A#4 1 0;504 G#4 1 0;508 G4 1 0;512 G#4 1 0;516 G#4 1 0;520 C5 1 0;524 C5 1 0;526 B4 1 0;544 C4 1 35;548 A#3 1 35;550 A#3 1 35;554 C4 1 35;552 C4 1 35;556 G3 1 35;560 C4 1 35;564 A#3 1 35;566 A#3 1 35;570 C4 1 35;568 C4 1 35;572 G3 1 35;544 C5 1 0;548 D#5 1 0;550 G5 1 0;560 F#5 1 0;564 D#5 1 0;566 C5 1 0;576 C5 1 0;580 D#5 1 0;582 G5 1 0;586 G#5 1 0;588 G5 1 0;592 F#5 1 0;596 D#5 1 0;598 C5 1 0;576 C4 1 35;580 A#3 1 35;582 A#3 1 35;586 C4 1 35;584 C4 1 35;588 G3 1 35;592 C4 1 35;596 A#3 1 35;598 A#3 1 35;602 C4 1 35;600 C4 1 35;604 G3 1 35;608 F4 1 35;612 D#4 1 35;614 D#4 1 35;618 F4 1 35;616 F4 1 35;620 C4 1 35;624 F4 1 35;628 D#4 1 35;630 D#4 1 35;634 F4 1 35;632 F4 1 35;636 C4 1 35;544 C4 1 0;548 D#4 1 0;550 G4 1 0;560 F#4 1 0;564 D#4 1 0;566 C4 1 0;576 C4 1 0;580 D#4 1 0;582 G4 1 0;586 G#4 1 0;588 G4 1 0;592 F#4 1 0;596 D#4 1 0;598 C4 1 0;640 C4 1 35;644 A#3 1 35;646 A#3 1 35;650 C4 1 35;648 C4 1 35;652 G3 1 35;656 C4 1 35;660 A#3 1 35;662 A#3 1 35;666 C4 1 35;664 C4 1 35;668 G3 1 35;608 F5 1 0;612 G#5 1 0;614 C6 1 0;624 B5 1 0;628 G#5 1 0;630 F5 1 0;640 C5 1 0;644 D#5 1 0;646 G5 1 0;650 G#5 1 0;652 G5 1 0;656 F#5 1 0;660 D#5 1 0;662 C5 1 0;608 F4 1 0;612 G#4 1 0;614 C5 1 0;624 B4 1 0;628 G#4 1 0;630 F4 1 0;640 C4 1 0;644 D#4 1 0;646 G4 1 0;650 G#4 1 0;652 G4 1 0;656 F#4 1 0;660 D#4 1 0;662 C4 1 0;668 G#5 1 0;670 G5 1 0;672 G4 1 35;676 F4 1 35;678 F4 1 35;680 G4 1 35;682 G4 1 35;684 D4 1 35;688 G4 1 35;692 F4 1 35;694 F4 1 35;696 G4 1 35;698 G4 1 35;700 D4 1 35;688 F5 1 0;690 F5 1 0;694 D#5 1 0;696 F5 1 0;700 D#5 1 0;702 C5 1 0;668 G#4 1 0;670 G4 1 0;688 F4 1 0;690 F4 1 0;694 D#4 1 0;696 F4 1 0;700 D#4 1 0;702 C4 1 0;704 C4 1 35;708 A#3 1 35;710 A#3 1 35;714 C4 1 35;712 C4 1 35;716 G3 1 35;736 C4 1 35;740 A#3 1 35;742 A#3 1 35;746 C4 1 35;744 C4 1 35;748 G3 1 35;752 G4 1 35;756 F4 1 35;758 F4 1 35;762 G4 1 35;760 G4 1 35;764 D4 1 35;720 G4 1 35;724 F4 1 35;726 F4 1 35;728 G4 1 35;730 G4 1 35;732 D4 1 35;768 C4 1 35;710 D#5 1 0;720 F5 1 0;722 F5 1 0;726 D#5 1 0;728 F5 1 0;732 D#5 1 0;734 C5 1 0;742 D#5 1 0;752 G5 1 0;754 F5 1 0;758 D#5 1 0;760 F5 1 0;764 G5 1 0;766 D6 1 0;710 D#4 1 0;720 F4 1 0;722 F4 1 0;726 D#4 1 0;728 F4 1 0;732 D#4 1 0;734 C4 1 0;742 D#4 1 0;752 G4 1 0;754 F4 1 0;758 D#4 1 0;760 F4 1 0;764 G4 1 0;766 D5 1 0;766 B5 1 0;766 G5 1 0'
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
xAxis = ADC(Pin(27))
yAxis = ADC(Pin(26))
 
button = Pin(1,Pin.IN, Pin.PULL_UP)
def joy():
    xValue = xAxis.read_u16()
    yValue = yAxis.read_u16()
    buttonValue= button.value()
    #print(str(xValue) +", " + str(yValue) + " -- " + str(buttonValue))
    #utime.sleep(0.1)

    xStatus = "middle"
    yStatus = "middle"
    buttonStatus = "not pressed"

    if xValue <= 600:
        xStatus = "left"
        #np[0] = (68, 122, 133) # set to my favorite shade of blue, full brightness
        #np.write()
        #c_note()
    elif xValue >= 60000:
        xStatus = "right"
        #np[0] = (191, 73, 146) # set to deep pink, full brightness
        #np.write()
        #c_note()
    if yValue <= 600:
        yStatus = "up"
        #np[0] = (191, 163, 73) # set to pale orange, full brightness
        #np.write()
        #c_note()
    elif yValue >= 60000:
        yStatus = "down"
        #np[0] = (79, 191, 73) # set to light green, full brightness
        #np.write()
        #c_note()
    if buttonValue == 0:
        buttonStatus = "pressed"
        #np[0] = (166, 73, 191) # set to purple, full brightness
        #np.write()
        #d_note()

    print("X: " + xStatus + ", Y: " + yStatus + " -- button " + buttonStatus)
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
    joy()
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

