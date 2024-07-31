

import time
from rotary_irq_rp2 import RotaryIRQ
from machine import Pin
from keypad import loop_keypad
from screen import main
from speaker_machine import c_note
from joypad import joy

SW = Pin(22,Pin.IN,Pin.PULL_UP)  

r = RotaryIRQ(pin_num_clk=20,
              pin_num_dt=17,
              min_val=0,
              reverse=False,
              range_mode=RotaryIRQ.RANGE_UNBOUNDED)
main()
val_old = r.value()
while True:
  loop_keypad()
  joy()
  try:  
    val_new = r.value()  
    if SW.value()==0 and n==0:  
        print("Button Pressed")  
        print("Selected Number is : ",val_new)
        c_note()
        n=1 
        while SW.value()==0:  
            continue  
    n=0  
    if val_old != val_new:  
        val_old = val_new  
        print('result =', val_new)  
    time.sleep_ms(50)  
  except KeyboardInterrupt:  
    break