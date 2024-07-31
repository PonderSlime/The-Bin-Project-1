#import sys
#import time

#from keypad import Keypad
from machine import Pin
import time
#import keypad
from keypad import loop_keypad
#from screen import loop_screen
#import screen
from screen import main
from speaker_machine import c_note

main()
while True:
  loop_keypad()
  #loop_screen()
#c_note()

#from keypad import Keypad


''' import sys
from rotary_irq_rp2 import RotaryIRQ
import time
from speaker_machine import c_note

r = RotaryIRQ(pin_num_clk=13,
              pin_num_dt=14,
              min_val=0,
              max_val=5,
              reverse=False,
              range_mode=RotaryIRQ.RANGE_WRAP)

val_old = r.value()
while True:
    val_new = r.value()

    if val_old != val_new:
      c_note()
        val_old = val_new
        print('result =', val_new)

    time.sleep_ms(50) '''
