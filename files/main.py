from machine import Pin
import time
from picozero import Speaker
from speaker_machine import c_note
from screen import main
from keypad import Keypad

while True:
    # Define the keypad matrix
    #print("played note")
    keypad = Keypad(
        rows=[Pin(2), Pin(3), Pin(4), Pin(5)],
        columns=[Pin(6), Pin(7), Pin(8), Pin(9)]
    )
    print('defined pins')
    # Initialize the keypad
    keypad.init()
    print('init keypad')

    # Define a variable to store the entered characters
    entered_characters = []
    print('ready!')
    # Main loop
    while True:
        print('looping!')
        key = keypad.get_key()
        print(key)
        time.sleep(0.1)

        if key:
            entered_characters.append(key)
            time.sleep(0.5)  # Adjust sleep duration as needed
            print(entered_characters)
        time.sleep(0.1)

c_note()
main()