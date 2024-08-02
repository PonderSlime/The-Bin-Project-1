from machine import Pin, ADC
#from speaker_machine import c_note, d_note
import utime

xAxis = ADC(Pin(27))
yAxis = ADC(Pin(26))

button = Pin(1,Pin.IN, Pin.PULL_UP)

def joy():
    xValue = xAxis.read_u16()
    yValue = yAxis.read_u16()
    buttonValue= button.value()
    print(str(xValue) +", " + str(yValue) + " -- " + str(buttonValue))
    #utime.sleep(0.1)

    xStatus = "middle"
    yStatus = "middle"
    buttonStatus = "not pressed"

    if xValue <= 600:
        xStatus = "left"
        #c_note()
    elif xValue >= 60000:
        xStatus = "right"
        #c_note()
    if yValue <= 600:
        yStatus = "up"
        #c_note()
    elif yValue >= 60000:
        yStatus = "down"
        #c_note()
    if buttonValue == 0:
        buttonStatus = "pressed"
        #d_note()

    print("X: " + xStatus + ", Y: " + yStatus + " -- button " + buttonStatus)

if __name__ == '__joy__':
    joy()