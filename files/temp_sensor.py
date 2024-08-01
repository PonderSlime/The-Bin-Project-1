from machine import Pin, SPI
from pico_dht_22 import PicoDHT22
from speaker_machine import d2_note

dht_sensor=PicoDHT22(Pin(15,Pin.IN,Pin.PULL_UP),dht11=True)

def tem_check():
    T,H = dht_sensor.read()
    if T is None:
        print(" sensor error")
        d_note()
    else:
        print("{}'C  {}%".format(T,H))
        #d2_note()
    #DHT22 not responsive if delay too short