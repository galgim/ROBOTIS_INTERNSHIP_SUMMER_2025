#USES STRUCT FUNCTION TO PACK DATA INTO BITS AND UNPACK LATER TO READ

from machine import UART, Pin
import time
import espnow 
import network
import machine #for pwm
import struct

#turn on network shit
esp = network.WLAN(network.STA_IF)
esp.active(True)

#turn on espnow shit
lol = espnow.ESPNOW()
lol.active(True)

#uart shit
uart = UART(1, baudrate = 115200, tx = 17, rx = 16, timeout = 10)

#unpack and print this shit
while True:
    host, msg = lol.recv()
    if msg:
        val = struct.unpack('H', msg)[0]#using struct unpack to decode data 2 byte integer
        arr = [val, 0, 0, 0, 0, 0]
        packed_dummy = struct.pack('6H', *arr)

        #wrap message in "<>", this will get removed
        wrapped = b'<' + packed_dummy + b'>'
        
        uart.write(wrapped) #wrapped message to jetson
        print("encover value", val)
    time.sleep(0.05)

