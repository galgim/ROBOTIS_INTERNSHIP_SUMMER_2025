import network
import espnow
from machine import Pin
from rotary_irq_esp import RotaryIRQ
import time
import struct


station = network.WLAN(network.STA_IF)
station.active(True)

# Initialize ESP-NOW
esp = espnow.ESPNow()
esp.active(True)

#add peer (ESPB MAC ADDRESS)
peer = b'x!\x3c\x8a\x1f\xa0\xdf\xc0'
esp.add_peer(peer)

#rotary encoder object initialization
r = RotaryIRQ(pin_num_clk=22, pin_num_dt=15, min_val=0, max_val=100, reverse=False)
r.set(50)  #set initial value

while True:
    val = r.value() #set variable equal to current encoder value
    packed = struct.pack('H', val)  #pack into a struct
    print("Sending:", val)
    esp.send(peer, packed)
    time.sleep(0.1)
