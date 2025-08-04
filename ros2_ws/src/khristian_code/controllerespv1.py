import espnow #ESP-NOW library
import network #ESP network library
from machine import SPI, Pin, ADC #SPI, pin, and ADC reading
import time #sleep method
from rotary_irq_esp import RotaryIRQ #rotary encoder library (already on the esp)
import struct

encoders = []#defined with for loop below

initial_values = [1, 2, 3, 4, 5, 6]#set initial values

encoder_pins = [ # usable ADC pins 0, 2, 4, 12, 13, 14, 15, 25, 26, 27, 32, 33, 34, 35, 36, and 39.
                    #B1,A1 / B2,A2 / B3,A3 / B4,A4 / B5,A5 / B6,A6
                    (22,15),(17,04),(18,05),(26,27),(35,34), (0,22)
                    ]


#define each rotary encoder object, total of 6 clk is B, dt is A
for i, (clk, dt) in encoder_pins:
    r = RotaryIRQ(pin_num_clk=clk,
                  pin_num_dt=dt,
                  min_val=0, #min 0
                  max_val=100, #max 100
                  reverse=False,
                  range_mode=RotaryIRQ.RANGE_BOUNDED,
                  pull_up=false)
    r.set(initial_values[i]) #set initial values
    encoders.append(r)
    
    
#define motor pins
motor_pins = {
    "forward_backward" : ADC(Pin(32)),
    "left_right" : ADC(Pin(33))
    }

#adc voltage range 0-3.3v
for adc in motor_pins:
    adc.atten(ADC.ATTN_11DB)

# A WLAN interface must be active to send()/recv()
station = network.WLAN(network.STA_IF)  # Or network.AP_IF
station.active(True)

# Initialize ESP-NOW
esp = espnow.ESPNow()
esp.active(True)

# Define the MAC address of the receiving ESP32 (ESP32 B) Yo
## YOU'RE GOING TO NEED TO SWAP THIS VALUE WITH THE ESP A MAC ADDRESS IF YOU FLASH THE WRONG ESP
#ESP32 A mac address 3c:8a:1f:a1:61:b8
peer = b'x!\x3c\x8a\x1f\xa0\xdf\xc0' #found via 3c:8a:1f:a0:df:c0 (ESP32 B)
esp.add_peer(peer)


def read_inputs():
    """
    Reads rotary and analog inputs, packs into bytes using struct
    """
    values = []

    # Read 6 encoder values (integers 0–100)
    for r in encoders:
        values.append(r.value())

    # Read 2 analog values (0–4095)
    for pin in motor_pins:
        val = motor_pins[pin].read()  # ← FIXED: was missing ()
        values.append(val)

    # Pack all 8 unsigned 16-bit integers into binary format
    # Format: 6 encoders + 2 analog = 8 values, each as unsigned short = 'H'
    packed = struct.pack('8H', *values)

    return packed


while True:
    output_data = read_inputs()
    
    
    print("---------------------\n")
    print(output_data)
    esp.send(peer, output_data)
    time.sleep(.05)
