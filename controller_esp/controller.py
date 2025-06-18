import espnow #ESP-NOW library
import network #ESP network library
from machine import SPI, Pin, ADC #SPI, pin, and ADC reading
import time #sleep method

arm_motor_pins = { # usable ADC pins 0, 2, 4, 12, 13, 14, 15, 25, 26, 27, 32, 33, 34, 35, 36, and 39.
                    "forward_backward" : ADC(Pin(32)),  
                    "left_right" : ADC(Pin(33)),
                    "arm_yaw" : ADC(Pin(34)),
                    "arm_segment1": ADC(Pin(35)),
                    "arm_segment2": ADC(Pin(36)),
                    "arm_segment3": ADC(Pin(37)),
                    "grabber_roll": ADC(Pin(38)),
                    "grabber_grab": ADC(Pin(39)),
                    }

movement_vectors = {
                    "forward_backward" : 0.0, # RC car forward & backward movement neg# = back, pos# = for
                    "left_right" : 0, #RC car front wheels left/right turn, neg# = left, pos#=right
                    "arm_yaw" : 0, #arm base horizontal movement, 0-270 (ARBITRARY)
                    "arm_segment1" : 0, #arm segment movement 0-180 (ARBITUARY)
                    "arm_segment2": 0, #arm segment movement 0-180 (ARBITRARY)
                    "arm_segment3":0,  #arm segment movement 0-180 (ARBITRARY)
                    "grabber_roll":0,  #grabber wrist movement 0-180 (ARBITRARY) 0=left movement, 180=right movement
                    "grabber_grab":0,  #grabber pinch movement 0-90 (ARBITRARY) 0=open, 90=closed 
                    }
vector_array = [0,0,0,0,0,0,0,0]

# A WLAN interface must be active to send()/recv()
station = network.WLAN(network.STA_IF)  # Or network.AP_IF
station.active(True)

# Initialize ESP-NOW
esp = espnow.ESPNow()
esp.active(True)

# Define the MAC address of the receiving ESP32 (ESP32 B)
peer = b'x!\x84\xc68\xb0'
esp.add_peer(peer)

def read_analog_inputs():
    count = 0
    for control in movement_vectors.keys(): #for all movement vector names, read their voltage level and update the dictionary
        adc_reading = arm_motor_pins[control].read()
        movement_vectors[control] = adc_reading
        vector_array[count] = adc_reading.to_bytes(10,big)
        count = count + 1
            
while True:
    read_analog_inputs()
    print("---------------------\n")
    print(vector_array)
    esp.send(peer, vector_array)
    time.sleep(2)
