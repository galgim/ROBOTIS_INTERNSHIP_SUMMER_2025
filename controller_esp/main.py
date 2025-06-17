import ujson #JSON configuration
from machine import SPI, Pin, ADC #SPI, pin, and ADC reading
import time #sleep method

spi = SPI(1, baudrate=40000) #using spi0 at 400kHz

#pin 13 MOSI
#pin 12 MISO
spi_chipSelect = Pin(4, mode=Pin.OUT, value=1); #arbitrary pin 4

arm_motor_pins = { # usable ADC pins 0, 2, 4, 12, 13, 14, 15, 25, 26, 27, 32, 33, 34, 35, 36, and 39.
                    "forward_backward" : ADC(Pin(35)),  
                    "left_right" : ADC(Pin(36)),
                    "arm_yaw" : ADC(Pin(0)),
                    "arm_segment1": ADC(Pin(2)),
                    "arm_segment2": ADC(Pin(4)),
                    "arm_segment3": ADC(Pin(12)),
                    "grabber_roll": ADC(Pin(13)),
                    "grabber_grab": ADC(Pin(14)),
                    }

#for pin in arm_motor_pins: # sets all ADC pins 
#   pin.atten(ADC.ATTN_11DB) # sets ADC pin read from 0V-3.3V
#    pin.width(ADC.WIDTH_10BIT) # sets analog reading to be 10 bits long

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
vectorArray = [0,0,0,0,0,0,0,0]
while True:
    count = 0
    for control in movement_vectors.keys(): #for all movement vector names, read their voltage level and update the dictionary
        adcReading = arm_motor_pins[control].read()
        movement_vectors[control] = adcReading
        vectorArray[count] = adcReading
        count = count + 1
    time.sleep(2)
    print("---------------------\n")
    print(vectorArray)
