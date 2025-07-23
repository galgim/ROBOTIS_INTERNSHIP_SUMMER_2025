# esp_uart.py
# This script reads data from a UART port, processes it into ADC values, maps them to angles,
# and sends the angles to a motor control function.

import serial # import serial library for UART communication
from arm_movement import portInitialization, simMotorRun, portTermination # import functions from arm_movement.py

BYTES_PER_MOTOR = 3 # number of bytes per motor (3 bytes for ADC value)
NUM_MOTORS = 6 # number of motors
TOTAL_BYTES = BYTES_PER_MOTOR * NUM_MOTORS # total bytes to read

def bytes_to_adc(triplet: bytes) -> int:
    return int.from_bytes(triplet, 'big') # convert 3 byte chunk to integer ADC value

def map_adc_to_angles(adc: int, min_adc=0, max_adc=4095, min_angle=0, max_angle=180) -> float: # ***max/mins STC***
    return (adc - min_adc) * (max_angle - min_angle)/(max_adc - min_adc) + min_angle # map ADC value to angle

motor_ids = [1, 2, 3, 4, 5, 6] # motor setup stuff
portInitialization('/dev/ttyUSB0', motor_ids) # initialize port for motor control

try: # main loop to read from UART and process data
    with serial.Serial('/dev/ttyTHS1', baudrate=115200, timeout=1) as uart: # open UART port
        while True:
            if uart.in_waiting >= TOTAL_BYTES: # check if enough data is available
                data = uart.read(TOTAL_BYTES) # read the expected number of bytes
                if len(data) == TOTAL_BYTES: # ensure we read the correct amount
                    # Process the data into ADC values and angles
                    adc_values = [bytes_to_adc(data[i:i+BYTES_PER_MOTOR]) for i in range(0, TOTAL_BYTES, BYTES_PER_MOTOR)]
                    angles = [map_adc_to_angles(adc) for adc in adc_values] # map ADC values to angles

                    print("ADC: ", adc_values) # print ADC values
                    print("Angles: ", angles) # print angle values

                    simMotorRun(angles, motor_ids) # send angles to simMotorRun function
                else:
                    print(f"Warning: Expected {TOTAL_BYTES} bytes, got {len(data)}") # handle unexpected byte count

except KeyboardInterrupt: # handle Ctrl+C input
    print("PERISH!!!") # EXPLODES the computer (jk it just stops the program)
except Exception as e: # handle other exceptions
    print(f"Error: {e}") # print error message

finally: # clean up resources
    portTermination() # terminate port connection
