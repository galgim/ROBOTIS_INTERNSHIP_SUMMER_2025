from machine import UART, Pin
from time import sleep
import espnow #ESP-NOW library
import network #access ESP network library

#file imports
import motor_movement #motor_movement.py 
import jetson_communication #jetson_communication.py

jetson_listen_byte_size = 24 # change value based on size sent from Jetson

#hardware UART with 115kHz, tx pin 17, rx pin 16, 1sec timeout 
uart = UART(1, baudrate=115200, tx=17, rx=16, timeout=1000)

# A WLAN interface must be active to send()/recv()
sta = network.WLAN(network.STA_IF)  # Or network.AP_IF
sta.active(True)
sta.disconnect()      # For ESP8266
# Initialize ESP-NOW
esp = espnow.ESPNow()
esp.active(True)


def uart_send(message):
    """Wraps message in start and end characters and sends to Jetson.


    """
    wrapped_message = wrap_message(message) 
    uart.write(wrapped_message)

def strip_message(message):
    """Removes start and end character from message

    Method assumes that the message actually has a start and end message 
    due to the fact that its called only after validate_message is called.
    """
    truncated_message = message[1:] # removes 0 index character
    truncated_message = message[:-1] # removes last index character
    return truncated_message

def validate_esp_message(message):
    """Checks if a message from the controller ESP is the right size.

    Function exists in the case that data begins to be corrupted 
    or longer than expected. Additional booleans can be added in the future.
    """
    bool_message_is_n_bytes = len(message) == jetson_listen_byte_size
    return bool_message_is_n_bytes

#continously poll for data from both controller ESP-NOW and Jetson UART
while True:
    if(esp.any()): # returns 0 if nothing in buffer, positive # otherwise
        _, esp_message = esp.recv() #receive data from controller esp
        if(validate_esp_message(esp_message)):
            move_motors(strip_message(esp_message))


    text_buffer = bytearray(8)
    if uart.any(): # returns 0 if nothing in buffer, positive # otherwise
        raw_message = uart.readinto(text_buffer[8])
        if(validate_jetson_message(raw_message)): 
            uart.write(strip_message(raw_message)) 
    sleep(2)
