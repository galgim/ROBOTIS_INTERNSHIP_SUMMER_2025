from dynamixel_sdk import * # Dynamixel SDK for communication
import serial # to communicate with ESP32 via UART
import time

# DYNAMIXEL SETUP
DEVICENAME = '/dev/ttyUSB0' # USB port for U2D2
BAUDRATE_DXL = 57600 # baudrate for talking to Dynamixel motors
PROTOCOL_VERSION = 2.0 # Dynamixel Protocol 2.0

# UART SETUP (Jetson <-> ESP32)
UART_PORT = '/dev/ttyTHS1' # Jetson UART (pins 8/10 on GPIO header)
UART_BAUD = 115200 # baudrate for ESP32 UART config

# DYNAMIXEL ADDRESSES
ADDR_TORQUE_ENABLE = 64 # address to enable/disable torque
ADDR_GOAL_POSITION = 116 # address to write a target position to
TORQUE_ENABLE = 1 # value to enable torque (1 = on)

# MOTOR CONFIGURATION
motor_ids = [1, 2, 3, 4, 5, 6, 7, 8] # 8(? (unsure of exact number. consult mech e.)) Dynamixel motor ID's
motor_ranges =
{
    1: (0, 4095), # arm base yaw
    2: (0, 4095), # shoulder
    3: (0, 4095), # elbow
    4: (0, 4095), # wrist
    5: (0, 4095), # grabber rotate
    6: (0, 4095), # grabber open/close
    7: (0, 4095), # placeholder
    8: (0, 4095), # placeholder
} # ***range of motion is a placeholder***. replace with actual range of motion for each given joint

# INITIALIZE DYNAMIXEL COMMUNICATION
portHandler = PortHandler(DEVICENAME) # create port handler for USB device
packetHandler = PacketHandler(PROTOCOL_VERSION) # create packet handler for specified protocol

# open the port and set baudrate
if not portHandler.openPort():
    print("ERROR: Failed to open the U2D2 port")
    exit()
if not portHandler.setBaudRate(BAUDRATE_DXL):
    print("ERROR: Failed to set Dynamixel baudrate")
    exit()

# enable torque for each motor
for dxl_id in motor_ids:
    # send 1 byte command to enable torque (turn on motor command)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    # check for comm errors (ex usb)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Communication error on motor {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
    # check for motor errors (ex out of range)
    elif dxl_error != 0:
        print(f"Dynamixel error on motor {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")

# INITIALIZE UART TO RECEIVE ESP32 DATA
uart = serial.Serial(UART_PORT, baudrate=UART_BAUD, timeout=1)

# HELPER FUNCTION
def adc_to_dxl_position(adc):
    # convert ADC value (0–4095) to a position within this motor's range of motion.
    # each motor maps the full ADC range proportionally into its own range
    min_pos, max_pos = motor_ranges[motor_id]
    ratio = adc_val / 4095.0
    pos = int(min_pos + ratio * (max_pos - min_pos))
    return max(min_pos, min(max_pos, pos)) 

# MAIN LOOP
while True:
    if uart.in_waiting >= 16:
        # read 16 bytes (8 values, each 2 bytes) ***(change if not 8 motors)***
        data = uart.read(16)

        for i in range(8):
            # extract 2 bytes per joint and convert to int
            adc_val = int.from_bytes(data[i*2:i*2+2], 'big')

            # convert ADC to motor position
            dxl_pos = adc_to_dxl_position(adc_val)

            # send position to appropriate Dynamixel motor
            dxl_id = motor_ids[i]
            packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, dxl_pos)

    # delay to prevent bus getting spammed
    time.sleep(0.02)
