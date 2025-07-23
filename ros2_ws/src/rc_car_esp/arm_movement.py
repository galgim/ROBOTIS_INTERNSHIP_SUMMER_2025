from dynamixel_sdk import * # Dynamixel SDK for communication
import serial # to communicate with ESP32 via UART
import time


# DYNAMIXEL SETUP
DEVICENAME = '/dev/ttyUSB0' # USB port for U2D2
BAUDRATE_DXL = 57600 # baudrate for talking to Dynamixel motors
PROTOCOL_VERSION = 2.0 # Dynamixel Protocol 2.0

# UART SETUP (Jetson <-> ESP32)
UART_PORT = '/dev/ttyTHS1' # UART port for ESP32
UART_BAUD = 115200 # baudrate for ESP32 UART config

# MOTOR CONFIGURATION
motor_ids = [1, 2, 3, 4, 5, 6] # 6 Dynamixel motor ID's
motor_ranges =
{
    1: (0, 4095), # arm base yaw
    2: (0, 4095), # shoulder
    3: (0, 4095), # elbow
    4: (0, 4095), # wrist
    5: (0, 4095), # grabber rotate
    6: (0, 4095), # grabber open/close
} # ***range of motion is a placeholder***. replace with actual range of motion for each given joint

# Global handles
portHandler = None # Dynamixel port handler
packetHandler = None # Dynamixel packet handler
 
def portInitialization(device_name, ids): # Initialize the port for Dynamixel motors
    global portHandler, packetHandler
    portHandler = PortHandler(device_name)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    if not portHandler.openPort():
        # exit if port cannot be opened
        print("ERROR: Failed to open the U2D2 port")
        exit()
    if not portHandler.setBaudRate(BAUDRATE_DXL):
        # exit if baudrate cannot be set
        print("ERROR: Failed to set Dynamixel baudrate")
        exit()
    for dxl_id in ids:
        # to enable torque for each motor
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            # handle communication error
            print(f"Communication error on motor {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            # handle packet error
            print(f"Dynamixel error on motor {dxl_id}: {packetHandler.getRxPacketError(dxl_error)}")

def adc_to_dxl_position(adc, motor_id):
    # convert ADC value to Dynamixel position based on motor ID
    min_pos, max_pos = motor_ranges[motor_id] # get the range for the specific motor
    ratio = adc / 4095.0 # scale ADC value to 0-1 range
    pos = int(min_pos + ratio * (max_pos - min_pos)) # scale ADC to motor range
    return max(min_pos, min(max_pos, pos)) # ensure position is within motor limits

def simMotorRun(angles, ids):
    # angles: list of angles (0-180), ids: list of motor IDs
    for angle, dxl_id in zip(angles, ids):
        # Convert angle (0-180) to ADC (0-4095)
        adc_val = int((angle / 180.0) * 4095) # scale angle to ADC value
        dxl_pos = adc_to_dxl_position(adc_val, dxl_id) # convert ADC to Dynamixel position
        packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, dxl_pos) # send pos command to motor

def portTermination():
    # terminate port connection (EXPLODES the computer)
    global portHandler
    if portHandler:
        portHandler.closePort()
