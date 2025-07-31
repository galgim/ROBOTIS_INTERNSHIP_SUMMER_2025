from dynamixel_sdk import * # Dynamixel SDK for communication
import serial # to communicate with ESP32 via UART
import time

#**********************************TEMP FOR TESTING***********************************
ADDR_PRESENT_POSITION = 132 #Address of the positions of the motors
ADDR_PROFILE_VELOCITY = 112 #Address of the velocity of the motors
ADDR_GOAL_POSITION = 116 #Address of goal position
ADDR_MOVING = 122 #Address of value that states if motor is moving or not
ADDR_MOVING_STATUS = 123
DXL_MOVING_STATUS_THRESHOLD = 10    # Dynamixel moving status threshold
LEN_GOAL_POSITION = 4 #Byte Length of goal position
LEN_PRESENT_POSITION = 4 #Byte length of present positiond
ADDR_TORQUE_ENABLE = 64
TORQUE_ENABLE = 1

# DYNAMIXEL SETUP
DEVICENAME = '/dev/ttyUSB0' # USB port for U2D2
BAUDRATE_DXL = 1000000 # baudrate for talking to Dynamixel motors
PROTOCOL_VERSION = 2.0 # Dynamixel Protocol 2.0

# UART SETUP (Jetson <-> ESP32)
UART_PORT = '/dev/ttyTHS1' # UART port for ESP32
UART_BAUD = 115200 # baudrate for ESP32 UART config

# MOTOR CONFIGURATION
motor_ids = [0, 1, 2, 3, 4, 5] # 6 Dynamixel motor ID's
motor_ranges = {
    0: (0, 4095), # arm base yaw
    1: (0, 4095), # shoulder
    2: (0, 4095), # elbow
    3: (0, 4095), # wrist pitch
    4: (0, 4095), # wrist roll
    5: (0, 4095), # grabber open/close
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

def WriteMotorData(motorID, data_address, data_inputs):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
    portHandler, motorID, data_address, data_inputs)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

def dxlSetVelo(vel_array, dxlIDs):
    if (len(vel_array) == len(dxlIDs)):
        idNum = len(dxlIDs)
        for id in range(idNum):
                    WriteMotorData(dxlIDs[id], ADDR_PROFILE_VELOCITY, vel_array[id])
    else:
        print("ERROR: Number of velocity inputs not matching with number of DXL ID inputs!")
    print("-------------------------------------")

def portTermination():
    # terminate port connection (EXPLODES the computer)
    global portHandler
    if portHandler:
        portHandler.closePort()

#**********************************TEMP FOR TESTING***********************************
if __name__ == "__main__":
    portInitialization(DEVICENAME, motor_ids)
    test_angles = [0, 0, 0, 0, 0, 0]  # Change these as needed for your test
    dxlSetVelo([1, 1, 1, 1, 1, 1], motor_ids) # Set velocity for testing
    simMotorRun(test_angles, motor_ids)
    time.sleep(2)
    portTermination()