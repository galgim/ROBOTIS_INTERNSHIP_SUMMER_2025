import serial # to communicate with ESP32 via UART
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import struct # for struct packing/unpacking

# ESP32 UART communication setup
class ESPUART(Node):
    def __init__(self):
        super().__init__('esp_uart')

        # initialize UART serial connection
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1) # /dev/ttyUSB0 is the UART port for ESP32
        self.publisher_ = self.create_publisher(Int32MultiArray, 'esp_values', 10) # topic to publish ADC values
        self.timer = self.create_timer(0.05, self.espRead)  # 20 Hz read rate

    # function to unpack bytes into list of integers
    def structUnpack(self, fmt, data):
        try:
            return list(struct.unpack(fmt, data))  # unpack data into ints
        except struct.error:
            self.get_logger().warn("Incomplete or malformed data received") # handle unpacking error
            return []

    # main loop (reads fixed number of bytes per message)
    def espRead(self):
        BYTES_PER_INT = 4 # 4 bytes per motor
        NUM_VALUES = 8 # number of motors
        TOTAL_BYTES = BYTES_PER_INT * NUM_VALUES # total bytes to read

        if self.ser.in_waiting > 0: # check if any data has been sent
            tag = self.ser.readline().decode('utf-8').strip()
            if tag == "movementArray":
                data = self.ser.read(TOTAL_BYTES) # read the bytes
                values = self.structUnpack('8i', data)
            if values:  # Only publish if unpack was successful
                msg = Int32MultiArray() # create a message to publish
                msg.data = values # assign unpacked values to message
                self.publisher_.publish(msg) # publish the message
                self.get_logger().info(f"ADC Values: {values}") # log the values

def main(args=None):
    rclpy.init(args=args) # initialize ROS2
    node = ESPUART() # create an instance of the ESPUART node
    rclpy.spin(node) # keep the node running
    node.destroy_node() # clean up node (explodes the computer)
    rclpy.shutdown() # shutdown ROS2

if __name__ == '__main__':
    main()
