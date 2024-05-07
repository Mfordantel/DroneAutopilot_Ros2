import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import serial

class SBUSNode(Node):
    def __init__(self):
        super().__init__('sbus_node')
        self.serial = serial.Serial(
			port='/dev/ttyS4',
			baudrate = 100000,
			parity=serial.PARITY_EVEN,
			stopbits=serial.STOPBITS_TWO,
			bytesize=serial.EIGHTBITS,
			timeout = 1,
		)
        
        self.activation_pub = self.create_publisher(UInt16MultiArray, '/autopilot/activation', 10)
        self.deactivation_pub = self.create_publisher(UInt16MultiArray, '/autopilot/deactivation', 10)

    def decode_sbus_packet(self, sbus_packet):
        channels = [0] * 16
        for i in range(16):
            byte_index = 1 + (i * 11 // 8)
            bit_index = (i * 11) % 8
            channels[i] = (sbus_packet[byte_index] | (sbus_packet[byte_index + 1] << 8) | (sbus_packet[byte_index + 2] << 16)) >> bit_index & 0x07FF
        return channels

    def read_and_process_sbus(self):
        while True:
            sbus_packet = self.serial.read(25)
            if len(sbus_packet) == 25:
                channels = self.decode_sbus_packet(sbus_packet)
                self.process_channels(channels)

    def process_channels(self, channels):
        channel_8 = channels[7]
        if channel_8 > 2000:
            msg = UInt16MultiArray()
            msg.data = channels[4:16]
            self.activation_pub.publish(msg)
        elif channel_8 < 2000:
            msg = UInt16MultiArray()
            msg.data = [0] * 12
            self.deactivation_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    sbus_node = SBUSNode()
    sbus_node.read_and_process_sbus()
    rclpy.spin(sbus_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()