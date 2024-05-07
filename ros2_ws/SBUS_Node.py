import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, String
import serial

class SBUSNode(Node):
    def __init__(self):
        super().__init__('sbus_node')
        self.serial = serial.Serial(
            port='/dev/ttyS4',
            baudrate=100000,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS,
            timeout=1,
        )
        self.activation_pub = self.create_publisher(UInt16MultiArray, '/autopilot/activation', 10)
        self.deactivation_pub = self.create_publisher(UInt16MultiArray, '/autopilot/deactivation', 10)
        self.pin_pub = self.create_publisher(String, '/pin_control', 10)

        self.initial_channels = [0] * 16
        self.autopilot_enabled = False

        self.timer = self.create_timer(0.1, self.read_and_process_sbus)

    def decode_sbus_packet(self, sbus_packet):
        channels = [0] * 16
        for i in range(16):
            byte_index = 1 + (i * 11 // 8)
            bit_index = (i * 11) % 8
            channels[i] = (sbus_packet[byte_index] | (sbus_packet[byte_index + 1] << 8) | (sbus_packet[byte_index + 2] << 16)) >> bit_index & 0x07FF
        return channels

    def read_and_process_sbus(self):
        sbus_packet = self.serial.read(25)
        if len(sbus_packet) == 25:
            channels = self.decode_sbus_packet(sbus_packet)
            self.process_channels(channels)

    def process_channels(self, channels):
        channel_8 = channels[7]
        if channel_8 > 2000 and not self.autopilot_enabled:
            self.initial_channels = channels.copy()
            self.initial_channels[12] = 2000
            self.pin_pub.publish(String(data="enable"))
            msg = UInt16MultiArray()
            msg.data = self.initial_channels[4:]
            self.activation_pub.publish(msg)
            self.autopilot_enabled = True
        elif self.autopilot_enabled:
            if channel_8 < 2000 or any(abs(channels[i] - self.initial_channels[i]) > 5 for i in range(4)):
                self.pin_pub.publish(String(data="disable"))
                msg = UInt16MultiArray()
                msg.data = [0] * 12
                self.deactivation_pub.publish(msg)
                self.autopilot_enabled = False

def main(args=None):
    rclpy.init(args=args)
    sbus_node = SBUSNode()
    rclpy.spin(sbus_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()