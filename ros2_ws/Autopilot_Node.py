import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, UInt16MultiArray
import time
import math

class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot_node')

        self.mavlink_sub = self.create_subscription(
            Float64MultiArray, '/mavlink/attitude', self.get_imu, 10)
        self.altitude_sub = self.create_subscription(
            Float64, '/mavlink/altitude', self.get_imu, 10)
        self.sbus_activation_sub = self.create_subscription(
            UInt16MultiArray, '/autopilot/activation', self.process_channels, 10)
        self.sbus_deactivation_sub = self.create_subscription(
            UInt16MultiArray, '/autopilot/deactivation', self.process_channels, 10)

        self.sbus_pub = self.create_publisher(UInt16MultiArray, 'sbus_output', 10)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.altitude = 0.0
        self.channels = [0] * 16
        self.autopilot_enabled = False
        self.target_altitude = None

        self.SBUS_min = 173
        self.SBUS_mid = 992
        self.SBUS_max = 2047
        self.SBUS_range = min(self.SBUS_max - self.SBUS_mid, self.SBUS_mid - self.SBUS_min)
        self.max_angular_velocity = 360

        self.pid_altitude = PID(Kp=1.0, Ki=0.09, Kd=0.2)
        self.pid_roll = PID(Kp=0.1, Ki=0.09, Kd=0.2)
        self.pid_pitch = PID(Kp=0.1, Ki=0.09, Kd=0.2)
        self.pid_yaw = PID(Kp=0.1, Ki=0.09, Kd=0.2)

        self.timer = self.create_timer(0.1, self.fly)

    def get_imu(self, msg):
        if isinstance(msg, Float64MultiArray):
            self.roll = msg.data[0]
            self.pitch = msg.data[1]
            self.yaw = msg.data[2]
        elif isinstance(msg, Float64):
            self.altitude = msg.data

    def process_channels(self, msg):
        if msg.data[0] == 0:
            self.autopilot_enabled = False
            self.target_altitude = None
        else:
            self.autopilot_enabled = True
            self.channels[4:] = msg.data[1:]
            self.target_altitude = self.altitude

    def fly(self):
        if self.autopilot_enabled:
            if self.target_altitude is not None:
                error_h = self.target_altitude - self.altitude
                self.throttle = self.pid_altitude.update(error_h, 0.1)
                self.throttle = min(max(self.throttle, 0.585), 0.605)

                throttle = self.throttle
                roll = self.SBUS_mid - self.pid_roll.update(self.roll, 0.1) / math.pi * 180 / self.max_angular_velocity * self.SBUS_range
                pitch = self.SBUS_mid - self.pid_pitch.update(self.pitch, 0.1) / math.pi * 180 / self.max_angular_velocity * self.SBUS_range
                yaw = self.SBUS_mid - self.pid_yaw.update(self.yaw, 0.1) / math.pi * 180 / self.max_angular_velocity * self.SBUS_range

                self.channels[:4] = [int(roll), int(pitch), int(throttle * 1700 + self.SBUS_min), int(yaw)]

            sbus_packet = SBUS.encode_sbus_packet(self.channels)
            sbus_msg = UInt16MultiArray(data=sbus_packet)
            self.sbus_pub.publish(sbus_msg)

class SBUS:
    @staticmethod
    def encode_sbus_packet(channels):
        sbus_packet = bytearray(25)
        sbus_packet[0] = 0x0F

        for i, value in enumerate(channels):
            byte_index = 1 + (i * 11 // 8)
            bit_index = (i * 11) % 8
            sbus_packet[byte_index] |= int(((value & 0x07FF) << bit_index) & 0xFF)
            sbus_packet[byte_index + 1] |= int(((value & 0x07FF) << bit_index) >> 8 & 0xFF)
            if bit_index >= 6 and i < 15:
                sbus_packet[byte_index + 2] |= int(((value & 0x07FF) << bit_index) >> 16 & 0xFF)

        sbus_packet[24] = 0x00
        return sbus_packet

class PID:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, delta_time):
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

def main(args=None):
    rclpy.init(args=args)
    autopilot_node = AutopilotNode()
    rclpy.spin(autopilot_node)
    autopilot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()