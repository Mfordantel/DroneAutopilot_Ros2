import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, UInt16MultiArray
import math

class AutopilotNode(Node):
    def __init__(self):
        super().__init__('autopilot_node')
        
        self.altitude_sub = self.create_subscription(Float64, 'altitude', self.altitude_callback, 10)
        self.attitude_sub = self.create_subscription(Float64MultiArray, 'attitude', self.attitude_callback, 10)
        self.activation_sub = self.create_subscription(UInt16MultiArray, '/autopilot/activation', self.activation_callback, 10)
        self.deactivation_sub = self.create_subscription(UInt16MultiArray, '/autopilot/deactivation', self.deactivation_callback, 10)
        
        self.sbus_pub = self.create_publisher(UInt16MultiArray, 'sbus_output', 10)
        
        self.altitude = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.channels = [0] * 16
        self.autopilot_enabled = False
        self.target_altitude = None
        
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        self.prev_yaw = 0.0
        self.prev_time = self.get_clock().now()
        
        self.SBUS_min = 173
        self.SBUS_mid = 992
        self.SBUS_max = 2047
        self.SBUS_range = min(self.SBUS_max - self.SBUS_mid, self.SBUS_mid - self.SBUS_min)
        
        self.pid_altitude = PID(Kp=0.5, Ki=0.1, Kd=0.2)
        self.pid_roll = PID(Kp=0.8, Ki=0.05, Kd=0.3)
        self.pid_pitch = PID(Kp=0.8, Ki=0.05, Kd=0.3)
        self.pid_yaw = PID(Kp=0.5, Ki=0.1, Kd=0.2)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def altitude_callback(self, msg):
        self.altitude = msg.data
        
    def attitude_callback(self, msg):
        self.roll = msg.data[0]
        self.pitch = msg.data[1]
        self.yaw = msg.data[2]
        
    def activation_callback(self, msg):
        self.autopilot_enabled = True
        self.channels[4:] = msg.data
        self.target_altitude = self.altitude
        
    def deactivation_callback(self, msg):
        self.autopilot_enabled = False
        self.target_altitude = None
        
    def control_loop(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time
        
        if self.autopilot_enabled:
            if self.target_altitude is not None:
                error_h = self.target_altitude - self.altitude
                throttle = self.pid_altitude.update(error_h, dt)
                throttle = min(max(throttle, 0.585), 0.605)
            
                roll_rate = (self.roll - self.prev_roll) / dt
                pitch_rate = (self.pitch - self.prev_pitch) / dt
                yaw_rate = (self.yaw - self.prev_yaw) / dt
            
                roll_output = self.pid_roll.update(self.roll, dt)
                roll_output += self.pid_roll.update_derivative(roll_rate, dt)
            
                pitch_output = self.pid_pitch.update(self.pitch, dt)
                pitch_output += self.pid_pitch.update_derivative(pitch_rate, dt)
            
                yaw_output = self.pid_yaw.update(self.yaw, dt)
                yaw_output += self.pid_yaw.update_derivative(yaw_rate, dt)
            
                roll = self.SBUS_mid - roll_output / math.pi * 180 / 360 * self.SBUS_range
                pitch = self.SBUS_mid - pitch_output / math.pi * 180 / 360 * self.SBUS_range
                yaw = self.SBUS_mid - yaw_output / math.pi * 180 / 360 * self.SBUS_range
            
                self.channels[:4] = [int(roll), int(pitch), int(throttle * 1700 + self.SBUS_min), int(yaw)]
            
                self.prev_roll = self.roll
                self.prev_pitch = self.pitch
                self.prev_yaw = self.yaw
        
        sbus_packet = self.encode_sbus_packet(self.channels)
        sbus_msg = UInt16MultiArray(data=sbus_packet)
        self.sbus_pub.publish(sbus_msg)
        
    def encode_sbus_packet(self, channels):
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
        self.previous_derivative = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        self.previous_derivative = derivative
        return output

    def update_derivative(self, derivative, dt):
        output = self.Kd * (derivative - self.previous_derivative) / dt
        self.previous_derivative = derivative
        return output

def main(args=None):
    rclpy.init(args=args)
    autopilot_node = AutopilotNode()
    rclpy.spin(autopilot_node)
    autopilot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()