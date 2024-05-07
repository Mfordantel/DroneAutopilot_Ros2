import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
from pymavlink import mavutil

class MavlinkNode(Node):
    def __init__(self):
        super().__init__('mavlink_node')
        self.connection = mavutil.mavlink_connection('/dev/ttyS3', baud=115200, dialect='common')

        self.alt_pub = self.create_publisher(Float64, 'altitude', 10)
        self.attitude_pub = self.create_publisher(Float64MultiArray, 'attitude', 10)

    def start_reading(self):
        while True:
            try:
                params = self.connection.recv_match(blocking=True)
                if params:
                    self.process_message(params)
            except Exception as e:
                self.get_logger().error('Failed to receive message: {}'.format(str(e)))

    def process_message(self, params):
        msg_type = params.get_type()
        if msg_type == "VFR_HUD":
            altitude_msg = Float64()
            altitude_msg.data = params.alt
            self.alt_pub.publish(altitude_msg)
            self.get_logger().info(f'Published altitude: {params.alt}')
        elif msg_type == "ATTITUDE":
            attitude_msg = Float64MultiArray()
            attitude_msg.data = [params.roll, params.pitch, params.yaw]
            self.attitude_pub.publish(attitude_msg)
            self.get_logger().info(f'Published attitude: Roll={params.roll}, Pitch={params.pitch}, Yaw={params.yaw}')

def main(args=None):
    rclpy.init(args=args)
    mavlink_node = MavlinkNode()
    mavlink_node.start_reading()
    rclpy.spin(mavlink_node)
    mavlink_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()