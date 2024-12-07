import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import serial
import numpy as np

class RobotHandle(Node):
    def __init__(self):
        super().__init__('robot_handle')
        
        # ROS 2 subscriptions and publishers
        self.create_subscription(Twist, 'cmd_vel', self.vel_callback, 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, 'wheel_velocity', 10)
        
        # Parameters
        self.declare_parameter('serial_port_left', '/dev/ttyACM0')
        self.declare_parameter('serial_port_right', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius', 0.08)
        self.declare_parameter('wheel_base', 0.3)

        # Retrieve parameters
        serial_port_left = self.get_parameter('serial_port_left').value
        serial_port_right = self.get_parameter('serial_port_right').value
        baud_rate = self.get_parameter('baud_rate').value
        self.wheel_r = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # Serial setup
        self.ser_left = serial.Serial(serial_port_left, baud_rate, timeout=1)
        self.ser_right = serial.Serial(serial_port_right, baud_rate, timeout=1)

        # Kinematic matrix for calculating wheel velocities
        self.speed_matrix = np.array([[self.wheel_r / 2, self.wheel_r / 2], 
                                      [self.wheel_r / self.wheel_base, -self.wheel_r / self.wheel_base]])

        # Timer for reading Hall sensor feedback
        self.create_timer(0.01, self.read_hall_from_arduino)

    def vel_callback(self, msg):
        # Receive target linear and angular velocities
        x_vel = msg.linear.x
        theta_vel = msg.angular.z
        vector_v = np.array([[x_vel], [theta_vel]])

        # Calculate target wheel velocities
        wheel_speeds = np.matmul(np.linalg.inv(self.speed_matrix), vector_v)
        target_speed_left = wheel_speeds[0][0]
        target_speed_right = wheel_speeds[1][0]

        # Send calculated velocities to Arduinos
        self.send_to_arduino(target_speed_left, target_speed_right)

    def read_hall_from_arduino(self):
        try:
            if self.ser_left.is_open and self.ser_right.is_open:
                # Read current velocities from each Arduino
                vl_current = self.ser_left.readline().decode('utf-8', errors='ignore').strip()
                vr_current = self.ser_right.readline().decode('utf-8', errors='ignore').strip()

                # Convert the received data to float values if possible
                try:
                    vl_current = float(vl_current)
                    vr_current = float(vr_current)
                except ValueError:
                    vl_current, vr_current = 0.0, 0.0

                # Publish the actual wheel velocities
                msgs = Float64MultiArray()
                msgs.data = [vl_current, vr_current]
                self.velocity_pub.publish(msgs)

                self.get_logger().info(f"Received from Arduino: vl={vl_current} m/s, vr={vr_current} m/s")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

    def send_to_arduino(self, speed_left, speed_right):
        try:
            if self.ser_left.is_open and self.ser_right.is_open:
                command_left = f"{int(speed_left)}\n"
                command_right = f"{int(speed_right)}\n"
                
                # Send data to left and right Arduinos
                self.ser_left.write(command_left.encode())
                self.ser_right.write(command_right.encode())

                self.get_logger().info(f"Sent to Arduino: Left={command_left.strip()}, Right={command_right.strip()}")
            else:
                self.get_logger().error("One or both serial ports are not open.")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotHandle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
