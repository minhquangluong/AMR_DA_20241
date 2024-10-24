import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
from std_msgs.msg import Float64MultiArray
# from math import cos, sin, pi
import numpy as np

class Robot_handle(Node):
    def __init__(self):
        super().__init__('handle')
        self.create_subscription(Twist, 'cmd_vel', self.vel_callback,10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, 'wheel_velocity', 10)
        # param
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('wheel_radius', 0.08)
        self.declare_parameter('wheel_base', 0.3)


        serial_port = self.get_parameter('serial_port').value
        baud_rate= self.get_parameter('baud_rate').value
        # pub_hz= self.get_parameter('publish_frequency').value
        self.wheel_r = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        # serial
        self.ser_= serial.Serial(serial_port,baud_rate,timeout=1)
        # matrix kinematic
        self.speed_matrix= np.array([[self.wheel_r/2, self.wheel_r/2],[self.wheel_r/self.wheel_base, -self.wheel_r/self.wheel_base]])
        
    def vel_callback(self,msg):
        x_vel = msg.linear.x
        theta_vel= msg.angular.z
        vector_v = np.array([[x_vel],[theta_vel]])

        #calculate vel
        wheel_speed = np.matmul(np.linalg.inv(self.speed_matrix),vector_v)

        left_wheel_speed = wheel_speed[0][0]
        right_wheel_speed = wheel_speed[1][0]

     
        msgs= Float64MultiArray()
        msgs.data = [left_wheel_speed, right_wheel_speed]
        self.velocity_pub.publish(msgs)

        command = f"{left_wheel_speed:.2f},{right_wheel_speed:.2f}\n"
        self.send_to_arduino(command)
        self.get_logger().info(f"Sent to Arduino: {command.strip()}")
    
    def send_to_arduino(self, command):
        try:
            if self.ser_.is_open:
              self.ser_.write(command.encode())  
            else:
              self.get_logger().error("Serial port is not open.")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial communication error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Robot_handle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        

        








