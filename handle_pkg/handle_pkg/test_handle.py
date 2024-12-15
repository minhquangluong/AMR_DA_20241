import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import serial
import numpy as np

class RobotHandle(Node):
    def __init__(self):
        super().__init__('robot_handle')
        
        
        self.create_subscription(Twist, 'cmd_vel', self.vel_callback, 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, 'wheel_velocity', 10)
        
        # Parameters
       
     
        self.declare_parameter('wheel_radius', 0.08)
        self.declare_parameter('wheel_base', 0.3)

        # Retrieve parameters
       
        self.wheel_r = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

       
        
        # Kinematic matrix for calculating wheel velocities
        self.speed_matrix = np.array([[self.wheel_r / 2, self.wheel_r / 2], 
                                      [self.wheel_r / self.wheel_base, -self.wheel_r / self.wheel_base]])
        print(self.speed_matrix)

        # Timer for reading Hall sensor feedback
        

    def vel_callback(self, msg):
        # Receive target linear and angular velocities
        x_vel = msg.linear.x
        theta_vel = msg.angular.z
        vector_v = np.array([[x_vel], [theta_vel]])

        # Calculate target wheel velocities
        wheel_speeds = np.matmul(np.linalg.inv(self.speed_matrix), vector_v)
        target_speed_left = wheel_speeds[0][0]
        target_speed_right = wheel_speeds[1][0]
        self.pub_vel(target_speed_left,target_speed_right)

       
    def pub_vel(self, a, b):
        msg= Float64MultiArray()
        msg.data= [a, b]
        self.velocity_pub.publish(msg)

   

    

def main(args=None):
    rclpy.init(args=args)
    node = RobotHandle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
