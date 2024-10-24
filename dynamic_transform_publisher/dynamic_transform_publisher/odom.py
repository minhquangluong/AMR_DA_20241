import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from math import sin, cos, atan2
import numpy as np

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Float64MultiArray, 'wheel_velocity', self.velocity_callback, 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Parameters
        self.declare_parameter('wheel_base', 0.3)  
        self.declare_parameter('wheel_radius', 0.08)  
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        
        self.last_time = self.get_clock().now()

        # Buffer for smoothing velocities
        self.left_velocity_buffer = []
        self.right_velocity_buffer = []
        self.velocity_buffer_size = 5

    def velocity_callback(self, msg):
     
        v_left = msg.data[0] * self.wheel_radius  
        v_right = msg.data[1] * self.wheel_radius  

        # range vel
        max_linear_speed = 0.5  # m/s
        max_angular_speed = 1.0  # rad/s
        v_left = max(min(v_left, max_linear_speed), -max_linear_speed)
        v_right = max(min(v_right, max_linear_speed), -max_linear_speed)

        # smooth vel
        v_left = self.moving_average_filter(v_left, self.left_velocity_buffer)
        v_right = self.moving_average_filter(v_right, self.right_velocity_buffer)

        
        v = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_base

        # delta time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  
        self.last_time = current_time

        # update_position
        delta_x = v * cos(self.theta) * dt
        delta_y = v * sin(self.theta) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta = self.normalize_angle(self.theta + delta_theta)

       
        self.publish_odom(v, omega)

        #odom -> base_link
        self.publish_tf()

    def moving_average_filter(self, new_value, buffer):
        buffer.append(new_value)
        if len(buffer) > self.velocity_buffer_size:
            buffer.pop(0)
        return sum(buffer) / len(buffer)

    def publish_odom(self, linear_vel, angular_vel):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Pose 
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Twist
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.angular.z = angular_vel

        self.odom_pub.publish(odom_msg)

    def publish_tf(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'

        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.theta)
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(tf_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
        return (qx, qy, qz, qw)

    def normalize_angle(self, angle):
        return atan2(sin(angle), cos(angle))

def main():
    rclpy.init()
    odom_publisher = OdomPublisher()

    try:
        rclpy.spin(odom_publisher)
    finally:
        odom_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
