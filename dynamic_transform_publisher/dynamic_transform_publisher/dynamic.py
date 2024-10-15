import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from math import cos, sin, pi, atan2

class Dynamic_TF_node(Node):
    def __init__(self):
        super().__init__("dynamic_tf_node")
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Float64MultiArray, "hall_data", self.hall_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Param
        self.declare_parameter('CPR', 90)
        self.declare_parameter('wheel_D', 0.18)  # (m)
        self.declare_parameter('wheel_base', 0.3)

        self.cpr = self.get_parameter('CPR').value
        self.wheel_d = self.get_parameter('wheel_D').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_circum = pi * self.wheel_d

    def hall_callback(self, msg):
        pulses_l = msg.data[0]
        pulses_r = msg.data[1]

        # Calculate distances L_R
        distance_l = (pulses_l / self.cpr) * self.wheel_circum
        distance_r = (pulses_r / self.cpr) * self.wheel_circum

        # Calculate movement distances
        distance_robot = (distance_l + distance_r) / 2
        angle_robot = (distance_l - distance_r) / self.wheel_base

        # Update position
        self.x += distance_robot * cos(self.theta)
        self.y += distance_robot * sin(self.theta)
        self.theta = self.normalize_angle(self.theta + angle_robot)
        
        # Publish odom
        odom_msg = Odometry()

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom_msg) 

        # Publish TF
        self.publish_dynamic_tf()

    def publish_dynamic_tf(self):
        q = self.euler_to_quaternion(0, 0, self.theta)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(tf)

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
    dynamic_tf_node = Dynamic_TF_node()

    try:
        rclpy.spin(dynamic_tf_node)
    finally:
        dynamic_tf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
