import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from math import cos, sin, pi

class Dynamic_TF_node(Node):
    def __init__(self):
        super().__init__("dynamic_tf_node")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Float64MultiArray, "hall_data", self.hall_callback, 10)
        self.current_position = [0.0, 0.0, 0.0]  # (x, y, theta)

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
        self.current_position[0] += distance_robot * cos(self.current_position[2])
        self.current_position[1] += distance_robot * sin(self.current_position[2])
        self.current_position[2] += angle_robot  # Update angle

        self.publish_dynamic_tf()

    def publish_dynamic_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'

        tf.transform.translation.x = self.current_position[0]
        tf.transform.translation.y = self.current_position[1]
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = sin(self.current_position[2] / 2)
        tf.transform.rotation.w = cos(self.current_position[2] / 2)

        self.tf_broadcaster.sendTransform(tf)

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
