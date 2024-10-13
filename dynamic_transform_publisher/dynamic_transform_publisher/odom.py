import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
import math

class OdometryCalculator(Node):
    def __init__(self):
        super().__init__('odometry_calculator')
        self.create_subscription(Float64MultiArray, "hall_data", self.hall_callback, 10)

        # Parameters
        self.declare_parameter('CPR', 90)  # Counts per revolution
        self.declare_parameter('wheel_radius', 0.08)  # in meters
        self.declare_parameter('wheel_separation', 0.3)  # in meters

        self.CPR = self.get_parameter('CPR').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pos = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.transform_broadcaster = TransformBroadcaster(self)

    def hall_callback(self, msg):
       
        pulses_l = msg.data[0]
        pulses_r = msg.data[1]

        # Tính toán khoảng cách dựa trên số xung từ encoder
        d_l = (pulses_l / self.CPR) * (2 * math.pi * self.wheel_radius)  # khoảng cách của bánh trái
        d_r = ((pulses_r-10) / self.CPR) * (2 * math.pi * self.wheel_radius)  # khoảng cách của bánh phải

        # Tính toán vị trí mới
        d = (d_l + d_r) / 2.0  # Khoảng cách trung bình
        delta_theta = (d_r - d_l) / self.wheel_separation  # Thay đổi góc

        # Cập nhật vị trí và góc
        self.theta += delta_theta
        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)

        # Tạo tin nhắn odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Tạo quaternion từ góc Euler
        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Xuất bản odometry
        self.odom_pub.publish(odom_msg)

        # Xuất bản chuyển đổi
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        self.transform_broadcaster.sendTransform(transform)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    odometry_calculator = OdometryCalculator()
    rclpy.spin(odometry_calculator)
    odometry_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
