import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PolygonStamped, Point32
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from math import sin, cos, atan2
from collections import deque
import numpy as np


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # Publisher and Subscriber
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.footprint_pub = self.create_publisher(PolygonStamped, '/robot_footprint', 10)
        self.create_subscription(Float64MultiArray, '/wheel_velocity', self.velocity_callback, 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Parameters (configurable)
        self.declare_parameter('wheel_base', 0.27)
        self.declare_parameter('wheel_radius', 0.08)
        # self.declare_parameter('robot_length', 0.43)
        # self.declare_parameter('robot_width', 0.48)

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.robot_length = self.get_parameter('robot_length').value
        self.robot_width = self.get_parameter('robot_width').value

        # Timing
        self.last_time = self.get_clock().now()

        # Buffer for smoothing velocities
        self.velocity_buffer_size = 5  # Increased for smoother results
        self.left_velocity_buffer = deque(maxlen=self.velocity_buffer_size)
        self.right_velocity_buffer = deque(maxlen=self.velocity_buffer_size)
        

        self.get_logger().info("OdomPublisher initialized successfully.")

    def velocity_callback(self, msg):
        """
        Callback to process velocity data from /wheel_velocity topic.
        """
        try:
            v_left = msg.data[0] * self.wheel_radius
            v_right = msg.data[1] * self.wheel_radius

            # Smooth velocity
            v_left = self.moving_average_filter(v_left, self.left_velocity_buffer)
            v_right = self.moving_average_filter(v_right, self.right_velocity_buffer)

            # Compute linear and angular velocities
            linear_velocity = (v_left + v_right) / 2.0
            angular_velocity = (v_right - v_left) / self.wheel_base

            # Update odometry
            self.update_odometry(linear_velocity, angular_velocity)

        except IndexError:
            self.get_logger().error("Invalid wheel_velocity data received.")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in velocity_callback: {e}")

    def moving_average_filter(self, new_value, buffer):
        """
        Apply a moving average filter to smooth noisy data.
        """
        buffer.append(new_value)
        if len(buffer) > self.velocity_buffer_size:
            buffer.pop(0)
        return sum(buffer) / len(buffer)

    def update_odometry(self, linear_velocity, angular_velocity):
        """
        Update odometry based on the current velocities.
        """
        # Current time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0:
            self.get_logger().warning("Delta time is zero or negative, skipping update.")
            return

        # Update robot position
        delta_x = linear_velocity * cos(self.theta) * dt
        delta_y = linear_velocity * sin(self.theta) * dt
        delta_theta = angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta = self.normalize_angle(self.theta + delta_theta)

        # Publish odometry and TF
        self.publish_odom(linear_velocity, angular_velocity, current_time)
        self.publish_tf(current_time)

        # Publish robot footprint
        # self.publish_footprint()

        # Update last time
        self.last_time = current_time

    def publish_odom(self, linear_velocity, angular_velocity, current_time):
        """
        Publish odometry message.
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
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

        # Pose covariance
        odom_msg.pose.covariance = [
        0.01, 0.0,    0.0,    0.0,    0.0,    0.0,
        0.0,    0.01, 0.0,    0.0,    0.0,    0.0,
        0.0,    0.0,    0.01, 0.0,    0.0,    0.0,
        0.0,    0.0,    0.0,    0.01, 0.0,    0.0,
        0.0,    0.0,    0.0,    0.0,    0.01, 0.0,
        0.0,    0.0,    0.0,    0.0,    0.0,    0.01
    ]

        # Twist
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity


        # Publish odometry
        self.odom_pub.publish(odom_msg)

    def publish_tf(self, current_time):
        """
        Publish transform from odom to base_footprint.
        """
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
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

    '''def publish_footprint(self):
        """
        Publish the robot's footprint as a polygon.
        """
        footprint = PolygonStamped()
        footprint.header.stamp = self.get_clock().now().to_msg()
        footprint.header.frame_id = 'base_footprint'

        # Define the four corners of the robot footprint
        half_length = self.robot_length / 2.0
        half_width = self.robot_width / 2.0

        footprint.polygon.points = [
            Point32(x=half_length, y=half_width, z=0.0),
            Point32(x=half_length, y=-half_width, z=0.0),
            Point32(x=-half_length, y=-half_width, z=0.0),
            Point32(x=-half_length, y=half_width, z=0.0)
        ]

        self.footprint_pub.publish(footprint)'''

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """
        Convert Euler angles to a quaternion.
        """
        half_roll = roll / 2.0
        half_pitch = pitch / 2.0
        half_yaw = yaw / 2.0

        sin_r = sin(half_roll)
        cos_r = cos(half_roll)
        sin_p = sin(half_pitch)
        cos_p = cos(half_pitch)
        sin_y = sin(half_yaw)
        cos_y = cos(half_yaw)

        qx = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y
        qy = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y
        qz = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y
        qw = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y

        return [qx, qy, qz, qw]

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to [-pi, pi].
        """
        return atan2(sin(angle), cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
