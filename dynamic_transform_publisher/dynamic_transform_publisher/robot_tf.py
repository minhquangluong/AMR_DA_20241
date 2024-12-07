import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class RobotTFNode(Node):
    def __init__(self):
        super().__init__('robot_tf_node')

        # TF Broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Parameters
        self.declare_parameter('tf_parameters.laser.translation', [0.16, 0.0, 0.03])
        self.declare_parameter('tf_parameters.laser.rotation', [0.0, 0.0, 0.0, 1.0])

        # Load Parameters
        self.laser_translation = self.get_parameter('tf_parameters.laser.translation').value
        self.laser_rotation = self.get_parameter('tf_parameters.laser.rotation').value

        # Log loaded parameters
        self.get_logger().info(f"Laser Translation: {self.laser_translation}")
        self.get_logger().info(f"Laser Rotation: {self.laser_rotation}")

        # Timer
        self.create_timer(0.1, self.publish_tf)

    def publish_tf(self):
        current_time = self.get_clock().now().to_msg()

        # Base_footprint -> Base_link
        base_to_link_tf = TransformStamped()
        base_to_link_tf.header.stamp = current_time
        base_to_link_tf.header.frame_id = 'base_footprint'
        base_to_link_tf.child_frame_id = 'base_link'
        base_to_link_tf.transform.translation.x = 0.0
        base_to_link_tf.transform.translation.y = 0.0
        base_to_link_tf.transform.translation.z = 0.12
        base_to_link_tf.transform.rotation.x = 0.0
        base_to_link_tf.transform.rotation.y = 0.0
        base_to_link_tf.transform.rotation.z = 0.0
        base_to_link_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(base_to_link_tf)

        # Base_link -> Laser
        link_to_laser_tf = TransformStamped()
        link_to_laser_tf.header.stamp = current_time
        link_to_laser_tf.header.frame_id = 'base_link'
        link_to_laser_tf.child_frame_id = 'laser'
        link_to_laser_tf.transform.translation.x = self.laser_translation[0]
        link_to_laser_tf.transform.translation.y = self.laser_translation[1]
        link_to_laser_tf.transform.translation.z = self.laser_translation[2]
        link_to_laser_tf.transform.rotation.x = self.laser_rotation[0]
        link_to_laser_tf.transform.rotation.y = self.laser_rotation[1]
        link_to_laser_tf.transform.rotation.z = self.laser_rotation[2]
        link_to_laser_tf.transform.rotation.w = self.laser_rotation[3]
        self.tf_broadcaster.sendTransform(link_to_laser_tf)

        # Log TF data
        self.get_logger().debug(f"Published TF: {base_to_link_tf}")
        self.get_logger().debug(f"Published TF: {link_to_laser_tf}")


def main():
    rclpy.init()
    node = RobotTFNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
