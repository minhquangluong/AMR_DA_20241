import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Polygon, Point32
from visualization_msgs.msg import Marker

class Robot_TF(Node):
    def __init__(self):
        super().__init__('robot_tf_node')

        # Param robot
        self.declare_parameter('robot_length', 0.6)  
        self.declare_parameter('robot_width', 0.4)   
        self.declare_parameter('wheel_D', 0.18)  
        self.declare_parameter('wheel_base', 0.3)  # m

        self.robot_length = self.get_parameter('robot_length').value
        self.robot_width = self.get_parameter('robot_width').value
        self.wheel_D = self.get_parameter('wheel_D').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # Param tf
        self.declare_parameter('tf_parameters.laser.translation', [0.0, 0.0, 0.0])
        self.declare_parameter('tf_parameters.laser.rotation', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('tf_parameters.left_wheel.translation', [0.0, 0.0, 0.0])
        self.declare_parameter('tf_parameters.left_wheel.rotation', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('tf_parameters.right_wheel.translation', [0.0, 0.0, 0.0])
        self.declare_parameter('tf_parameters.right_wheel.rotation', [0.0, 0.0, 0.0, 1.0])

        # Laser
        self.laser_translation = self.get_parameter_or('tf_parameters.laser.translation').value
        self.laser_rotation = self.get_parameter_or('tf_parameters.laser.rotation').value
        # Left wheel
        self.left_wheel_translation = self.get_parameter_or('tf_parameters.left_wheel.translation').value
        self.left_wheel_rotation = self.get_parameter_or('tf_parameters.left_wheel.rotation').value
        # Right wheel
        self.right_wheel_translation = self.get_parameter_or('tf_parameters.right_wheel.translation').value
        self.right_wheel_rotation = self.get_parameter_or('tf_parameters.right_wheel.rotation').value

        self.tf_broadcaster = TransformBroadcaster(self)

        # Publish robot footprint
        self.footprint_pub = self.create_publisher(Polygon, 'robot_footprint', 10)

        # Publish robot visualization
        self.marker_pub = self.create_publisher(Marker, 'robot_visualization', 10)

        # Timer 
        self.create_timer(0.05, self.publish_tf_and_footprint)
      
        self.create_timer(0.01,  self.publish_marker)

    def publish_tf_and_footprint(self):
        # Base_footprint -> base_link
        tf_ = TransformStamped()
        tf_.header.stamp = self.get_clock().now().to_msg()
        tf_.header.frame_id = 'base_footprint'  
        tf_.child_frame_id = 'base_link'
        tf_.transform.translation.x = 0.0
        tf_.transform.translation.y = 0.0
        tf_.transform.translation.z = 0.0
        tf_.transform.rotation.x = 0.0
        tf_.transform.rotation.y = 0.0
        tf_.transform.rotation.z = 0.0
        tf_.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf_)

        # Base_link -> laser
        tf_ = TransformStamped()
        tf_.header.stamp = self.get_clock().now().to_msg()
        tf_.header.frame_id = 'base_link'
        tf_.child_frame_id = 'laser'
        tf_.transform.translation.x = self.laser_translation[0]
        tf_.transform.translation.y = self.laser_translation[1]
        tf_.transform.translation.z = self.laser_translation[2]
        tf_.transform.rotation.x = self.laser_rotation[0]
        tf_.transform.rotation.y = self.laser_rotation[1]
        tf_.transform.rotation.z = self.laser_rotation[2]
        tf_.transform.rotation.w = self.laser_rotation[3]

        self.tf_broadcaster.sendTransform(tf_)

        # Base_link -> left_wheel
        tf_ = TransformStamped()
        tf_.header.stamp = self.get_clock().now().to_msg()
        tf_.header.frame_id = 'base_link' 
        tf_.child_frame_id = 'left_wheel'
        tf_.transform.translation.x = self.left_wheel_translation[0]
        tf_.transform.translation.y = self.left_wheel_translation[1]
        tf_.transform.translation.z = self.left_wheel_translation[2]
        tf_.transform.rotation.x = self.left_wheel_rotation[0]
        tf_.transform.rotation.y = self.left_wheel_rotation[1]
        tf_.transform.rotation.z = self.left_wheel_rotation[2]
        tf_.transform.rotation.w = self.left_wheel_rotation[3]
        self.tf_broadcaster.sendTransform(tf_)

        # Base_link -> right_wheel
        tf_ = TransformStamped()
        tf_.header.stamp = self.get_clock().now().to_msg()
        tf_.header.frame_id = 'base_link' 
        tf_.child_frame_id = 'right_wheel'
        tf_.transform.translation.x = self.right_wheel_translation[0]
        tf_.transform.translation.y = self.right_wheel_translation[1]
        tf_.transform.translation.z = self.right_wheel_translation[2]
        tf_.transform.rotation.x = self.right_wheel_rotation[0]
        tf_.transform.rotation.y = self.right_wheel_rotation[1]
        tf_.transform.rotation.z = self.right_wheel_rotation[2]
        tf_.transform.rotation.w = self.right_wheel_rotation[3]
        self.tf_broadcaster.sendTransform(tf_)

        # Create robot footprint
        footprint = Polygon()
        footprint.points = [Point32(x=-self.robot_width/2, y=-self.robot_length/2, z=0.0),
                            Point32(x=self.robot_width/2, y=-self.robot_length/2, z=0.0),
                            Point32(x=self.robot_width/2, y=self.robot_length/2, z=0.0),
                            Point32(x=-self.robot_width/2, y=self.robot_length/2, z=0.0)]

        self.footprint_pub.publish(footprint)

        # Create marker for visualization
        
    def publish_marker(self):
    # Create marker for visualization
       marker = Marker()
       marker.header.frame_id = 'base_footprint'
       marker.header.stamp = self.get_clock().now().to_msg()
       marker.ns = "robot"
       marker.id = 0
       marker.type = Marker.CUBE
       marker.action = Marker.ADD
       marker.pose.position.x = 0.0
       marker.pose.position.y = 0.0
       marker.pose.position.z = 0.0
       marker.scale.x = self.robot_width
       marker.scale.y = self.robot_length
       marker.scale.z = 0.1
       marker.color.a = 1.0  # Alpha
       marker.color.r = 0.0
       marker.color.g = 1.0  # Green
       marker.color.b = 0.0


       self.marker_pub.publish(marker)

    


       
       

def main():
    rclpy.init()
    robot_tf_node = Robot_TF()

    try:
        rclpy.spin(robot_tf_node)
    finally:
        robot_tf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
