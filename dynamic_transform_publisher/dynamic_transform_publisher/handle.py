import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import serial


class Feed_back(Node):
    def __init__(self):
        super().__init__('feed_back')
        # param
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_frequency', 10.0)

        serial_port = self.get_parameter('serial_port').value
        baud_rate= self.get_parameter('baud_rate').value
        pub_hz= self.get_parameter('publish_frequency').value

        # serial
        self.ser_= serial.Serial(serial_port,baud_rate,timeout=1)
        self.pub_= self.create_publisher(Float64MultiArray, 'hall_data',10)

        timer_period = 1.0 / pub_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.ser_.in_waiting > 0:
            try:
                line = self.ser_.readline().decode('utf-8').strip()
                counts = line.split(',')
                if (len(counts)==2):
                    count_l = int(counts[0])  
                    count_r = int(counts[1])

                    msg = Float64MultiArray()
                    msg.data = [float(count_l), float(count_r)]
                    self.pub_.publish(msg)
                    self.get_logger().info(f'Published counts: left={count_l}, right={count_r}')
            except (ValueError, IndexError):
                self.get_logger().warn('Received invalid data')

def main(args = None):
    rclpy.init(args=args)
    hall = Feed_back()
    rclpy.spin(hall)        
    hall.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()         



