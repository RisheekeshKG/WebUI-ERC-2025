import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import math
import time

class FakeLidarPublisher(Node):
    def __init__(self):
        super().__init__('fake_lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.5, self.publish_scan)  # 2 Hz

    def publish_scan(self):
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'

        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = math.pi / 4  # 8 rays (45 deg apart)
        scan.time_increment = 0.0
        scan.scan_time = 0.5
        scan.range_min = 0.1
        scan.range_max = 10.0

        scan.ranges = [1.0, 1.2, 1.5, 1.0, 0.8, 0.7, 0.9, 1.1]
        scan.intensities = []

        self.publisher_.publish(scan)
        self.get_logger().info('Published fake scan ✅')

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
