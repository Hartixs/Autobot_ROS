#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.declare_parameter('upper_threshold', 11.5)
        self.declare_parameter('lower_threshold', 0.05)
        self.upper = self.get_parameter('upper_threshold').value
        self.lower = self.get_parameter('lower_threshold').value
        self.get_logger().info(f"Scan filter: keeping [{self.lower}m – {self.upper}m]")

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', sensor_qos)

    def scan_callback(self, msg: LaserScan):
        filtered = LaserScan()
        filtered.header          = msg.header
        filtered.angle_min       = msg.angle_min
        filtered.angle_max       = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment  = msg.time_increment
        filtered.scan_time       = msg.scan_time
        filtered.range_min       = msg.range_min
        filtered.range_max       = self.upper
        filtered.ranges = [
            r if (math.isfinite(r) and self.lower < r < self.upper) else float('inf')
            for r in msg.ranges
        ]
        filtered.intensities = list(msg.intensities) if msg.intensities else []
        self.pub.publish(filtered)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()