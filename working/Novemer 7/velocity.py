import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os
import signal
import sys
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import Clock

class VelocityRecorder(Node):
    def __init__(self):
        super().__init__('velocity_recorder')

        # Subscribe to /odom topic
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # CSV file path
        package_name = 'pure_pursuit_pid'
        package_path = get_package_share_directory(package_name)
        self.file_path = os.path.join(package_path, 'velocity.csv')

        # Open file and write header
        self.file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.file)

        self.get_logger().info(f'Recording velocity data to: {self.file_path}')

        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        # For timestamp reference
        self.start_time = self.get_clock().now()

    def odom_callback(self, msg):
        # Extract linear velocities
        v_x = msg.twist.twist.linear.x

        # Compute time since start
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        start = self.start_time.seconds_nanoseconds()[0] + self.start_time.seconds_nanoseconds()[1] * 1e-9
        time = stamp - start
        # Write to CSV
        self.writer.writerow([time, v_x])
        self.file.flush()

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received! Closing CSV file...')
        self.file.close()
        self.get_logger().info('File saved successfully.')
        rclpy.shutdown()
        sys.exit(0)

    def destroy_node(self):
        if not self.file.closed:
            self.file.close()
            self.get_logger().info('Velocity recording stopped and file saved.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VelocityRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
