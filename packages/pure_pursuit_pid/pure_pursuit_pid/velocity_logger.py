# velocity_logger_node.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import csv
import os
import signal
import sys
from ament_index_python.packages import get_package_share_directory
import time

class VelocityLogger(Node):
    def __init__(self):
        super().__init__('velocity_logger')

        # Variables to store velocities
        self.vel_measured = None
        self.vel_reference = None

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Float64, '/v_ref', self.v_ref_callback, 10)

        # File setup inside ROS2 package
        package_name = 'pure_pursuit_pid'
        package_path = get_package_share_directory(package_name)
        self.file_path = os.path.join(package_path, 'velocity_log.csv')

        self.file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time_sec', 'v_measured', 'v_reference'])

        self.start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info(f'Velocity logging started. Writing to: {self.file_path}')

        # Register signal handler for Ctrl-C
        signal.signal(signal.SIGINT, self.signal_handler)

        # Timer for logging at 20 Hz
        self.create_timer(0.05, self.log_velocity)

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.vel_measured = (vx**2 + vy**2)**0.5  # magnitude of velocity

    def v_ref_callback(self, msg: Float64):
        self.vel_reference = msg.data

    def log_velocity(self):
        # Only log if we have received messages
        if self.vel_measured is None or self.vel_reference is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.writer.writerow([now, self.vel_measured, self.vel_reference])
        self.file.flush()  # ensure data is written to disk immediately

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received! Closing CSV file...')
        if not self.file.closed:
            self.file.close()
        self.get_logger().info('CSV file saved successfully.')
        rclpy.shutdown()
        sys.exit(0)

    def destroy_node(self):
        if not self.file.closed:
            self.file.close()
            self.get_logger().info('CSV file closed on node shutdown.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VelocityLogger()
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
