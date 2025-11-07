import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
import csv
import os
import signal
import sys
from ament_index_python.packages import get_package_share_directory

class SteeringRecorder(Node):
    def __init__(self):
        super().__init__('steering_recorder')

        # Subscriber to Pure Pursuit command velocity
        self.create_subscription(PointStamped, '/steering_time', self.cmd_callback, 10)

        # CSV file path
        packageName = 'pure_pursuit_pid'
        packagePath = get_package_share_directory(packageName)
        self.file_path = os.path.join(packagePath, 'steer.csv')

        # Open file and write header
        self.file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['angular_z'])  # time for reference

        self.get_logger().info(f'Steering recording started. Writing to: {self.file_path}')

        # Register signal handler for Ctrl-C
        signal.signal(signal.SIGINT, self.signal_handler)

    def cmd_callback(self, msg):
        steer = msg.point.x  # angular velocity
        time = msg.point.y
        # Write a row to CSV
        self.writer.writerow([steer, time])
        self.file.flush()  # Ensure data is written to file

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received! Closing CSV file...')
        self.file.close()
        self.get_logger().info('File saved successfully.')
        rclpy.shutdown()
        sys.exit(0)

    def destroy_node(self):
        if not self.file.closed:
            self.file.close()
            self.get_logger().info('Steering recording stopped and file saved.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SteeringRecorder()
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
