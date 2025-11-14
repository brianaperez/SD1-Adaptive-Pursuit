# path_recorder_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PointStamped
from std_msgs.msg import Float64
import csv
import os
import signal
import sys
from ament_index_python.packages import get_package_share_directory

class ErrorRecorder(Node):
    def __init__(self):
        super().__init__('error_recorder')

        # Subscribers
        #self.create_subscription(PoseArray, '/pose_info', self.listener_callback, 10)
        self.create_subscription(PointStamped, '/error', self.listener_callback, 10)

        # CSV file path
        packageName = 'pure_pursuit_pid'
        packagePath = get_package_share_directory(packageName)
        self.file_path = os.path.join(packagePath, 'tracking.csv')

        # Open file and write header
        self.file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.file)

        self.get_logger().info(f'Recording started. Writing to: {self.file_path}')

        # Register signal handler for Ctrl-C
        signal.signal(signal.SIGINT, self.signal_handler)

    def listener_callback(self, msg):
        error = msg.point.x
        time = msg.point.y
        self.writer.writerow([error, time])
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
            self.get_logger().info('Path recording stopped and file saved.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ErrorRecorder()
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
