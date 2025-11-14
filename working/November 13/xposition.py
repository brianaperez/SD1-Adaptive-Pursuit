import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import csv
import os
import signal
import sys
from ament_index_python.packages import get_package_share_directory

class PositionRecorder(Node):
    def __init__(self):
        super().__init__('position_recorder')

        # Subscribe to /pose_info topic
        self.create_subscription(PoseArray, '/pose_info', self.pose_callback, 10)

        # CSV file path
        package_name = 'pure_pursuit_pid'
        package_path = get_package_share_directory(package_name)
        self.file_path = os.path.join(package_path, 'position.csv')

        # Open file and write header
        self.file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.file)

        self.get_logger().info(f'Recording position data to: {self.file_path}')

        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        # For timestamp reference
        self.start_time = self.get_clock().now()

    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) < 2:
            return  # Safety check

        # Extract x-position from pose array index 1
        x_pos = msg.poses[1].position.x

        # Compute time since start
        now = self.get_clock().now()
        sec, nanosec = now.seconds_nanoseconds()
        t = sec + nanosec / 1e9

        # Write to CSV
        self.writer.writerow([t, x_pos])
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
            self.get_logger().info('Position recording stopped and file saved.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PositionRecorder()
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
