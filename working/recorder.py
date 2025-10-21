# path_recorder_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
import csv
import os
import signal
import sys
from ament_index_python.packages import get_package_share_directory

class PathRecorder(Node):
    def __init__(self):
        super().__init__('path_recorder')
        self.subscription = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.listener_callback,
            10)
        
        packageName = 'pure_pursuit_pid'
        packagePath = get_package_share_directory(packageName)
        self.file_path = os.path.join(packagePath, 'recorded_path.csv')
        
        self.file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['x', 'y', 'z'])

        self.get_logger().info(f'Path recording started. Writing to: {self.file_path}')
        
        # Register signal handler for Ctrl-C
        signal.signal(signal.SIGINT, self.signal_handler)

    def listener_callback(self, msg):
        pose = msg.poses[1]  # Assuming single pose in PoseArray
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        self.writer.writerow([x, y, z])

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received! Saving and closing the CSV file...')
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
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # This will rarely be reached because of signal handler, but good to have
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
