import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import csv
import os
import signal
import sys
from ament_index_python.packages import get_package_share_directory

class TrackingPointRecorder(Node):
    def __init__(self):
        super().__init__('track_recorder')

        self.sub = self.create_subscription(PointStamped, '/lookaheadpoint', self.callback, 10)

        packageName = 'pure_pursuit_pid'
        packagePath = get_package_share_directory(packageName)
        self.file_path = os.path.join(packagePath, 'track_path.csv')
        
        self.file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.file)


    def callback(self, msg):
        self.writer.writerow([msg.point.x, msg.point.y, msg.point.z])
        #self.get_logger().info(f"Logged point: x={msg.x}, y={msg.y}, z={msg.z}")

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received! Saving and closing the CSV file...')
        self.file.close()
        self.get_logger().info('File saved successfully.')
        rclpy.shutdown()
        sys.exit(0)

    def destroy_node(self):
        if not self.file.closed:
            self.file.close()
            self.get_logger().info('Track point recording stopped and file saved.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrackingPointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down tracking point recorder.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
