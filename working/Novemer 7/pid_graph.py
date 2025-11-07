import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import csv
import os
import sys
import signal
from ament_index_python.packages import get_package_share_directory
import math

class PIDRecorder(Node):
    def __init__(self):
        super().__init__('pid_recorder')

        # Subscribers
        self.create_subscription(Twist, '/pid_cmd_vel', self.pid_callback, 10)
        self.create_subscription(Float64, '/pid_dt', self.dt_callback, 10)

        self.dt = None
        self.pid_output = None

        # File setup
        package_name = 'pure_pursuit_pid'
        pkg_path = get_package_share_directory(package_name)
        self.file_path = os.path.join(pkg_path, 'pid.csv')
        self.file = open(self.file_path, 'w', newline='')
        self.writer = csv.writer(self.file)
        #self.writer.writerow(['pid_output', 'dt'])
        self.get_logger().info(f"PID logging to: {self.file_path}")

        # Handle Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

    def pid_callback(self, msg):
        self.pid_output = msg.linear.x
        self.write_data()

    def dt_callback(self, msg):
        self.dt = msg.data
        self.write_data()

    def write_data(self):
        # Only write when both values are valid
        if self.pid_output is not None and self.dt is not None:
            if not math.isnan(self.pid_output) and not math.isnan(self.dt):
                self.writer.writerow([self.pid_output, self.dt])
                self.file.flush()

    def signal_handler(self, sig, frame):
        self.get_logger().info('Interrupt received, closing file...')
        self.file.close()
        rclpy.shutdown()
        sys.exit(0)

    def destroy_node(self):
        if not self.file.closed:
            self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIDRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
