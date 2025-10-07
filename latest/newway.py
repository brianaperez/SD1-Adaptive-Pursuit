import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Float64
import csv
import os
import math
from ament_index_python.packages import get_package_share_directory

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__("waypoint_publisher")

        packageName = 'pure_pursuit_pid'
        csvFile = 'trajectory.csv'
        packagePath = get_package_share_directory(packageName)
        csvPath = os.path.join(packagePath, csvFile)
        self.waypoints = self.readCSV(csvPath)

        self.path_publish = self.create_publisher(Path, '/waypoints', 10)

        self.timer = self.create_timer(1.0, self.publish_path)

    def readCSV(self, csvPath):
        waypoints = []
        try:
            with open(csvPath, 'r') as file:
                read = csv.reader(file)
                for row in read:
                    if len(row) < 2:
                        continue
                    x = float(row[0])
                    y = float(row[1])
                    z = float(row[2]) if len(row) > 2 else 0.0 #ignore Z but keep it in tuple
                    waypoints.append((x, y, z))
        except FileNotFoundError:
            self.get_logger().error(f"File {csvPath} was not found.")
        except Exception as e:
            self.get_logger().error(f"Error reading waypoints: {e}")

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints.")
        return waypoints
    
    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "my_robot/odom" #or odom
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for waypoint in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "my_robot/odom"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.path_publish.publish(path_msg)
        #self.get_logger().info(f"Published path with {len(path_msg.poses)} waypoints.")

def main():
    rclpy.init()
    node = WaypointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()