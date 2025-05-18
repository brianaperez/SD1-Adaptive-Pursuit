import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import os
from ament_index_python.packages import get_package_share_directory

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__("waypoint_publisher")
        self.publisher_ = self.create_publisher(Path, '/waypoints', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)

        packageName = 'adaptive_pursuit'
        csvFile = 'trajectory.csv'
        packagePath = get_package_share_directory(packageName)
        csvPath = os.path.join(packagePath, csvFile)

        self.waypoints = self.readCSV(csvPath)
    
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
                    z = float(row[2])

                    pose = PoseStamped()
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = z
                    pose.pose.orientation.w = 1.0
                    waypoints.append(pose)
        except FileNotFoundError: #if the file was not found spit out this error in log
            self.get_logger().error(f"File {csvPath} was not found.")
        except Exception as e: #if waypoint cannnot be read
             self.get_logger().error(f"Error in Reading Waypoints : {e}")
        return waypoints
    
    def publish_waypoints(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        path.poses = self.waypoints

        self.publisher_.publish(path)
        self.get_logger().info(f"Published {len(self.waypoints)} waypoints!")

def main():
    rclpy.init()
    node = WaypointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()