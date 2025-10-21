import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import csv
import os
import math
from ament_index_python.packages import get_package_share_directory

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__("waypoint_publisher")

        self.declare_parameter('wheelbase', 1.8)
        self.declare_parameter('kV', 5.0)
        self.declare_parameter('min_l', 10.0)
        self.declare_parameter('max_l', 400.0)
        self.declare_parameter('distance', 0.0)

        self.wheelbase = self.get_parameter('wheelbase').value
        self.kV = self.get_parameter('kV').value
        self.min_l = self.get_parameter('min_l').value
        self.max_l = self.get_parameter('max_l').value
        self.distance = self.get_parameter("distance").value


        self.last_lookahead_idx = -1

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_z = 0.0
        self.vehicle_v = 0.0
        self.vehicle_yaw = 0.0

        self.tracking_point_publisher = self.create_publisher(PointStamped, '/track_point', 10)
        self.lookahead_distance_publisher = self.create_publisher(Float64, '/lookahead_distance', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        packageName = 'pure_pursuit_pid'
        csvFile = 'trajectory.csv'
        packagePath = get_package_share_directory(packageName)
        csvPath = os.path.join(packagePath, csvFile)

        outputFile = 'transformed.csv'
        outputPath = os.path.join(packagePath, outputFile)

        closeFile = 'close.csv'
        closePath = os.path.join(packagePath, closeFile)
        self.closestFile = open(closePath, mode='w', newline='')
        self.closestWriter = csv.writer(self.closestFile)

        self.waypoints = self.readCSV(csvPath)
        self.writeCSV(self.waypoints, outputPath)

        self.timer = self.create_timer(1.0, self.update_tp)

    def odom_callback(self, odom_msg: Odometry):
        self.vehicle_x = odom_msg.pose.pose.position.x
        self.vehicle_y = odom_msg.pose.pose.position.y
        self.vehicle_z = odom_msg.pose.pose.position.z
        self.vehicle_v = odom_msg.twist.twist.linear.x

        q = odom_msg.pose.pose.orientation
        self.vehicle_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def readCSV(self, csvPath):
        waypoints = []
        try:
            with open(csvPath, 'r') as file:
                read = csv.reader(file)
                for row in read:
                    if len(row) < 3:
                        continue
                    x = float(row[0])
                    y = float(row[1])
                    z = float(row[2])
                    waypoints.append((x, y, z))
        except FileNotFoundError:
            self.get_logger().error(f"File {csvPath} was not found.")
        except Exception as e:
            self.get_logger().error(f"Error reading waypoints: {e}")

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints.")
        return waypoints

    def writeCSV(self, waypoints, output_csv):
        try:
            with open(output_csv, 'w', newline='') as file:
                writer = csv.writer(file)
                for point in waypoints:
                    writer.writerow(point)
            self.get_logger().info(f"Wrote waypoints to {output_csv}.")
        except Exception as e:
            self.get_logger().error(f"Failed to write waypoints: {e}")
            

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    def compute_lad(self):
        return max(self.min_l, min(self.kV * self.vehicle_v, self.max_l))

    def update_tp(self):
        if not self.waypoints:
            self.get_logger().warn("No waypoints loaded")
            return

        x = self.vehicle_x
        y = self.vehicle_y
        z = self.vehicle_z

        self.lookahead_distance = self.compute_lad()
        self.get_logger().info(f"LD = {self.lookahead_distance:.3f}")
        lookahead_idx = self.find_lookahead_idx(x, y ,z)

        if lookahead_idx is None:
            self.get_logger().info("No forward waypoint found")
            return

        wx, wy, wz = self.waypoints[lookahead_idx]
        self.distance = math.hypot(wx - x, wy - y)

        ld_msg = Float64()
        ld_msg.data = self.lookahead_distance
        self.lookahead_distance_publisher.publish(ld_msg)

        tp = PointStamped()
        tp.header.stamp = self.get_clock().now().to_msg()
        tp.header.frame_id = 'odom'
        tp.point.x = wx
        tp.point.y = wy
        tp.point.z = wz
        #self.get_logger().info(f"TP: {tp.point.x:.2f}, {tp.point.y:.2f}")
        self.tracking_point_publisher.publish(tp)

    def find_closest_forward_idx(self, x, y, z):
        min_dist = float('inf')
        closest_idx = None
        for i, (wx, wy, wz) in enumerate(self.waypoints):
            dx = x - wx
            dy = y - wy
            dz = z - wz
            distance = math.hypot(dx, dy, dz)
            if distance < min_dist:
                min_dist = distance
                closest_idx = i
        return closest_idx


    def find_lookahead_idx(self, x, y,z):
        idx = self.find_closest_forward_idx(x, y,z)
        cl_idx_x, cl_idx_y , cl_idx_z = self.waypoints[idx]
        self.closestWriter.writerow([cl_idx_x, cl_idx_y,cl_idx_z])
        #From the closest_index we have to find the closest lookahead index
        if idx is None:
            return None

        heading_x = math.cos(self.vehicle_yaw)
        heading_y = math.sin(self.vehicle_yaw)

        startidx = max(min(idx, self.last_lookahead_idx), 0)
        #for i in range(max(idx, self.last_lookahead_idx + 1), len(self.waypoints)):
        for i in range(startidx, len(self.waypoints)):
            wx, wy, wz = self.waypoints[i]
            dx = wx - x
            dy = wy - y
            dz = wz - z
            dot = dx * heading_x + dy * heading_y
            
            if dot > 0:
                dist = math.hypot(wx - x, wy - y)
                if dist >= self.lookahead_distance:
                    self.last_lookahead_idx = i
                    return i
                
        return len(self.waypoints) - 1



def main():
    rclpy.init()
    node = WaypointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
