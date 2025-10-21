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

        #Lookahead Distance Parameter
        self.declare_parameter('wheelbase', 1.8)
        self.declare_parameter('kV', 1.0)
        self.declare_parameter('min_l', 20.0)

        #Settings self variables for wheelbase, Velocity Gain and Minimum Lookahead
        self.wheelbase = self.get_parameter('wheelbase').value
        self.kV = self.get_parameter('kV').value
        self.min_l = self.get_parameter('min_l').value

        #Internal vehicle state
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_z = 0.0
        self.vehicle_v = 0.0
        self.vehicle_yaw = 0.0

        #Publisher to /waypoints and /track_point
        self.tracking_point_publisher = self.create_publisher(PointStamped, '/track_point',10)
        self.lookahead_distance_publisher = self.create_publisher(Float64, '/lookahead_distance', 10)

        #Subscriber to Odometry for robot current telemetry data x,y,z angles etc.
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback,10)

        #From previuos work to search for a waypoint file within package folder at a certain path
        packageName = 'pure_pursuit_pid'
        csvFile = 'trajectory.csv'
        packagePath = get_package_share_directory(packageName)
        csvPath = os.path.join(packagePath, csvFile)

        outputFile = 'transformed.csv'
        outputPath = os.path.join(packagePath, outputFile)

        #Read CSV file and store as waypoints array
        self.waypoints = self.readCSV(csvPath)
        self.writeCSV(self.waypoints, outputPath)

        #Timer to call update tracking point as the vehicle is in motion
        self.timer = self.create_timer(1.0, self.update_tp)

    def odom_callback(self,odom_msg: Odometry):
        #Update values for car position for x y z and velocity
        self.vehicle_x = odom_msg.pose.pose.position.x
        self.vehicle_y = odom_msg.pose.pose.position.y
        self.vehicle_z = odom_msg.pose.pose.position.z
        self.vehicle_v = odom_msg.twist.twist.linear.x

        #This is for the vehicles angle
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
                    y = float(row[1]) * -1 #needed to flip y coordinates due to inability to transform coordinates to same frame as odometry
                    z = float(row[2])
                    waypoints.append((x,y,z))
        
        except FileNotFoundError: #if the file was not found spit out this error in log
            self.get_logger().error(f"File {csvPath} was not found.")
        except Exception as e: #if waypoint cannnot be read
             self.get_logger().error(f"Error in Reading Waypoints : {e}")
        
        self.get_logger().info(f"Recieved {len(waypoints)} waypoints !!")
        return waypoints
    
    def writeCSV(self, waypoints, output_csv):
        try:
            with open(output_csv, 'w', newline='') as file:
                writer = csv.writer(file)
                for point in waypoints:
                    writer.writerow(point)
            self.get_logger().info(f"Waypoints written to {output_csv} successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to write waypoints to {output_csv}: {e}")

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def compute_lad(self):
        return max(self.min_l, self.wheelbase, self.kV * abs(self.vehicle_v))

    def update_tp(self):
        if not self.waypoints:
            self.get_logger().warn("No waypoints loaded")
            return

        #Get Robot x y and z values as well as velocity
        x = self.vehicle_x
        y = self.vehicle_y
        dv = self.vehicle_v

        self.lookahead_distance = self.compute_lad()

        #Using function to find the closest index to current car position
        closest_idx = self.find_closest_ahead_idx(x, y)
        if closest_idx is None:
            self.get_logger().info("No forward waypoint")
            return

        wx, wy, wz = self.waypoints[closest_idx]

        distance = math.hypot(wx - x, wy - y)

        self.lookahead_distance = max(self.compute_lad(), distance)

        #Publish ld as a Float64 message for Pure Pursuit to recieve for steering angle
        ld_msg = Float64()
        ld_msg.data = self.lookahead_distance
        self.lookahead_distance_publisher.publish(ld_msg)

        #Create point that takes values of this tracking point
        tp = PointStamped()
        tp.header.stamp = self.get_clock().now().to_msg()
        tp.header.frame_id = 'odom'
        tp.point.x = wx
        tp.point.y = wy
        tp.point.z = wz

        #Publish Tracking Point to topic /track_point
        self.tracking_point_publisher.publish(tp)
        self.get_logger().info(f"TP: {tp.point.x:.2f}, {tp.point.y:.2f}")

    def find_closest_ahead_idx(self, x, y):
        dx = math.cos(self.vehicle_yaw)
        dy = math.sin(self.vehicle_yaw)

        closest_idx = None
        closest_dist = float('inf')

        for i, (wx, wy, wz) in enumerate(self.waypoints):
            vx = wx - x
            vy = wy - y
            dot = vx * dx + vy * dy

            if dot > 0:
                distance = math.hypot(vx, vy)
                if distance < closest_dist:
                    cloest_dist = distance
                    closest_idx = i
        return closest_idx
        
def main():
    rclpy.init()
    node = WaypointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()