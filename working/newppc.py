import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseArray
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
import math
import time

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.declare_parameter('wheelbase', 1.8)
        self.declare_parameter('lookahead_min', 10.0)
        self.declare_parameter('lookahead_gain', 1.0)
        self.declare_parameter('ts', 6.25)

        self.L = self.get_parameter('wheelbase').value
        self.kv = self.get_parameter('lookahead_gain').value
        self.ld_min = self.get_parameter('lookahead_min').value
        self.target_speed = self.get_parameter('ts').value

        # Default States
        self.lookahead_point = None #no lookahead found first
        self.distance = 0.0
        self.current_velocity = 0.0 #vehicle at rest
        self.odom_pose = None
        self.path = [] #initially empty path
        self.wp_idx = 0 #last index of lookahead point, intially is at 0

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_orient = 0.0
        self.vehicle_yaw = 0.0
        self.vehicle_v = 0.0
        
        #Subscribers
        self.create_subscription(Path, '/waypoints', self.path_callback, 10) #subscribe to waypoints topic
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/pose_info', self.pose_callback, 10)

        # Publisher for velocity reference error for PID and Publisher for Angular Velocity for Steering Control
        self.v_ref_pub = self.create_publisher(Float64, '/v_ref', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/pp_cmd_vel', 10)
        self.lookahead_pub = self.create_publisher(PointStamped, '/lookahead_point',10)
        self.car_point = self.create_publisher(PointStamped, '/car_point',10)
        self.track_point_pub = self.create_publisher(PointStamped, '/track_point', 10) #edit by rivaldo to use desired position in PID

        self.create_timer(0.05, self.purepursuit_callback) #20Hz

    #Callbacks
    def path_callback(self, msg):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses] #storing waypoints as tuples of (x,y) as z is not needed
        #self.get_logger().info(f"Received path with {len(self.path)} waypoints.")

    def odom_callback(self, odom_msg: Odometry):
        #Get Vehicle Velocity
        self.vehicle_v = odom_msg.twist.twist.linear.x

    def pose_callback(self, msg):
        pose = msg.poses[1] #Car position stored here

        #Storing Car X, Y and Z as well as the Yaw
        self.vehicle_x = pose.position.x
        self.vehicle_y = pose.position.y
        self.vehicle_z = pose.position.z
        self.vehicle_orient = pose.orientation
        self.vehicle_yaw = self.euler_from_quaternion(self.vehicle_orient.x, self.vehicle_orient.y, self.vehicle_orient.z, self.vehicle_orient.w)

    def euler_from_quaternion(self, x, y, z, w):
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw
    
    def find_closest_waypoint_idx(self, x, y, yaw):
        close_dist = float('inf')
        close_idx = 0
        for i in range(self.wp_idx, len(self.path)):
            wp_x, wp_y = self.path[i]
            dist = math.hypot(wp_x - x, wp_y - y)
            dx = wp_x - x
            dy = wp_y - y
            dot = dx * math.cos(yaw) + dy * math.sin(yaw)
            if dot < -0.5:
                continue  # waypoint is behind the vehicle

            if dist < close_dist:
                close_dist = dist
                close_idx = i
        return close_idx
    
    def find_lookaheadpoint(self, x, y, ld, future_wp):
        for i, waypoint in enumerate(future_wp):
            wp_x, wp_y = waypoint[:2]
            dx = wp_x - x
            dy = wp_y - y
            distance = math.hypot(dx, dy)

            if distance >= ld:
                self.wp_idx = i + self.wp_idx  # Update global waypoint index
                return (wp_x, wp_y)
        return None

    #Functions Needed for Calculations
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))
    
    def get_alpha(self, x, y, yaw, lap_x, lap_y):
        dx = lap_x - x
        dy = lap_y - y
        target_angle = math.atan2(dy, dx)
        
        return self.normalize_angle(target_angle - yaw)
    
    def compute_curvature(self, alpha, ld):
        return 2.0 * math.sin(alpha) / ld if ld >1e-6 else 0.0
   
    def steering_angle_to_angular_velocity(self, steering_angle):
        v = self.vehicle_v
        L = self.L
        w = (v/L) * math.tan(steering_angle)
        return w
    
    #Pure Pursuit Control Loop Logic is contained below
    def purepursuit_callback(self):
        if not self.path:
            return  # No path received yet        
        #Get Current Car Position
        x = self.vehicle_x
        y = self.vehicle_y
        yaw = self.vehicle_yaw
       
        #Establish Dynamic Lookahead Distance
        ld = max(self.ld_min, self.kv * self.vehicle_v) #LD is Max between lookahead minimum, Kv * velocity)
        #ld = self.kv * self.current_velocity

        #With established ld, we first find the closest waypoint index in front of the vehicle from the path
        self.wp_idx = self.find_closest_waypoint_idx(x, y, yaw)

        #Create subpath starting from closest index and ahead
        future_wp = self.path[self.wp_idx:] #consider only waypoints ahead of the vehicle
        #Establish the lookahead point using the function find_lookaheadpoint taking into consideration car position, ld and subpath
        self.lookahead_point = self.find_lookaheadpoint(x, y, ld, future_wp)

        #Condition of lookahead is not found, output error message in terminal
        if self.lookahead_point is None: #if it returns None we need to log into the terminal that a point was not found and return
            self.get_logger().info("No lookahead point found.")
            return
        
        #At this point we should have a lookahead point so extract its x and y values
        lap_x, lap_y = self.lookahead_point #works since self.lookahead_point is a tuple (x,y)
        
        #For RViz2
        point_msg = PointStamped()
        point_msg.header.frame_id = "world"
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = lap_x
        point_msg.point.y = lap_y
        point_msg.point.z = 0.0
        self.track_point_pub.publish(point_msg) #another edit by rivaldo to also use position in PID

        p_msg = PointStamped()
        p_msg.header.frame_id = "world"
        p_msg.header.stamp = self.get_clock().now().to_msg()
        p_msg.point.x = x
        p_msg.point.y = y
        p_msg.point.z = 0.0
        self.car_point.publish(p_msg)

        self.lookahead_pub.publish(point_msg)

        #Since we have a lookahead point, we can now compute its distance from the vehicle
        self.distance = math.sqrt((lap_x - x)**2 + (lap_y - y)**2)

        #Now we must calculate alpha using the car position, waypoint position and yaw
        #the get_alpha function returns it normalized between -pi and pi already
        alpha = self.get_alpha(x, y, yaw, lap_x, lap_y)

        #With alpha calculated and normalized, we can compute curvature
        curvature = self.compute_curvature(alpha, ld)

        #From curvature, we can compute steering angle
        steering_angle = math.atan(curvature * self.L)

        #Gazebo expects a Twist message for steering control as angular velocity so convert steering angle to angular velocity
        angular_velocity = self.steering_angle_to_angular_velocity(steering_angle)

        #Now with angular velocity, we can publish to pp__cmd_vel topic or known as pure pursuit command velocity topic
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
