import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseArray
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64
import math

class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        # --- SAME AS PURE PURSUIT ---
        self.declare_parameter('wheelbase', 1.8)
        self.L = self.get_parameter('wheelbase').value

        # --- DIFFERENCE FROM PURE PURSUIT: Stanley gain (k) ---
        self.declare_parameter('stanley_gain', 2.5)
        self.k = self.get_parameter('stanley_gain').value

        # Vehicle state
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0
        self.vehicle_v = 0.0

        # Path
        self.path = []
        self.wp_idx = 0

        # Subscribers (exactly same as Pure Pursuit)
        self.create_subscription(Path, '/waypoints', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/pose_info', self.pose_callback, 10)

        # Publishers (same as PP except topic name)
        self.curvature_pub = self.create_publisher(Float64, '/curvature', 10)
        self.closest_point_pub = self.create_publisher(PointStamped, '/closest_point', 10)
        self.car_point = self.create_publisher(PointStamped, '/car_point', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/stanley_cmd_vel', 10)

        # Timer (same 20 Hz)
        self.create_timer(0.05, self.stanley_callback)

    # ---------------- Callbacks ----------------

    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_callback(self, msg):
        self.vehicle_v = msg.twist.twist.linear.x

    def pose_callback(self, msg):
        pose = msg.poses[1]
        self.vehicle_x = pose.position.x
        self.vehicle_y = pose.position.y

        # --- SAME YAW EXTRACTION AS PURE PURSUIT ---
        self.vehicle_yaw = self.euler_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

    # --- SAME FUNCTION FROM YOUR PURE PURSUIT ---
    def euler_from_quaternion(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)

    # EXACT SAME ANGLE NORMALIZER YOU USE
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # --------- Stanley-Specific Error Computation ---------

    def find_closest_segment_idx(self, x, y):
        """
        --- DIFFERENCE FROM PURE PURSUIT ---
        Instead of finding the closest waypoint ahead,
        Stanley requires the closest *segment* so we can compute
        cross-track error correctly.
        """
        min_dist = float('inf')
        closest_idx = 0

        for i in range(len(self.path) - 1):
            x1, y1 = self.path[i]
            x2, y2 = self.path[i+1]

            # Project vehicle onto segment
            dx_seg = x2 - x1
            dy_seg = y2 - y1
            dx = x - x1
            dy = y - y1
            seg_len = dx_seg*dx_seg + dy_seg*dy_seg

            if seg_len < 1e-6:
                continue

            t = max(0.0, min(1.0, (dx*dx_seg + dy*dy_seg) / seg_len))
            proj_x = x1 + t * dx_seg
            proj_y = y1 + t * dy_seg
            dist = math.hypot(x - proj_x, y - proj_y)

            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        return closest_idx

    def compute_cross_track_error(self, i, x, y):
        """
        --- DIFFERENCE FROM PURE PURSUIT ---
        Pure Pursuit never computes cross-track error.
        Stanley uses the signed perpendicular distance to the path.
        """
        x1, y1 = self.path[i]
        x2, y2 = self.path[i+1]

        dx_path = x2 - x1
        dy_path = y2 - y1

        dx = x - x1
        dy = y - y1

        # Signed cross product formula
        numerator = (dx * dy_path) - (dy * dx_path)
        denom = math.hypot(dx_path, dy_path)

        return numerator / denom if denom > 1e-6 else 0.0

    # ---------------- Control Loop ----------------

    def stanley_callback(self):
        if len(self.path) < 2:
            return

        x = self.vehicle_x
        y = self.vehicle_y
        yaw = self.vehicle_yaw
        v = max(self.vehicle_v, 0.01)

        # --- FIND CLOSEST SEGMENT (different from PP) ---
        i = self.find_closest_segment_idx(x, y)

        x1, y1 = self.path[i]
        x2, y2 = self.path[i + 1]

        # Publish closest point (same as PP)
        cp_msg = PointStamped()
        cp_msg.header.frame_id = "world"
        cp_msg.point.x = x1
        cp_msg.point.y = y1
        self.closest_point_pub.publish(cp_msg)

        # --- CROSS-TRACK ERROR (new in Stanley) ---
        e_ct = self.compute_cross_track_error(i, x, y)

        # --- HEADING ERROR (similar to your get_alpha but uses path tangent) ---
        path_heading = math.atan2(y2 - y1, x2 - x1)
        heading_error = self.normalize_angle(path_heading - yaw)

        # --- STANLEY CONTROL LAW ---
        """
        DIFFERENCE FROM PURE PURSUIT:
        Pure Pursuit uses curvature = 2 sin(alpha) / ld 
        Stanley uses:
        steering = heading_error + atan(k * crosstrack / v)
        """
        cross_track_term = math.atan2(self.k * e_ct, v)
        steering_angle = heading_error + cross_track_term

        # Publish curvature (optional but matches your structure)
        self.curvature_pub.publish(Float64(data=steering_angle))

        # Convert steering → angular velocity (same as PP)
        angular_velocity = (v / self.L) * math.tan(steering_angle)

        # Publish command (same structure)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)

        # Publish car point (same as PP)
        car_msg = PointStamped()
        car_msg.header.frame_id = "world"
        car_msg.point.x = x
        car_msg.point.y = y
        self.car_point.publish(car_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = StanleyController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
