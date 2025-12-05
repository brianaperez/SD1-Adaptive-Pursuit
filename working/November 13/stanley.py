import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseArray
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float64
import math


class StanleyController(Node):
    def __init__(self):
        super().__init__('stanley_controller')

        # ------------------- PARAMETERS (MATCH PURE PURSUIT) -------------------
        self.declare_parameter('wheelbase', 1.8)
        self.declare_parameter('lookahead_min', 11.0)
        self.declare_parameter('lookahead_gain', 3.0)
        self.declare_parameter('stanley_gain', 1.8)     # NEW for Stanley

        self.L = self.get_parameter('wheelbase').value
        self.ld_min = self.get_parameter('lookahead_min').value
        self.kv = self.get_parameter('lookahead_gain').value
        self.k = self.get_parameter('stanley_gain').value

        # ------------------- STATE VARIABLES (MATCH PURE PURSUIT) -------------------
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_yaw = 0.0
        self.vehicle_v = 0.0

        self.path = []
        self.wp_idx = 0
        self.lookahead_point = None

        # ------------------- SUBSCRIBERS (MATCH PURE PURSUIT) -------------------
        self.create_subscription(Path, '/waypoints', self.path_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/pose_info', self.pose_callback, 10)

        # ------------------- PUBLISHERS (MATCH PURE PURSUIT) -------------------
        self.curvature_pub = self.create_publisher(Float64, '/curvature', 10)
        self.lookahead_pub = self.create_publisher(PointStamped, '/lookahead_point', 10)
        self.closest_point_pub = self.create_publisher(PointStamped, '/closest_point', 10)
        self.car_point = self.create_publisher(PointStamped, '/car_point', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/stanley_cmd_vel', 10)

        # ------------------- TIMER LOOP (MATCH PURE PURSUIT) -------------------
        self.create_timer(0.05, self.stanley_callback)


    # ================================
    # CALLBACKS (MATCH PURE PURSUIT)
    # ================================
    def path_callback(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def odom_callback(self, msg):
        self.vehicle_v = msg.twist.twist.linear.x

    def pose_callback(self, msg):
        pose = msg.poses[1]
        self.vehicle_x = pose.position.x
        self.vehicle_y = pose.position.y
        self.vehicle_yaw = self.euler_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

    # SAME FUNCTION AS PURE PURSUIT
    def euler_from_quaternion(self, x, y, z, w):
        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2*(y*y + z*z)
        return math.atan2(siny_cosp, cosy_cosp)

    # SAME FUNCTION AS PURE PURSUIT
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))


    # ============================================================
    # PURE PURSUIT FUNCTIONS: Closest waypoint + Lookahead
    # (UNCHANGED, EXACTLY AS YOU USE THEM)
    # ============================================================

    def find_closest_waypoint_idx(self, x, y, yaw):
        close_dist = float('inf')
        close_idx = 0

        for i in range(self.wp_idx, len(self.path)):
            wp_x, wp_y = self.path[i]
            dist = math.hypot(wp_x - x, wp_y - y)

            dx = wp_x - x
            dy = wp_y - y
            dot = dx * math.cos(yaw) + dy * math.sin(yaw)

            if dot < 0:
                continue

            if dist < close_dist:
                close_dist = dist
                close_idx = i

        return close_idx

    def find_lookaheadpoint(self, x, y, ld, future_wp):
        for wp in future_wp:
            dx = wp[0] - x
            dy = wp[1] - y
            if math.hypot(dx, dy) >= ld:
                return wp
        return future_wp[-1]


    # ============================================================
    # STANLEY CROSS-TRACK ERROR (NEW, clearly marked)
    # ============================================================

    def compute_cross_track_error(self, x, y, x1, y1, x2, y2):
        """
        --- DIFFERENCE FROM PURE PURSUIT ---
        Pure Pursuit does not compute cross-track error.
        This uses the signed perpendicular distance formula.
        """

        dx_path = x2 - x1
        dy_path = y2 - y1
        dx = x - x1
        dy = y - y1

        numerator = dx * dy_path - dy * dx_path
        denom = math.hypot(dx_path, dy_path)
        if denom < 1e-6:
            return 0.0

        return numerator / denom


    # ============================================================
    # MAIN CONTROL LOOP — LOOKAHEAD STANLEY VERSION
    # ============================================================

    def stanley_callback(self):
        if not self.path:
            return

        x = self.vehicle_x
        y = self.vehicle_y
        yaw = self.vehicle_yaw
        v = max(self.vehicle_v, 0.01)

        # ------------------- STEP 1: SAME AS PURE PURSUIT -------------------
        ld = max(self.ld_min, self.kv * v)

        self.wp_idx = self.find_closest_waypoint_idx(x, y, yaw)
        future_wp = self.path[self.wp_idx:]

        self.lookahead_point = self.find_lookaheadpoint(x, y, ld, future_wp)
        lap_x, lap_y = self.lookahead_point

        # Publish lookahead point
        msg = PointStamped()
        msg.header.frame_id = "world"
        msg.point.x = lap_x
        msg.point.y = lap_y
        self.lookahead_pub.publish(msg)

        # Publish car point
        car_msg = PointStamped()
        car_msg.header.frame_id = "world"
        car_msg.point.x = x
        car_msg.point.y = y
        self.car_point.publish(car_msg)

        # ------------------- STEP 2: Segment behind lookahead -------------------
        """
        --- DIFFERENCE FROM PURE PURSUIT ---
        Pure Pursuit does NOT use a segment.
        Stanley needs a path segment to compute heading + cross-track error.
        """

        try:
            lap_idx = self.path.index((lap_x, lap_y))
        except ValueError:
            return  # Shouldn't happen unless broken path

        prev_idx = max(0, lap_idx - 1)

        x1, y1 = self.path[prev_idx]
        x2, y2 = self.path[lap_idx]

        # RViz marker for closest segment point
        cp_msg = PointStamped()
        cp_msg.header.frame_id = "world"
        cp_msg.point.x = x1
        cp_msg.point.y = y1
        self.closest_point_pub.publish(cp_msg)

        # ------------------- STEP 3: Stanley Errors (NEW) -------------------

        # Heading of the path segment
        path_heading = math.atan2(y2 - y1, x2 - x1)
        heading_error = self.normalize_angle(path_heading - yaw)

        # Cross-track error
        e_ct = self.compute_cross_track_error(x, y, x1, y1, x2, y2)

        # Stanley correction term
        stanley_term = math.atan2(self.k * e_ct, v)

        # Steering angle (Stanley formula)
        steering_angle = heading_error + stanley_term

        # Publish curvature for debugging (just like PP)
        self.curvature_pub.publish(Float64(data=steering_angle))

        # Convert steering to angular velocity
        angular_velocity = (v / self.L) * math.tan(steering_angle)

        # ------------------- STEP 4: Publish Twist (MATCH PURE PURSUIT) -------------------
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    controller = StanleyController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
