import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        # Vehicle Parameters
        self.wheelbase = 1.8  # L in the formula

        self.track_x = 0.0
        self.track_y = 0.0
        self.lookahead_distance = 3.0 #set as the minimum lookahead distrance param
        self.linear_velocity = 0.0

        self.has_lookahead = False
        self.has_track_point = False

        # Subscribe to the lookahead distance value published by waypoint publisher
        self.lookahead_distance_sub = self.create_subscription(
            Float64,
            '/lookahead_distance',
            self.lookahead_distance_callback,
            10
        )

        # Subscribe to /odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Subscribe to the tracking point published by waypoint publisher
        self.path_subscriber = self.create_subscription(
            PointStamped, 
            '/track_point',
            self.tracking_point_callback,
            10
        )

        # Publisher for pure_pursuit_cmd_vel topic
        self.pure_pursuit_cmd_vel = self.create_publisher(Twist,
            '/pure_pursuit_cmd_vel',
            10)
        self.get_logger().info("Pure Pursuit Controller node started")

    def lookahead_distance_callback(self, msg):
        self.lookahead_distance = msg.data
        self.has_lookahead = True
        
    def tracking_point_callback(self, msg):
        self.track_x = msg.point.x
        self.track_y = msg.point.y
        self.has_track_point = True

    def euler_from_quaternion(self, x, y, z, w):
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

    def odom_callback(self, msg):
        #Ensures we have recieve the lookahead and tracking point from waypoint logic
        if not(self.has_lookahead and self.has_track_point):
            self.get_logger().info("Wating for lookahead and tracking point")
            return
        
        self.linear_velocity = msg.twist.twist.linear.x
        current_pose = msg.pose.pose.position
        current_x = current_pose.x
        current_y = current_pose.y
        
        #self.get_logger().info(f"Linear V {self.linear_velocity}")
        q = msg.pose.pose.orientation
        yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)  # ψ_veh in the formula

        # Calculate and publish message
        steering_angle = self.generate_control_output(current_x, current_y, yaw)
        
        # Publish message
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.steering_angle_to_angular_velocity(steering_angle)
        #self.get_logger().info(f"Angular z = {self.steering_angle_to_angular_velocity(steering_angle)}")
        self.pure_pursuit_cmd_vel.publish(twist)
        
        # Debug logging
        #self.logger.info(f"Vehicle pose: ({current_x:.2f}, {current_y:.2f}, {math.degrees(yaw):.1f}°) | "
                       # f"Steering angle: {math.degrees(steering_angle):.1f}°")

    def steering_angle_to_angular_velocity(self, steering_angle):
        angular_velocity = math.tan(steering_angle) * self.linear_velocity / self.wheelbase
        #self.get_logger().info(f"Linear Velocity: {self.linear_velocity:.2f}")
        return angular_velocity

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def generate_control_output(self, current_x, current_y, yaw):
        """
        Calculates the steering angle required for path following using Pure Pursuit algorithm.
        
        Implements:
        α = arctan((Y_way - Y_veh)/(X_way - X_veh)) - ψ_veh  
        δ = arctan(2L*sin(α)/l_d)
        
        Where:
        - α is the angular error between vehicle heading and direction to target waypoint
        - δ is the steering angle for the kinematic bicycle model
        - L is the wheelbase
        - l_d is the lookahead distance
        - ψ_veh is the vehicle's current yaw angle

        Args:
            current_x (float): Current x-coordinate of the vehicle (X_veh).
            current_y (float): Current y-coordinate of the vehicle (Y_veh).
            yaw (float): Current yaw angle (orientation) of the vehicle (ψ_veh).

        Returns:
            float: Steering angle in radians (δ).
        """
        # Find the target waypoint at lookahead distance
        dx = self.track_x - current_x  # X_way - X_veh
        dy = self.track_y - current_y  # Y_way - Y_veh

        angle_to_waypoint = math.atan2(dy, dx)
        alpha = angle_to_waypoint - yaw
        normal_alpha = self.normalize_angle(alpha)

        #self.get_logger().info(f"Lookahead Distance:{self.lookahead_distance:.2f}")
        curvature = (2 * self.wheelbase * math.sin(normal_alpha)) / self.lookahead_distance
        steering_angle = math.atan(curvature)

        #self.get_logger().info(f"δ = {math.degrees(steering_angle):.2f}°")
        '''
        # Log debug information
        '''
        #self.get_logger().info(f"Target waypoint: ({self.track_x:.2f}, {self.track_y:.2f}) | "
        #                f"α = {math.degrees(alpha):.1f}° | "
        #                f"δ = {math.degrees(steering_angle):.1f}°")
        
        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()