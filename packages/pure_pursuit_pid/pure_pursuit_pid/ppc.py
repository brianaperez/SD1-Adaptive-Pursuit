import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
from messages.msg import VehicleState # Import the custom message type for vehicle state
import math

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.logger = self.get_logger()
        self.waypoints = []

        # Vehicle Parameters
        self.wheelbase = 1.8  # L in the formula

        # Tunable parameters
        self.k = 1.0
        #self.linear_velocity = 3.0
        self.lookahead_distance = 9.0  # l_d in the formula

        # Subscribe to /gazebo/vehicle_state topic
        self.subscription = self.create_subscription(
            VehicleState,
            '/gazebo/vehicle_state',
            self.pose_callback,
            10)

        # Subscribe to the path published by WaypointPublisher node
        self.path_subscriber = self.create_subscription(
            Path,
            '/vehicle_waypoints',
            self.path_callback,
            10)

        # Publisher for cmd_vel topic
        #self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pure_pursuit_cmd_vel = self.create_publisher(Twist, '/pure_pursuit_cmd_vel', 10)
        self.logger.info("Pure Pursuit Controller node started")

    def path_callback(self, path_msg):
        # Update the waypoints when a new path is received
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in path_msg.poses]
        self.logger.info(f"Received path with {len(self.waypoints)} waypoints")

    def pose_callback(self, msg):
        if not self.waypoints:
            # If no waypoints available, do nothing
            return

        current_pose = msg.front_axle
        current_x = current_pose.x
        current_y = current_pose.y
        yaw = msg.yaw  # ψ_veh in the formula

        # Calculate and publish cmd_vel message
        steering_angle = self.generate_control_output(current_x, current_y, yaw)
        
        # Publish cmd_vel message
        twist = Twist()
        #twist.linear.x = self.linear_velocity  # constant linear velocity
        twist.linear.x = 0.0
        twist.angular.z = self.steering_angle_to_angular_velocity(steering_angle)
        #self.cmd_vel_publisher.publish(twist)
        self.pure_pursuit_cmd_vel.publish(twist)
        
        # Debug logging
        self.logger.info(f"Vehicle pose: ({current_x:.2f}, {current_y:.2f}, {math.degrees(yaw):.1f}°) | "
                        f"Steering angle: {math.degrees(steering_angle):.1f}°")

    def distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def steering_angle_to_angular_velocity(self, steering_angle):
        angular_velocity = math.tan(steering_angle) * self.linear_velocity / self.wheelbase
        return angular_velocity

    def is_waypoint_ahead(self, waypoint_x, waypoint_y, vehicle_x, vehicle_y, vehicle_yaw):
        """Check if a waypoint is ahead of the vehicle in its current heading direction."""
        # Transform waypoint to vehicle's local coordinate frame
        dx = waypoint_x - vehicle_x
        dy = waypoint_y - vehicle_y
        
        # Rotate to vehicle frame
        local_x = math.cos(-vehicle_yaw) * dx - math.sin(-vehicle_yaw) * dy
        
        # Waypoint is ahead if local_x > 0
        return local_x > 0

    def find_lookahead_point(self, current_x, current_y, yaw):
        """Find the target waypoint at the lookahead distance."""
        # Filter waypoints that are ahead of the vehicle
        waypoints_ahead = [
            wp for wp in self.waypoints 
            if self.is_waypoint_ahead(wp[0], wp[1], current_x, current_y, yaw)
        ]
        
        if not waypoints_ahead:
            # If no waypoints ahead, use the last waypoint
            if self.waypoints:
                return self.waypoints[-1]
            else:
                return None
        
        # Find the waypoint closest to the lookahead distance
        target_waypoint = None
        min_distance_diff = float('inf')
        
        for wp in waypoints_ahead:
            dist = self.distance((current_x, current_y), wp)
            distance_diff = abs(dist - self.lookahead_distance)
            
            if distance_diff < min_distance_diff:
                min_distance_diff = distance_diff
                target_waypoint = wp
        
        # If no waypoint is close to lookahead distance, find first waypoint beyond it
        if target_waypoint is None:
            for wp in waypoints_ahead:
                dist = self.distance((current_x, current_y), wp)
                if dist >= self.lookahead_distance:
                    return wp
            
            # If all waypoints are closer than lookahead distance, use the farthest one
            target_waypoint = max(waypoints_ahead, 
                                key=lambda wp: self.distance((current_x, current_y), wp))
        
        return target_waypoint

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
        target_waypoint = self.find_lookahead_point(current_x, current_y, yaw)
        
        if target_waypoint is None:
            self.logger.warn("No target waypoint found")
            return 0.0
        
        x_way, y_way = target_waypoint
        
        dx = x_way - current_x  # X_way - X_veh
        dy = y_way - current_y  # Y_way - Y_veh
        
        if abs(dx) < 1e-6:
            angle_to_waypoint = math.pi/2 if dy > 0 else -math.pi/2
        else:
            angle_to_waypoint = math.atan2(dy, dx)  # arctan((Y_way - Y_veh)/(X_way - X_veh))
        
        alpha = angle_to_waypoint - yaw  # α = arctan((Y_way - Y_veh)/(X_way - X_veh)) - ψ_veh
        
        alpha = self.normalize_angle(alpha)
        
        # δ = arctan(2L*sin(α)/l_d)
        steering_angle = math.atan(2 * self.wheelbase * math.sin(alpha) / self.lookahead_distance)
        
        # Log debug information
        self.logger.info(f"Target waypoint: ({x_way:.2f}, {y_way:.2f}) | "
                        f"α = {math.degrees(alpha):.1f}° | "
                        f"δ = {math.degrees(steering_angle):.1f}°")
        
        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()