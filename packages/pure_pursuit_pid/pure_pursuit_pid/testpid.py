import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Path, Odometry
#import matplotlib.pyplot as plt
#from visualization_msgs.msg import Marker # Import for marker messages
import math
import time

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.error = 0
        self.track_x = 0.0
        self.track_y = 0.0
        
        # PID parameters - tuned based on notes (kp=0.1, ki=1 was most desirable)
        self.Kp = 3.0       # Proportional gain
        self.Ki = 0.1      # Integral gain  
        self.Kd = 5.0      # Derivative gain (from notes: kd=5 reduces oscillations)
        
        # Anti-windup parameters
        self.I_max = 10.0   # Maximum integral term to prevent windup
        self.output_max = 20.0  # Maximum control output (speed limit)
        self.output_min = -6.0 # Minimum control output
        
        # PID state variables
        now = self.get_clock().now().seconds_nanoseconds()
        self.current_time = now[0] + now[1] * 1e-9
        self.previous_time = self.current_time
        self.current_error = 0
        self.previous_error = 0
        self.P = 0
        self.I = 0  # Integral term accumulator
        self.D = 0
        
        # Sampling time for discrete implementation
        self.dt = 0.1  # Expected sampling time (100ms)

        # Subscribe to /odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.track_point_sub = self.create_subscription(
            PointStamped, 
            '/track_point',
            self.tracking_point_callback,
            10
        )

        # Publisher for cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist,
            '/pid_cmd_vel',
            10
        )
        
    def tracking_point_callback(self, msg):
        self.track_x = msg.point.x
        self.track_y = msg.point.y
    
    def euler_from_quaternion(self, x, y, z, w):
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def odom_callback(self, msg):
        current_pose = msg.pose.pose.position
        current_x = current_pose.x
        current_y = current_pose.y
        
        q = msg.pose.pose.orientation
        yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)  # ψ_veh in the formula

        dx = self.track_x - current_x
        dy = self.track_y - current_y

        # Calculate error distance to waypoint
        self.error = self.calc_error((self.track_x, self.track_y), (current_x, current_y), yaw)
        #self.data.append(self.error)
        #self.update_plot()
        #self.marker_publisher_.publish(self.marker)
        
        # Update timing for PID calculation
        self.previous_time = self.current_time
        now = self.get_clock().now().seconds_nanoseconds()
        self.current_time = now[0] + now[1] * 1e-9
        self.dt = self.current_time - self.previous_time
        
        # Ensure minimum sampling time to avoid division issues
        if self.dt < 0.001:
            self.dt = 0.001
            
        # Calculate and publish cmd_vel message
        self.generate_control_output()

    def calc_error(self, desired_point, actual_position, yaw):
        """Calculate signed error distance to waypoint"""
        x_diff = desired_point[0] - actual_position[0]
        y_diff = desired_point[1] - actual_position[1]
        error_length = math.sqrt(x_diff ** 2 + y_diff ** 2)
        
        heading_vector = (math.cos(yaw),math.sin(yaw))
        error_vector = (x_diff, y_diff)
        dot_product = (heading_vector[0] * error_vector[0] + heading_vector[1] * error_vector[1])
                          
        # Determine sign based on whether we're approaching the waypoint
        # Positive error means we need to move forward, negative means we've overshot
        error_sign = 1 if dot_product > 0 else -1  # Simple threshold-based sign
        return error_length * error_sign
    
    def generate_control_output(self):
        """
        PID Controller implementation based on notes:
        u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
        
        Discrete form:
        P = Kp * error
        I = I_prev + Ki * error * dt  (with anti-windup)
        D = Kd * (error - error_prev) / dt
        u = P + I + D
        """
        
        # Update error terms
        self.previous_error = self.current_error
        self.current_error = self.error
        
        # Calculate PID terms
        # Proportional term: Kp * e(t)
        self.P = self.Kp * self.current_error
        
        # Integral term: Ki * ∫e(t)dt with anti-windup
        # Using trapezoidal integration: I += Ki * (e_curr + e_prev) * dt / 2
        integral_increment = self.Ki * (self.current_error + self.previous_error) * self.dt / 2.0
        self.I += integral_increment
        
        # Anti-windup: Clamp integral term to prevent windup
        if self.I > self.I_max:
            self.I = self.I_max
        elif self.I < -self.I_max:
            self.I = -self.I_max
            
        # Derivative term: Kd * de(t)/dt
        self.D = self.Kd * (self.current_error - self.previous_error) / self.dt
        
        # Calculate total control output
        control_output = self.P + self.I + self.D
        
        # Apply output saturation to prevent excessive speeds
        if control_output > self.output_max:
            # Additional anti-windup: reduce integral if output is saturated
            excess = control_output - self.output_max
            self.I -= excess * 0.1  # Back-calculate to reduce windup
            control_output = self.output_max
        elif control_output < self.output_min:
            excess = self.output_min - control_output
            self.I += excess * 0.1  # Back-calculate to reduce windup
            control_output = self.output_min
  
        # Convert control output to speed (always move forward toward waypoint)
        speed = abs(control_output)  # Take absolute value for forward motion
        
        # Stop if very close to waypoint (within 0.5m)
        if abs(self.current_error) < 0.5:
            speed = 0.0
            #self.logger.info("Reached waypoint! Stopping.")

        #speed = control_output

        # Log PID components for debugging
        #self.get_logger().info(f"Error: {self.current_error:.2f}m | "
        #               f"P: {self.P:.2f} | I: {self.I:.2f} | D: {self.D:.2f} | "
         #              f"Output: {control_output:.2f} | Speed: {speed:.2f}")
        
        # Publish cmd_vel message
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0  # No angular velocity for simple point-to-point control
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()