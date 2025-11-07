import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped, PoseArray
from nav_msgs.msg import Path
from std_msgs.msg import Float64
import math
import time

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.error = 0
        self.current_error = 0
        self.track_x = 0.0
        self.track_y = 0.0
        self.closest_point_x = 0.0
        self.closest_point_y = 0.0
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0

        # PID parameters - tuned based on notes
        self.Kp = 1.0   # Proportional gain
        self.Ki = 1.0       # Integral gain  
        self.Kd = 0.0   # Derivative gain
        
        # Anti-windup parameters
        self.I_max = 100.0   # Maximum integral term to prevent windup
        self.output_max = 6.25  # Maximum control output (speed limit)
        self.output_min = -6.25 # Minimum control output
        
        # PID state variables
        self.current_time = time.time()
        self.previous_time = self.current_time
        self.current_error = 0
        self.previous_error = 0
        self.P = 0
        self.I = 0  # Integral term accumulator
        self.D = 0
        self.output_max = 6.25
        self.output_min = 0.0
        
        # Sampling time for discrete implementation
        self.dt = 0.1  # Expected sampling time (100ms)

        # Subscribe to /lookahead_point topic
        self.track_point_sub = self.create_subscription(
            PointStamped, 
            '/lookahead_point',
            self.tracking_point_callback,
            10
        )

        self.subscription = self.create_subscription(
            PoseArray,
            '/pose_info',
            self.pose_callback,
            10)

        # Publisher for cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist,
            '/pid_cmd_vel',
            10
        )

        self.dt_publisher = self.create_publisher(Float64,
            '/pid_dt',
            10
        )

        self.cp_subscription = self.create_subscription(PointStamped, '/closest_point', self.closest_point_callback, 10)

        self.error_publisher = self.create_publisher(PointStamped, '/error', 10)
        self.timer = self.create_timer(0.1, self.generate_control_output)  # 10 Hz
    
    def closest_point_callback(self, msg):
        self.closest_point_x = msg.point.x
        self.closest_point_y = msg.point.y
        
    def tracking_point_callback(self, msg):
        self.track_x = msg.point.x
        self.track_y = msg.point.y
    
    def pose_callback(self, msg):
        pose = msg.poses[1] #Car position stored here
        self.vehicle_x = pose.position.x
        self.vehicle_y = pose.position.y
    
    def calc_positional_error(self, lookahead_point, vehicle_pos):
        dx = lookahead_point[0] - vehicle_pos[0]
        dy = lookahead_point[1] - vehicle_pos[1]
        error = math.sqrt(dx**2 + dy**2)
        return error
    
    def generate_control_output(self):
        if self.closest_point_x == 0.0 or self.closest_point_y == 0.0:
            return  # Wait until we have valid vehicle position

        current_x = self.vehicle_x #should be updated from pose_callback
        current_y = self.vehicle_y #should be updated from pose_callback
        vehicle_pos = (current_x, current_y)
        lookahead_point = (self.track_x, self.track_y)

        self.error = self.calc_positional_error(lookahead_point, vehicle_pos)
        # Update error terms
        self.previous_error = self.current_error
        self.current_error = self.error       

        error_msg = PointStamped()
        error_msg.point.x = self.error
        error_msg.point.y = self.current_time
        self.error_publisher.publish(error_msg)
        
        # Update timing for PID calculation
        self.previous_time = self.current_time
        self.current_time = time.time()
        self.dt = self.current_time - self.previous_time
        self.dt = max(self.dt, 0.01)
        
        # Ensure minimum sampling time to avoid division issues
        #if self.dt < 0.001:
        #    self.dt = 0.001
            
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
        self.get_logger().info(f"P {self.P:.2f}= {self.Kp} * {self.current_error:.2f} |" 
                               f" I {self.I:.2f}"
                               f"D {self.D:.2f} = {self.Kd} * {self.current_error:.2f} - {self.previous_error:.2f} / {self.dt:.2f}")
        
        # Calculate total control output
        control_output = self.P + self.I + self.D
        
        # Apply output saturation to prevent excessive speeds
        # if control_output > self.output_max:
        #     # Additional anti-windup: reduce integral if output is saturated
        #     excess = control_output - self.output_max
        #     self.I -= excess * 0.1  # Back-calculate to reduce windup
        #     control_output = self.output_max
        # elif control_output < self.output_min:
        #     excess = self.output_min - control_output
        #     self.I += excess * 0.1  # Back-calculate to reduce windup
        #     control_output = self.output_min
            
        # Convert control output to speed (always move forward toward waypoint)
        speed = control_output  # Take absolute value for forward motion
        speed = max(min(speed, self.output_max), self.output_min)  # Clamp speed to [0, output_max]

        # Stop if very close to waypoint (within 0.5m)
        if abs(self.current_error) < 0.5:
            speed = 0.0
            self.logger.info("Reached waypoint! Stopping.")
        
        # Log PID components for debugging
        ##               f"P: {self.P:.2f} | I: {self.I:.2f} | D: {self.D:.2f} | "
         #               f"Output: {speed:.2f}")

        #self.get_logger().info(
        #    f"LAP: ({self.track_x:.2f}, {self.track_y:.2f}) "
        #    f"CAR: ({current_x:.2f}, {current_y:.2f}) "
        #    f"Error: {self.error:.3f} Speed: {speed:.2f}"
        #    f" | P: {self.P:.2f} | I: {self.I:.2f} | D: {self.D:.2f} | "
       # )
        dt_msg = Float64()
        dt_msg.data = self.current_time
        self.dt_publisher.publish(dt_msg)

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