import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Path, Odometry
#import matplotlib.pyplot as plt
#from visualization_msgs.msg import Marker # Import for marker messages
import math
import time

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.error = 0
        self.current_error = 0
        self.track_x = 0.0
        self.track_y = 0.0
        '''
        # For graph
        self.data = []
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Error Distance (m)')
        self.ax.set_title('PID Error vs Time')
        '''
        # PID parameters - tuned based on notes (kp=0.1, ki=1 was most desirable)
        self.Kp = 10.0       # Proportional gain
        self.Ki = 1.0       # Integral gain  
        self.Kd = 5.0       # Derivative gain (from notes: kd=5 reduces oscillations)
        
        # Anti-windup parameters
        self.I_max = 100.0   # Maximum integral term to prevent windup
        self.output_max = 5.0  # Maximum control output (speed limit)
        self.output_min = -5.0 # Minimum control output
        
        # PID state variables
        self.current_time = time.time()
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
        #self.marker_publisher_ = self.create_publisher(Marker, '/waypoint', 10)

        '''
        # Publish marker for visualization (Vehicle's position)
        marker = Marker()
        marker.header.frame_id = "map"  # Change to your frame ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.waypoint[0]
        marker.pose.position.y = self.waypoint[1]
        marker.pose.position.z = self.waypoint[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.0
        marker.scale.x = 5.0  # Adjust the scale as needed
        marker.scale.y = 5.0
        marker.scale.z = 5.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker = marker

        self.logger.info(f"PID Controller node started - Kp:{self.Kp}, Ki:{self.Ki}, Kd:{self.Kd}")
        '''
    def tracking_point_callback(self, msg):
        self.track_x = msg.point.x
        self.track_y = msg.point.y

    def odom_callback(self, msg):
        current_pose = msg.pose.pose.position
        current_x = current_pose.x
        current_y = current_pose.y

        # Calculate error distance to waypoint
        self.error = self.calc_error((self.track_x, self.track_y), (current_x, current_y))
        #self.data.append(self.error)
        #self.update_plot()
        #self.marker_publisher_.publish(self.marker)
        
        # Update timing for PID calculation
        self.previous_time = self.current_time
        self.current_time = time.time()
        self.dt = self.current_time - self.previous_time
        
        # Ensure minimum sampling time to avoid division issues
        if self.dt < 0.001:
            self.dt = 0.001
            
        # Calculate and publish cmd_vel message
        self.generate_control_output()

    def calc_error(self, desired_point, actual_position):
        """Calculate signed error distance to waypoint"""
        x_diff = desired_point[0] - actual_position[0]
        y_diff = desired_point[1] - actual_position[1]
        error_length = math.sqrt(x_diff ** 2 + y_diff ** 2)
        
        # Determine sign based on whether we're approaching the waypoint
        # Positive error means we need to move forward, negative means we've overshot
        error_sign = 1 if error_length > 1.0 else -1  # Simple threshold-based sign
        return error_length * error_sign
    '''
    def update_plot(self):
        """Update real-time plot of error"""
        self.line.set_data(range(len(self.data)), self.data)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)  # Pause briefly to allow the plot to update
    '''
    def generate_control_output(self):
        
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
        
        # Log PID components for debugging
        #self.logger.info(f"Error: {self.current_error:.2f}m | "
                      #  f"P: {self.P:.2f} | I: {self.I:.2f} | D: {self.D:.2f} | "
                     #   f"Output: {control_output:.2f} | Speed: {speed:.2f}")

        # Publish cmd_vel message
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0  # No angular velocity for simple point-to-point control
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = PurePursuitController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
message.txt
9 KB
