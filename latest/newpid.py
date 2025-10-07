import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
import time
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        #PID PARAMETERS (MATCH WITH PURE PURSUIT ALWWAYS)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.position_ref = None #Lookahead point from pure pursuit
        self.odom = None #Odometry of the vehicle
        self.integral_term = 0
        self.derivative_term = 0
        self.last_error = None
        self.last_time = self.get_clock().now() #will be when program starts

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_v = 0.0

        # Subscribe to Odometry and Velcoity Reference Topic from Pure Pursuit
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback,10)
        self.lap_sub = self.create_subscription(PointStamped, '/lookahead_point', self.lap_callback,10 )
        self.v_ref_sub = self.create_subscription(Float64, '/v_ref', self.vref_callback,10)

        # Publish
        self.pid_pub = self.create_publisher(Twist, '/pid_cmd_vel',10)

        #Timer
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("PID Controller Node Activated")

    def vref_callback(self, msg):
        self.v_ref = msg.data #Extract the velocity reference from /v_ref topic

    def lap_callback(self, msg):
        self.position_ref = (msg.point.x, msg.point.y) #Extract the lookahead point from /lookahead_point topic

    def odom_callback(self, odom_msg: Odometry):
        self.vehicle_x = odom_msg.pose.pose.position.x
        self.vehicle_y = odom_msg.pose.pose.position.y
        self.vehicle_v = odom_msg.twist.twist.linear.x

    def compute_perror(self):
        if self.position_ref is None or self.odom is None:
            return None
        
        x_ref, y_ref = self.position_ref
        pos = self.odom.pose.pose.position
        x = pos.x
        y = pos.y
        error = math.hypot(x_ref - x, y_ref - y) #Euclidean distance between the two points
        return error

    def control_loop(self):
        #Need to get dt for our calculations within the loop
        #Get current time and store within variable
        current_time = self.get_clock().now()
        #Compute dt by taking current time minus the last time and converted into seconds
        dt = (current_time - self.last_time).nanoseconds / 1e9 # seconds
        #Update last time to present time
        self.last_time = current_time

        #Defined error below from our class slides
        error = self.compute_perror()
        if error is None or dt <= 0.0:
            return  # wait until we have valid data
        #Defined the integral term of our control output
        self.integral_term += error * dt

        #If last error is not Null, we set the derivative term as such
        if self.last_error is not None:
            self.derivative_term = (error - self.last_error)/dt
        #If the last error is null, derivative term is just 0.0 coming from default state above
        #Update the error as last error
        self.last_error = error

        #Put all terms needed into the output variable
        output = self.kp * error + self.ki * self.integral_term + self.derivative_term *self.kd

        self.last_error = error
        self.last_time = current_time

        movement = Twist()
        movement.linear.x = min(output, self.v_ref) #clamping since we are not expecting to do reversal
        movement.angular.z = 0.0 #comes from Pure Pursuit
        self.pid_pub.publish(movement) #publish 

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()