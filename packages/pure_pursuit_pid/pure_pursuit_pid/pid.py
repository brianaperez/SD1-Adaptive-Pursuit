import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class PIDVelocityController(Node):
    def __init__(self):
        super().__init__('pid_velocity_controller')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(Twist,
            '/pid_cmd_vel',
            10
        )

        # PID coefficients
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.1

        self.target_velocity = 3.0  # m/s
        self.integral = 0.0
        self.previous_error = 0.0

        self.current_velocity = 0.0

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x

    def control_loop(self):
        error = self.target_velocity - self.current_velocity
        self.integral += error * 0.1
        derivative = (error - self.previous_error) / 0.1

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error

        cmd = Twist()
        cmd.angular.z = 0.0
        #cmd.linear.x = max(0.0, min(output, 3.0))  # Clamp to [0, 2.0]
        cmd.linear.x = 20.0

        self.publisher.publish(cmd)
        #self.get_logger().info(f"Current vel: {self.current_velocity:.2f} | Output: {output:.2f}")

def main():
    rclpy.init()
    node = PIDVelocityController()
    rclpy.spin(node)
    rclpy.shutdown()