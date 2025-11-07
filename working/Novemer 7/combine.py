import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelCombiner(Node):
    def __init__(self):
        super().__init__('cmd_vel_combiner')

        #Class Variables for cmd commands for steering control and robot movement
        self.pure_pursuit_cmd = None
        self.pid_cmd = None

        #Create Subscribers to the Pure Pursuit Steering Commands
        #They will have linear movement set to 0.0 and only contain steering angles
        self.pp_sub = self.create_subscription(
            Twist,
            '/pp_cmd_vel',
            self.pp_callback,
            10)

        #Create Subscribers to the PID Controller Commands
        #They will set the speed of the car and set its angles to 0.0
        self.pid_sub = self.create_subscription(
            Twist,
            '/pid_cmd_vel',
            self.pid_callback,
            10)

        #Create publisher for sending commands to the car itself
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        #Timer to call function timer_callback at a certain rate to determine robot movement and steering in one line at a time
        self.timer = self.create_timer(0.1, self.timer_callback)  # 20 Hz

    #Callback function to set the msg from pure_pursuit_cmd_vel to the class variables set in the beginning
    def pp_callback(self, msg):
        self.pure_pursuit_cmd = msg
        #self.get_logger().info(f"Recieved angle {msg.angular.z}")

    #Callback function to set the msg from pid_cmd_vel to the class variable set in the beginning
    def pid_callback(self, msg):
        self.pid_cmd = msg
        #self.get_logger().info(f"Recieved Speed {msg.linear.x}")

    #Timer_callback function that will be sending command to the robot
    def timer_callback(self):
        #No commands will be set unless we have both steering and speed commands together
        if self.pure_pursuit_cmd is None or self.pid_cmd is None:
            return  # wait for both commands

        #Create a new twist message
        combined = Twist()
        #Extract only the PID controller linear.x value only
        combined.linear.x = self.pid_cmd.linear.x  # longitudinal from PID
        #combined.linear.x = 3.0 #Constant Velocity
        #Extract only the steering angle from Pure Pursuit Controller angular.z only
        #combined.angular.z = self.pure_pursuit_cmd.angular.z  # steering from Pure Pursuit
        combined.angular.z = 0.0 #locked steering for testing

        #combined now contains both steering angle and velocity for the robot
        #publish to the /cmd_vel topic to be sent for the robot
        #self.get_logger().info(f"Yaw Rate (r/s): {combined.angular.z:.3f} | V: {combined.linear.x:.2f}")
        self.cmd_vel_pub.publish(combined)


def main(args=None):
    rclpy.init(args=args)
    combiner = CmdVelCombiner()
    rclpy.spin(combiner)
    combiner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
