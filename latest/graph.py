import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

class VelocityPlotter(Node):
    def __init__(self):
        super().__init__('velocity_plotter')

        # ROS 2 Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.v_ref_sub = self.create_subscription(Float64, '/v_ref', self.vref_callback, 10)

        # Buffers
        self.max_len = 300  # 30 seconds @ 10Hz
        self.time_stamps = deque(maxlen=self.max_len)
        self.actual_velocities = deque(maxlen=self.max_len)
        self.target_velocities = deque(maxlen=self.max_len)
        self.velocity_ratios = deque(maxlen=self.max_len)

        # State
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.time = 0.0

        # Timer to record data
        self.timer = self.create_timer(0.1, self.record_data)  # 10 Hz

        # Start the plot
        self.init_plot()

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x

    def vref_callback(self, msg):
        self.target_velocity = msg.data

    def record_data(self):
        self.time += 0.1
        self.time_stamps.append(self.time)
        self.actual_velocities.append(self.current_velocity)
        self.target_velocities.append(self.target_velocity)

        # Avoid divide by zero — clamp small velocities to prevent blow-up
        if abs(self.current_velocity) < 1e-4:
            ratio = 0.0
        else:
            ratio = self.target_velocity / self.current_velocity

        self.velocity_ratios.append(ratio)

    def init_plot(self):
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        # Plot 1: Velocities
        self.line_actual, = self.ax1.plot([], [], label='Actual Velocity', color='blue')
        self.line_target, = self.ax1.plot([], [], label='Target Velocity (v_ref)', color='green')
        self.ax1.set_ylabel("Velocity (m/s)")
        self.ax1.set_title("Actual vs Target Velocity")
        self.ax1.legend()
        self.ax1.grid(True)

        # Plot 2: Velocity Ratio
        self.line_ratio, = self.ax2.plot([], [], label='v_ref / v', color='red')
        self.ax2.set_xlabel("Time (s)")
        self.ax2.set_ylabel("Velocity Ratio")
        self.ax2.set_title("Tracking Ratio (v_ref / v)")
        self.ax2.legend()
        self.ax2.grid(True)
        self.ax2.set_ylim(0, 2)  # Adjust as needed

        # Animate
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.tight_layout()
        plt.show()

    def update_plot(self, frame):
        # Update velocity lines
        self.line_actual.set_data(self.time_stamps, self.actual_velocities)
        self.line_target.set_data(self.time_stamps, self.target_velocities)

        # Update ratio line
        self.line_ratio.set_data(self.time_stamps, self.velocity_ratios)

        # Dynamically adjust X axis
        if self.time_stamps:
            self.ax1.set_xlim(max(0, self.time_stamps[0]), self.time_stamps[-1] + 1)

        return self.line_actual, self.line_target, self.line_ratio


def main(args=None):
    rclpy.init(args=args)
    node = VelocityPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down velocity plotter...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
