import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import math
import threading
import tkinter as tk

class SimpleLiveGraph(Node):
    def __init__(self):
        super().__init__('simple_live_graph')

        # ROS2 subscribers
        self.create_subscription(PointStamped, '/car_point', self.car_callback, 10)
        self.create_subscription(PointStamped, '/lookahead_point', self.lookahead_callback, 10)
        self.create_subscription(Float64, '/v_ref', self.vref_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # State variables
        self.car_x = self.car_y = 0.0
        self.lookahead_x = self.lookahead_y = 0.0
        self.v_ref = self.v_actual = 0.0
        self.pos_err = self.v_err = 0.0

        # Start Tkinter GUI in a separate thread
        threading.Thread(target=self.start_gui, daemon=True).start()

        # Update every 100 ms
        self.create_timer(0.1, self.update_errors)

    def car_callback(self, msg):
        self.car_x = msg.point.x
        self.car_y = msg.point.y

    def lookahead_callback(self, msg):
        self.lookahead_x = msg.point.x
        self.lookahead_y = msg.point.y

    def vref_callback(self, msg):
        self.v_ref = msg.data

    def odom_callback(self, msg):
        self.v_actual = msg.twist.twist.linear.x

    def update_errors(self):
        dx = self.lookahead_x - self.car_x
        dy = self.lookahead_y - self.car_y
        self.pos_err = math.hypot(dx, dy)
        self.v_err = self.v_ref - self.v_actual

        # Update GUI labels
        if hasattr(self, 'label_pos'):
            self.label_pos.config(text=f"Position Error: {self.pos_err:.3f} m")
            self.label_vref.config(text=f"Velocity Ref: {self.v_ref:.3f} m/s")
            self.label_vact.config(text=f"Velocity Actual: {self.v_actual:.3f} m/s")
            self.label_verr.config(text=f"Velocity Error: {self.v_err:.3f} m/s")

    def start_gui(self):
        self.root = tk.Tk()
        self.root.title("Pure Pursuit + PID Live Monitor")
        self.root.geometry("400x200")

        self.label_pos = tk.Label(self.root, text="Position Error: 0.000", font=('Consolas', 14))
        self.label_vref = tk.Label(self.root, text="Velocity Ref: 0.000", font=('Consolas', 14))
        self.label_vact = tk.Label(self.root, text="Velocity Actual: 0.000", font=('Consolas', 14))
        self.label_verr = tk.Label(self.root, text="Velocity Error: 0.000", font=('Consolas', 14))

        self.label_pos.pack(pady=5)
        self.label_vref.pack(pady=5)
        self.label_vact.pack(pady=5)
        self.label_verr.pack(pady=5)

        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleLiveGraph()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
