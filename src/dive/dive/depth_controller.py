import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class DepthController(Node):
    def __init__(self):
        super().__init__('depth_controller')

        # --- Parameters ---
        self.target = -1.8         # target depth
        self.tol = 0.1             # tolerance 
        self.kp = 1.2              # proportional gain
        self.max_speed = 0.6       # max vertical speed
        self.initial_speed = -0.6  # fast dive at start
        self.hold_time = 1.0       # hold before "ready"

        # --- State ---
        self.centered_since = None
        self.active = True
        self.mode_ready = False

        # --- IO ---
        self.pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        # Latch /depth/ready so late subscribers get it
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.depth_ready_pub = self.create_publisher(Bool, '/depth/ready', qos)
        self.sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.cb, 10)
        self.create_subscription(Bool, '/mode/ready', self.mode_cb, 1)
        self.get_logger().info("DepthController loaded → waiting for /mode/ready...")

    def mode_cb(self, msg: Bool):
        if msg.data:
            self.mode_ready = True
            self.get_logger().info("✅ /mode/ready received → starting depth control")

    def cb(self, msg: Float64):
        if not self.mode_ready or not self.active:
            return

        current = msg.data
        err = self.target - current
        cmd = Twist()

        if abs(err) < self.tol:
            cmd.linear.z = 0.0

            if self.centered_since is None:
                self.centered_since = self.get_clock().now()
            held = (self.get_clock().now() - self.centered_since).nanoseconds / 1e9
            self.get_logger().info(f"Holding={current:.2f} m | held={held:.2f}s")

            if held >= self.hold_time:
                self.get_logger().info("✅ Depth stabilized → signalling /depth/ready")
                self.depth_ready_pub.publish(Bool(data=True))
                self.active = False
                self.destroy_subscription(self.sub)
                self.pub.publish(Twist())  # stop
                return
        else:
            self.centered_since = None
            if abs(err) > 0.5:
                vz = self.initial_speed if err < 0 else -self.initial_speed
            else:
                vz = max(-self.max_speed, min(self.kp * err, self.max_speed))
            cmd.linear.z = vz
            self.get_logger().info(f"Current={current:.2f} m | vz={vz:.2f}")

        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DepthController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
