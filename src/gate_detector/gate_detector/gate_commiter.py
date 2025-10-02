#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class GateCommitter(Node):
    def __init__(self):
        super().__init__('gate_committer')

        # ==== Parameters ====
        self.commit_speed    = 0.5    # forward speed (x)
        self.commit_down     = -0.1   # downward push (z)
        self.commit_duration = 4.0    # seconds if latch=False
        self.latch           = True   # if True → go forever until external stop
        self.pre_yaw_speed   = 0.10   # angular speed for pre-yaw
        self.pre_yaw_time    = 0.20   # seconds to yaw before commit

        # ==== State ====
        self._yawing = False
        self._committing = False
        self._start_ns = None

        # ==== ROS I/O ====
        self.cmd_pub = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10
        )
        self.create_subscription(Bool, '/gate/commit', self._commit_cb, 10)
        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info("GateCommitter loaded → waiting for /gate/commit.")

    # Callbacks
    
    def _commit_cb(self, msg: Bool):
        if not msg.data or self._committing or self._yawing:
            return
        self._yawing = True
        self._start_ns = self.get_clock().now().nanoseconds
        self.get_logger().info("↩️ COMMIT received → yawing left before forward...")

    def _tick(self):
        if not (self._yawing or self._committing):
            return

        elapsed = (self.get_clock().now().nanoseconds - self._start_ns) / 1e9
        cmd = Twist()

        # --- Pre-yaw stage ---
        if self._yawing:
            cmd.angular.z = self.pre_yaw_speed
            self.cmd_pub.publish(cmd)

            if elapsed >= self.pre_yaw_time:
                self._yawing = False
                self._committing = True
                self._start_ns = self.get_clock().now().nanoseconds
                self.get_logger().info(
                    f"✅ Pre-yaw done, committing forward {self.commit_speed} m/s, down {self.commit_down} m/s"
                )
            return

        # --- Commit stage ---
        if self._committing:
            cmd.linear.x = self.commit_speed
            cmd.linear.z = self.commit_down
            self.cmd_pub.publish(cmd)

            # Stop automatically if latch disabled
            if not self.latch and elapsed >= self.commit_duration > 0:
                self._committing = False
                self.cmd_pub.publish(Twist())  # stop vehicle
                self.get_logger().info("⏹️ Commit duration elapsed → STOP")
                self.destroy_timer(self.timer)


def main():
    rclpy.init()
    node = GateCommitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
