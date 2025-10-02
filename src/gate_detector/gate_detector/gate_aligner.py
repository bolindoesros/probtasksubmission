#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from vision_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class GateAligner(Node):
    def __init__(self):
        super().__init__('gate_aligner')

        # ---- Tunables ----
        self.center_tol    = 0.075     # how close to center counts as aligned
        self.min_area      = 0.010     # must be at least this big to align
        self.safe_commit   = 0.25      # start commit logic when gate fills >20%
        self.direct_commit = 0.50      # emergency commit if gate fills >50%
        self.hold_time     = 0.10      # seconds stable before commit
        self.detect_frames = 3         # frames to confirm detection
        self.lost_grace    = 0.5       # seconds to tolerate lost gate

        # Motion
        self.k_yaw         = 0.8       # yaw proportional gain
        self.creep_speed   = 0.10      # slow forward while approaching (<20%)
        self.search_spin   = -0.5      # spin rate when searching

        # ---- State ----
        self.depth_ready    = False
        self.state          = "SEARCHING"
        self.centered_since = None
        self.last_seen      = None
        self.visible_count  = 0
        self.commit_sent    = False

        # ---- IO ----
        # Make /gate/commit a latched publisher (TRANSIENT_LOCAL) so late subscribers receive last True
        commit_qos = QoSProfile(depth=1)
        commit_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.commit_pub = self.create_publisher(Bool, '/gate/commit', commit_qos)
        self.cmd_pub    = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.sub        = self.create_subscription(BoundingBoxArray, '/main_camera/detection/bounding_boxes', self.cb, 10)

        # Subscribe to /depth/ready with latched QoS
        depth_qos = QoSProfile(depth=1)
        depth_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, '/depth/ready', self.depth_cb, depth_qos)
        self.timer = self.create_timer(0.05, self._tick)
        self.latest_cmd = Twist()
        self.get_logger().info("GateAligner loaded → waiting for /depth/ready...")

    def depth_cb(self, msg: Bool):
        if msg.data and not self.depth_ready:
            self.depth_ready = True
            self.get_logger().info("✅ /depth/ready received → SEARCHING for gate.")

    def _now(self):
        return self.get_clock().now()

    def _extract_bbox(self, b):
        if hasattr(b, 'center') and hasattr(b, 'size'):
            cx, cy = float(b.center.x), float(b.center.y)
            w, h = float(b.size.x), float(b.size.y)
            if w > 0.0 and h > 0.0:
                return (cx, cy, w, h)
        if all(hasattr(b, f) for f in ('x', 'y', 'w', 'h')):
            cx, cy = float(b.x), float(b.y)
            w, h = float(b.w), float(b.h)
            if w > 0.0 and h > 0.0:
                return (cx, cy, w, h)
        if all(hasattr(b, f) for f in ('xmin', 'xmax', 'ymin', 'ymax')):
            xmin, xmax = float(b.xmin), float(b.xmax)
            ymin, ymax = float(b.ymin), float(b.ymax)
            w, h = max(0.0, xmax - xmin), max(0.0, ymax - ymin)
            if w > 0.0 and h > 0.0:
                cx, cy = xmin + w / 2.0, ymin + h / 2.0
                return (cx, cy, w, h)
        return None

    def cb(self, msg: BoundingBoxArray):
        if self.commit_sent or not self.depth_ready:
            return

        if not msg.bounding_boxes:
            self._maybe_lost()
            if self.state == "SEARCHING":
                cmd = Twist()
                cmd.angular.z = self.search_spin
                self.latest_cmd = cmd
                self.get_logger().info("[SEARCH] spinning for gate...")
            return

        # Pick largest detection
        best, best_area = None, -1.0
        for b in msg.bounding_boxes:
            bb = self._extract_bbox(b)
            if bb:
                _, _, w, h = bb
                a = w * h
                if a > best_area:
                    best_area, best = a, bb
        if not best:
            self._maybe_lost()
            return

        cx, cy, w, h = best
        ex = cx - 0.5
        area = w * h
        centered = abs(ex) < self.center_tol
        self.last_seen = self._now()

        # SEARCHING → ALIGNING
        if self.state == "SEARCHING":
            self.visible_count += 1
            if self.visible_count >= self.detect_frames:
                self.state = "ALIGNING"
                self.get_logger().info("✅ Gate detected → ALIGNING.")
            else:
                cmd = Twist()
                cmd.angular.z = self.search_spin
                self.latest_cmd = cmd
            return

        # ALIGNING
        if self.state == "ALIGNING":
            cmd = Twist()
            cmd.angular.z = -self.k_yaw * ex  # yaw correction always

            if centered and area > self.min_area:
                if self.centered_since is None:
                    self.centered_since = self._now()
                held = (self._now() - self.centered_since).nanoseconds / 1e9

                # Stage 1: creep forward until gate ≥ safe_commit
                if area < self.safe_commit:
                    cmd.linear.x = self.creep_speed
                    self.get_logger().info(f"[ALIGN] creeping forward, area={area:.3f}")

                # Stage 2: once large enough, stop creep and hold center
                else:
                    cmd.linear.x = 0.0
                    self.get_logger().info(f"[ALIGN] stable near gate, area={area:.3f}")

                    # Emergency commit if super close
                    if area >= self.direct_commit:
                        self.get_logger().info("🚀 Gate fills >50% → committing immediately!")
                        self._publish_commit()
                        return

                    # Normal commit after hold
                    if held >= self.hold_time:
                        self._publish_commit()
                        return
            else:
                self.centered_since = None

            self.latest_cmd = cmd

    def _maybe_lost(self):
        if self.state == "ALIGNING" and self.last_seen:
            dt = (self._now() - self.last_seen).nanoseconds / 1e9
            if dt < self.lost_grace:
                return
            self.get_logger().warn("⚠️ Lost gate → back to SEARCHING.")
            self.state = "SEARCHING"
            self.visible_count = 0
            self.centered_since = None

    def _publish_commit(self):
        if self.commit_sent:
            return
        self.commit_sent = True
        self.state = "COMMITTED"
        self.commit_pub.publish(Bool(data=True))
        self.get_logger().info("🎯 Alignment stable → COMMIT=True")
        self.latest_cmd = Twist()
        self.cmd_pub.publish(self.latest_cmd)
        self.destroy_timer(self.timer)
        self.destroy_subscription(self.sub)

    def _tick(self):
        if not self.commit_sent and self.depth_ready:
            self.cmd_pub.publish(self.latest_cmd)


def main():
    rclpy.init()
    n = GateAligner()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
