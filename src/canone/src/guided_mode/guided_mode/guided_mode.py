import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from mavros_msgs.srv import SetMode
from std_msgs.msg import Bool


class ModeSwitcher(Node):
    """set mode to GUIDED via mavros"""
    def __init__(self):
        super().__init__('guided_mode')
        self.cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        self.req = SetMode.Request(base_mode=0, custom_mode="GUIDED")

        # Publisher → signal depth controller (latched)
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.ready_pub = self.create_publisher(Bool, '/mode/ready', qos)

    def send_request(self):
        self.get_logger().info("Sending GUIDED mode request...")
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info("✅ Vehicle is now in GUIDED mode")
            self.ready_pub.publish(Bool(data=True))
        else:
            self.get_logger().error("❌ Failed to set GUIDED mode")

def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcher()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
