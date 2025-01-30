import rclpy
from rclpy.node import Node
from mavros_msgs.msg import VfrHud

class VFRSubscriber(Node):
    def __init__(self):
        super().__init__('vfr_subscriber')
        self.subscription = self.create_subscription(
            VfrHud,
            '/mavros/vfr_hud',
            self.callback,
            10
        )
        self.get_logger().info("Subscribed to /mavros/vfr_hud")

    def callback(self, msg):
        self.get_logger().info(f"Altitude: {msg.altitude}, Vertical Speed: {msg.climb}, "
                               f"Ground Speed: {msg.groundspeed}, Heading: {msg.heading}")

def main(args=None):
    rclpy.init(args=args)
    node = VFRSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
