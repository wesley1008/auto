import rclpy
from rclpy.node import Node
from mavros_msgs.msg import VfrHud, AttitudeTarget
from rclpy.qos import QoSProfile, ReliabilityPolicy

class VerticalSpeedControlNode(Node):
    def __init__(self):
        super().__init__('vertical_speed_control_node')

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.vfr_hud_subscription = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.vfr_hud_callback, qos_profile)
        self.attitude_publisher = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        self.declare_parameter('target_vertical_speed', 0.0)
        self.declare_parameter('kp_vs', 0.2)
        self.declare_parameter('ki_vs', 0.0134)
        self.declare_parameter('kd_vs', 0.04)
	
        self.vertical_speed = 0.0
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_thrust = 0.5

        self.timer = self.create_timer(0.05, self.control_vertical_speed)

        self.get_logger().info('Vertical speed control node initialized.')

    def vfr_hud_callback(self, msg):
        self.vertical_speed = msg.climb  # Read vertical speed from MAVROS

    def control_vertical_speed(self):
        target_vertical_speed = self.get_parameter('target_vertical_speed').value
        kp_vs = self.get_parameter('kp_vs').value
        ki_vs = self.get_parameter('ki_vs').value
        kd_vs = self.get_parameter('kd_vs').value

        error = target_vertical_speed - self.vertical_speed
        proportional = kp_vs * error

        self.error_sum += error * 0.05
        integral = ki_vs * self.error_sum

        derivative = kd_vs * (error - self.last_error) / 0.05
        self.last_error = error

        raw_thrust = proportional + integral + derivative

        # Adjust thrust incrementally (emulating human-like control)
        if raw_thrust > self.last_thrust + 0.02:
            target_thrust = self.last_thrust + 0.02
        elif raw_thrust < self.last_thrust - 0.02:
            target_thrust = self.last_thrust - 0.02
        else:
            target_thrust = raw_thrust

        target_thrust = max(0.3, min(1.0, target_thrust))  # Keep thrust within limits
        self.last_thrust = target_thrust

        # Publish thrust
        attitude_target = AttitudeTarget()
        attitude_target.orientation.x = 0.0
        attitude_target.orientation.y = 0.0
        attitude_target.orientation.z = 0.0
        attitude_target.orientation.w = 1.0
        attitude_target.body_rate.x = 0.0
        attitude_target.body_rate.y = 0.0
        attitude_target.body_rate.z = 0.0
        attitude_target.thrust = target_thrust

        self.attitude_publisher.publish(attitude_target)

        self.get_logger().info(
            f"Target VS: {target_vertical_speed:.2f}, Vertical Speed: {self.vertical_speed:.2f}, Thrust: {target_thrust:.2f}, Error: {error:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VerticalSpeedControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
