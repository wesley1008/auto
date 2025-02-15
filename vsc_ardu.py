import sys
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import VfrHud, AttitudeTarget, State
from rclpy.qos import QoSProfile, ReliabilityPolicy
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton


# 定義一個 ROS2 節點來控制垂直速度
class VerticalSpeedControlNode(Node):
    def __init__(self):
        super().__init__('vertical_speed_control_node')

        # 設置 QoS 設置，指定訊息的深度和可靠性
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # 訂閱 /mavros/vfr_hud 以獲取飛行參數，並訂閱 /mavros/state 以獲取飛行狀態
        self.vfr_hud_subscription = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.vfr_hud_callback, qos_profile)
        self.attitude_publisher = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self.state_subscription = self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile)

        # 宣告一些參數，如目標垂直速度和 PID 控制參數
        self.declare_parameter('target_vertical_speed', 0.0)
        self.declare_parameter('kp_vs', 0.108)
        self.declare_parameter('ki_vs', 0.06210)
        self.declare_parameter('kd_vs', 0.020205)

        # 初始化狀態變量
        self.altitude = 0.0
        self.vertical_speed = 0.0
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_thrust = 0.5
        self.is_offboard = False

        # 設置定時器，每 0.05 秒執行一次 control_vertical_speed 函數
        self.timer = self.create_timer(0.05, self.control_vertical_speed)

        self.get_logger().info('Vertical speed control node initialized.')

    # 用來處理 /mavros/vfr_hud 訊息的回調函數
    def vfr_hud_callback(self, msg):
        self.altitude = msg.altitude  # 更新飛行高度
        self.vertical_speed = msg.climb  # 更新垂直速度

    # 用來處理 /mavros/state 訊息的回調函數
    def state_callback(self, msg):
        # 檢查是否處於 GUIDED_NOGPS 模式
        self.is_offboard = (msg.mode == "GUIDED_NOGPS")
        if not self.is_offboard:
            self.get_logger().warn("Not in GUIDED_NOGPS mode. PID control paused.")

    # 控制垂直速度的函數
    def control_vertical_speed(self):
        if not self.is_offboard:
            # 如果不在 GUIDED_NOGPS 模式下，則發送一個空的指令以停止控制
            attitude_target = AttitudeTarget()
            attitude_target.orientation.x = 0.0
            attitude_target.orientation.y = 0.0
            attitude_target.orientation.z = 0.0
            attitude_target.orientation.w = 0.5
            attitude_target.body_rate.x = 0.0
            attitude_target.body_rate.y = 0.0
            attitude_target.body_rate.z = 0.0
            self.attitude_publisher.publish(attitude_target)
            return

        # 從參數中獲取目標垂直速度和 PID 參數
        target_vertical_speed = self.get_parameter('target_vertical_speed').value
        kp_vs = self.get_parameter('kp_vs').value
        ki_vs = self.get_parameter('ki_vs').value
        kd_vs = self.get_parameter('kd_vs').value

        # 計算 PID 控制器的三個部分（比例、積分、微分）
        error = target_vertical_speed - self.vertical_speed
        proportional = kp_vs * error

        self.error_sum += error * 0.05  # 積分部分
        integral = ki_vs * self.error_sum

        derivative = kd_vs * (error - self.last_error) / 0.05  # 微分部分
        self.last_error = error

        # 計算原始的推力
        raw_thrust = proportional + integral + derivative

        # 限制推力變化量，模擬人類控制
        if raw_thrust > self.last_thrust + 0.02:
            target_thrust = self.last_thrust + 0.02
        elif raw_thrust < self.last_thrust - 0.02:
            target_thrust = self.last_thrust - 0.02
        else:
            target_thrust = raw_thrust

        target_thrust = max(0.3, min(1.0, target_thrust))  # 限制推力範圍
        self.last_thrust = target_thrust

        # 發送目標姿態命令，控制垂直速度
        attitude_target = AttitudeTarget()
        attitude_target.orientation.x = 0.0
        attitude_target.orientation.y = 0.0
        attitude_target.orientation.z = 0.0
        attitude_target.orientation.w = 1.0
        attitude_target.body_rate.x = 0.0
        attitude_target.body_rate.y = 0.0
        attitude_target.body_rate.z = target_vertical_speed  # 使用 body_rate.z 控制垂直速度
        attitude_target.thrust = target_thrust

        # 發送控制命令
        self.attitude_publisher.publish(attitude_target)

        self.get_logger().info(
            f"Target VS: {target_vertical_speed:.2f}, Vertical Speed: {self.vertical_speed:.2f}, Thrust: {target_thrust:.2f}, Error: {error:.2f}"
        )


# GUI 用來控制垂直速度和 PID 參數
class VerticalSpeedControlGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Vertical Speed Control")

        layout = QVBoxLayout()

        # 設置目標垂直速度的 UI 元素
        self.vs_label = QLabel("Target Vertical Speed (m/s):")
        self.vs_input = QLineEdit()
        self.vs_input.setText(str(self.node.get_parameter('target_vertical_speed').value))
        self.set_vs_button = QPushButton("Set Vertical Speed")
        self.set_vs_button.clicked.connect(self.set_vertical_speed)

        # 設置 PID 參數的 UI 元素
        self.kp_label = QLabel("Kp:")
        self.kp_input = QLineEdit()
        self.kp_input.setText(str(self.node.get_parameter('kp_vs').value))
        self.ki_label = QLabel("Ki:")
        self.ki_input = QLineEdit()
        self.ki_input.setText(str(self.node.get_parameter('ki_vs').value))
        self.kd_label = QLabel("Kd:")
        self.kd_input = QLineEdit()
        self.kd_input.setText(str(self.node.get_parameter('kd_vs').value))
        self.set_pid_button = QPushButton("Set PID Parameters")
        self.set_pid_button.clicked.connect(self.set_pid_parameters)

        layout.addWidget(self.vs_label)
        layout.addWidget(self.vs_input)
        layout.addWidget(self.set_vs_button)

        layout.addWidget(self.kp_label)
        layout.addWidget(self.kp_input)
        layout.addWidget(self.ki_label)
        layout.addWidget(self.ki_input)
        layout.addWidget(self.kd_label)
        layout.addWidget(self.kd_input)
        layout.addWidget(self.set_pid_button)

        self.setLayout(layout)

    # 設置目標垂直速度
    def set_vertical_speed(self):
        try:
            target_vs = float(self.vs_input.text())
            self.node.set_parameters([rclpy.parameter.Parameter('target_vertical_speed', rclpy.Parameter.Type.DOUBLE, target_vs)])
            self.node.get_logger().info(f"Target vertical speed set to: {target_vs} m/s")
        except ValueError:
            self.node.get_logger().error("Invalid input for vertical speed. Please enter a valid number.")

    # 設置 PID 參數
    def set_pid_parameters(self):
        try:
            kp = float(self.kp_input.text())
            ki = float(self.ki_input.text())
            kd = float(self.kd_input.text())

            self.node.set_parameters([
                rclpy.parameter.Parameter('kp_vs', rclpy.Parameter.Type.DOUBLE, kp),
                rclpy.parameter.Parameter('ki_vs', rclpy.Parameter.Type.DOUBLE, ki),
                rclpy.parameter.Parameter('kd_vs', rclpy.Parameter.Type.DOUBLE, kd),
            ])
            self.node.get_logger().info(f"PID parameters set to: Kp={kp}, Ki={ki}, Kd={kd}")
        except ValueError:
            self.node.get_logger().error("Invalid input for PID parameters. Please enter valid numbers.")


# 程式入口
def main(args=None):
    rclpy.init(args=args)
    node = VerticalSpeedControlNode()

    # 初始化 PyQt5 應用並顯示 GUI
    app = QApplication(sys.argv)
    gui = VerticalSpeedControlGUI(node)

    gui.show()

    # 啟動 ROS2 節點的獨立執行緒
    def spin_node():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    import threading
    ros_thread = threading.Thread(target=spin_node)
    ros_thread.start()

    sys.exit(app.exec_())

    # 關閉 ROS2 節點
    node.destroy_node()
    rclpy.shutdown()


# 主程式入口
if __name__ == '__main__':
    main()

