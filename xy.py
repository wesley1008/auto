import sys
import rclpy
import numpy as np
from rclpy.node import Node
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mavros_msgs.msg import VfrHud, AttitudeTarget, State, GPSRAW
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton
from scipy.spatial.transform import Rotation as R
import threading
import time
import matplotlib.pyplot as plt
import sys


# 定義一個 ROS2 節點來控制垂直速度
class VerticalSpeedControlNode(Node):
    def __init__(self):
        super().__init__('vertical_speed_control_node')

        # 設置 QoS 設置，指定訊息的深度和可靠性
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT



        self.vfr_hud_subscription = self.create_subscription(VfrHud, '/mavros/vfr_hud', self.vfr_hud_callback, qos_profile)
        self.attitude_publisher = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)
        self.state_subscription = self.create_subscription(State, '/mavros/state', self.state_callback, qos_profile)
        self.gps_subscription = self.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self.gps_callback, 10)

        # 宣告一些參數，如目標垂直速度和 PID 控制參數
        self.declare_parameter('target_vertical_speed', 0.0)
        self.declare_parameter('kp_vs', 0.108)
        self.declare_parameter('ki_vs', 0.06210)
        self.declare_parameter('kd_vs', 0.020205)

        self.declare_parameter('ok_to_set_fly', 'n')

        self.declare_parameter('target_altitude', 0.0)
        self.declare_parameter('kp_alt', 0.32)
        self.declare_parameter('ki_alt', 0.0)
        self.declare_parameter('kd_alt', 0.0)
        self.declare_parameter('max_vertical_speed', 4.0)
        self.declare_parameter('min_vertical_speed', -2.0)
        self.declare_parameter('altitude_tolerance', 0.0)

        self.declare_parameter('target_vx', 0.0)
        self.declare_parameter('target_vy', 0.0)
        self.declare_parameter('kp_xy_vs', 0.25)
        self.declare_parameter('ki_xy_vs', 0.0)
        self.declare_parameter('kd_xy_vs', 0.0)

        # 預設參數(speed)
        self.vertical_speed = 0.0
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_thrust = 0.5
        self.is_offboard = False

        # xy
        self.vx = 0.0
        self.vy = 0.0
        self.last_x = None
        self.last_y = None
        self.last_time = time.time() 
        self.is_get_first_alt = True
        self.get_first_alt = 0.0
        self.get_first_lon = 0.0
        self.get_first_lat = 0.0
        self.error_vx = 0.0
        self.error_vy = 0.0
        self.last_error_vx = 0.0
        self.last_error_vy = 0.0

        # 預設參數(altitu)
        self.altitude = 0.0
        self.error_alt_sum = 0.0
        self.last_alt_error = 0.0
        self.last_alt_thrust = 0.5
        self.target_speeds = []



        self.atimer = None
        self.timer = self.create_timer(0.05, self.control_vertical_speed)
        self.stimer = self.create_timer(0.5, self.start_set_high)
        

        self.get_logger().info('Vertical speed control node initialized.')
    
    def gps_callback(self, msg):
        if self.is_get_first_alt:
            self.get_first_alt = msg.alt
            self.get_first_lat = msg.lat
            self.get_first_lon = msg.lon
            self.is_get_first_alt = False

        current_time = time.time()
        # start location is (-353632629, 1491652380)
        self.x_pos = (msg.lat+353632629)/100
        self.y_pos = (msg.lon-1491652380)/100
        dt = current_time - self.last_time

        if dt<0.156:
            return

        if self.last_x is not None and self.last_y is not None:
            # 計算 ΔX, ΔY, Δt
            self.vx = (self.x_pos - self.last_x) / dt  # 計算 X 方向速度
            self.vy = (self.y_pos - self.last_y) / dt  # 計算 Y 方向速度

        # 更新記錄
        self.last_x = self.x_pos
        self.last_y = self.y_pos
        self.last_time = current_time


    def vfr_hud_callback(self, msg):
        self.altitude = msg.altitude
        self.vertical_speed = msg.climb  # Read vertical speed from MAVROS

    def state_callback(self, msg):
        self.is_offboard = (msg.mode == "GUIDED_NOGPS")
        if not self.is_offboard:
            self.get_logger().warn("Not in OFFBOARD mode. PID control paused.(sc)")
    
    def start_set_high(self):
        if self.get_parameter('ok_to_set_fly').value == "y":
            # 如果 altitude_timer 尚未存在，则创建新的定时器
            if self.atimer is None:
                self.get_logger().info("Creating altitude control timer.")
                self.atimer = self.create_timer(0.05, self.control_altitu)
        else:
            # 如果 altitude_timer 存在，则销毁它
            if self.atimer is not None:
                self.get_logger().info("Destroying altitude control timer.")
                self.destroy_timer(self.atimer)
                self.atimer = None


    def control_vertical_speed(self):
        if not self.is_offboard:
            # Publish thrust
            attitude_target = AttitudeTarget()
            attitude_target.orientation.x = 0.0
            attitude_target.orientation.y = 0.0
            attitude_target.orientation.z = 0.0
            attitude_target.orientation.w = 0.5
            attitude_target.body_rate.x = 0.0
            attitude_target.body_rate.y = 0.0
            attitude_target.body_rate.z = 0.0
            self.attitude_publisher.publish(attitude_target)
            return  # Pause PID control if not in OFFBOARD mode(From cvs)
        # vs para
        target_vertical_speed = self.get_parameter('target_vertical_speed').value
        kp_vs = self.get_parameter('kp_vs').value
        ki_vs = self.get_parameter('ki_vs').value
        kd_vs = self.get_parameter('kd_vs').value

        # vs error
        error = target_vertical_speed - self.vertical_speed
        proportional = kp_vs * error

        
        # vs caculate
        self.error_sum += error * 0.05
        integral = ki_vs * self.error_sum

        derivative = kd_vs * (error - self.last_error) / 0.05
        self.last_error = error

        raw_thrust = proportional + integral + derivative

        #Adjust thrust incrementally (emulating human-like control)
        if raw_thrust > self.last_thrust + 0.02:
            target_thrust = self.last_thrust + 0.02
        elif raw_thrust < self.last_thrust - 0.02:
            target_thrust = self.last_thrust - 0.02
        else:
            target_thrust = raw_thrust

        target_thrust = max(0.3, min(1.0, target_thrust))  # Keep thrust within limits
        self.last_thrust = target_thrust


        # xy para(0, 0)
        target_vx = self.get_parameter('target_vx').value
        target_vy = self.get_parameter('target_vy').value
        kp_xy_vs = self.get_parameter('kp_xy_vs').value
        ki_xy_vs = self.get_parameter('ki_xy_vs').value
        kd_xy_vs = self.get_parameter('kd_xy_vs').value

        # xy error
        error_vx = target_vx - self.vx
        error_vy = target_vy - self.vy

        # xy angle
        # proportional_pitch = kp_xy_vs * error_vx
        proportional_roll = -kp_xy_vs * error_vy
        proportional_pitch = 0.0

        self.error_vx += error_vx * 0.05
        self.error_vy += error_vy * 0.05
        integral_vx = ki_xy_vs * self.error_vx
        integral_vy = ki_xy_vs * self.error_vy

        derivative_vx = kd_xy_vs * (error_vx - self.last_error_vx) / 0.05
        derivative_vy = kd_xy_vs * (error_vy - self.last_error_vy) / 0.05
        self.last_error_vx = error_vx
        self.last_error_vy = error_vy

        pitch_correction = proportional_pitch + integral_vx + derivative_vx
        roll_correction = proportional_roll + integral_vy + derivative_vy

        max_angle = np.deg2rad(10)  # 限制最大 10°
        pitch_correction = np.clip(pitch_correction, -max_angle, max_angle)
        roll_correction = np.clip(roll_correction, -max_angle, max_angle)


        # alttitude man min limit
        # max_angle = np.deg2rad(10)
        # pitch_correction = np.clip(pitch_correction, -max_angle, max_angle)
        # roll_correction = np.clip(roll_correction, -max_angle, max_angle)

        # 轉換為四元數
        quat = R.from_euler('xyz', [roll_correction, pitch_correction, 0]).as_quat()


        # Publish thrust
        attitude_target = AttitudeTarget()
        attitude_target.orientation.x = quat[0]
        attitude_target.orientation.y = quat[1]
        attitude_target.orientation.z = quat[2]
        attitude_target.orientation.w = quat[3]
        attitude_target.type_mask = 7
        attitude_target.thrust = 0.5

        self.attitude_publisher.publish(attitude_target)

        self.get_logger().info(
            f"Target X: {target_vx:.2f},Target Y: {target_vy:.2f}, roll: {roll_correction:.2f}, pitch: {pitch_correction:.2f}, ErrorX: {error_vx:.2f}, ErrorY: {error_vy:.2f}"
        )

    def control_altitu(self):
        self.get_logger().info("Controlling altitude...")

        # 如果不在 OFFBOARD 模式，则暂停 PID 控制
        if not self.is_offboard:
            return  # Pause PID control if not in OFFBOARD mode(From ca)

        # 获取目标高度和 PID 参数
        target_alt = self.get_parameter('target_altitude').value
        kp_alt = self.get_parameter('kp_alt').value
        ki_alt = self.get_parameter('ki_alt').value
        kd_alt = self.get_parameter('kd_alt').value

        # 计算高度误差
        error_alt = target_alt - self.altitude + 584.18
        proportional = kp_alt * error_alt

        # 积分项，限制积分项避免积分饱和
        self.error_alt_sum += error_alt * 0.05  # 0.05 为控制周期 (秒)
        integral_limit = 10.0  # 可根据需求调整限制范围
        self.error_alt_sum = max(-integral_limit, min(integral_limit, self.error_alt_sum))
        integral = ki_alt * self.error_alt_sum

        # 微分项
        derivative = kd_alt * (error_alt - self.last_alt_error) / 0.05
        self.last_alt_error = error_alt

        # 计算目标垂直速度 (target_vertical_speed)
        raw_vertical_speed = proportional + integral + derivative

        # 限制目标垂直速度范围，避免超出可控速度
        min_speed = self.get_parameter('min_vertical_speed').value
        max_speed = self.get_parameter('max_vertical_speed').value
        target_vertical_speed = max(min_speed, min(max_speed, raw_vertical_speed))
        # self.get_logger().info(
        #     f"Min Speed: {min_speed}, Max Speed: {max_speed}, Raw VS: {raw_vertical_speed}"
        # )

        

        # 将目标垂直速度设置到参数中，供速度 PID 控制器使用
        self.set_parameters([
            rclpy.parameter.Parameter(
                'target_vertical_speed',
                rclpy.Parameter.Type.DOUBLE,
                target_vertical_speed
            )
        ])

        # 输出日志供调试
        self.get_logger().info(
            f"Altitude Error: {error_alt:.2f}, Target VS: {target_vertical_speed:.2f} m/s, "
            f"P: {proportional:.2f}, I: {integral:.2f}, D: {derivative:.2f}"
        )






class VerticalSpeedControlGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.fig)
        self.line, = self.ax.plot([], [], label="Target Vertical Speed (m/s)", color="blue")
        self.ax.set_title("Target Vertical Speed Over Time")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (m/s)")
        self.ax.legend()
        self.ax.grid()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Vertical Speed & Altitude Control")

        layout = QVBoxLayout()

        # Target Vertical Speed Control
        self.vs_label = QLabel("Target Vertical Speed (m/s):")
        self.vs_input = QLineEdit()
        self.vs_input.setText(str(self.node.get_parameter('target_vertical_speed').value))
        self.set_vs_button = QPushButton("Set Vertical Speed")
        self.set_vs_button.clicked.connect(self.set_vertical_speed)

        # OK?
        self.ok_label = QLabel("Target Vertical Speed (m/s):")
        self.ok_input = QLineEdit()
        self.ok_input.setText(str(self.node.get_parameter('ok_to_set_fly').value))
        self.set_ok_button = QPushButton("OK to set alt")
        self.set_ok_button.clicked.connect(self.ok_to_set_fly)

        # Altitude Control
        self.alt_label = QLabel("Target Altitude (m):")
        self.alt_input = QLineEdit()
        self.alt_input.setText(str(self.node.get_parameter('target_altitude').value))
        self.set_alt_button = QPushButton("Set Altitude")
        self.set_alt_button.clicked.connect(self.set_altitude)

        # PID Parameters Control for Vertical Speed
        self.kp_label = QLabel("Kp (Vertical Speed):")
        self.kp_input = QLineEdit()
        self.kp_input.setText(str(self.node.get_parameter('kp_vs').value))
        self.ki_label = QLabel("Ki (Vertical Speed):")
        self.ki_input = QLineEdit()
        self.ki_input.setText(str(self.node.get_parameter('ki_vs').value))
        self.kd_label = QLabel("Kd (Vertical Speed):")
        self.kd_input = QLineEdit()
        self.kd_input.setText(str(self.node.get_parameter('kd_vs').value))
        self.set_pid_button = QPushButton("Set PID Parameters")
        self.set_pid_button.clicked.connect(self.set_pid_parameters)

        # Altitude PID Control
        self.alt_kp_label = QLabel("Kp (Altitude):")
        self.alt_kp_input = QLineEdit()
        self.alt_kp_input.setText(str(self.node.get_parameter('kp_alt').value))
        self.alt_ki_label = QLabel("Ki (Altitude):")
        self.alt_ki_input = QLineEdit()
        self.alt_ki_input.setText(str(self.node.get_parameter('ki_alt').value))
        self.alt_kd_label = QLabel("Kd (Altitude):")
        self.alt_kd_input = QLineEdit()
        self.alt_kd_input.setText(str(self.node.get_parameter('kd_alt').value))
        self.alt_speed_label = QLabel("Max Vertical Speed (m/s):")
        self.alt_speed_input = QLineEdit()
        self.alt_speed_input.setText(str(self.node.get_parameter('max_vertical_speed').value))
        self.alt_speedd_label = QLabel("Min Vertical Speed (m/s):")
        self.alt_speedd_input = QLineEdit()
        self.alt_speedd_input.setText(str(self.node.get_parameter('min_vertical_speed').value))
        self.alt_tolerance_label = QLabel("Altitude Tolerance (m):")
        self.alt_tolerance_input = QLineEdit()
        self.alt_tolerance_input.setText(str(self.node.get_parameter('altitude_tolerance').value))
        self.set_alt_pid_button = QPushButton("Set Altitude Parameters")
        self.set_alt_pid_button.clicked.connect(self.set_alt_pid_parameters)

        # Add all widgets to the layout
        layout.addWidget(self.vs_label)
        layout.addWidget(self.vs_input)
        layout.addWidget(self.set_vs_button)

        layout.addWidget(self.ok_label)
        layout.addWidget(self.ok_input)
        layout.addWidget(self.set_ok_button)

        layout.addWidget(self.kp_label)
        layout.addWidget(self.kp_input)
        layout.addWidget(self.ki_label)
        layout.addWidget(self.ki_input)
        layout.addWidget(self.kd_label)
        layout.addWidget(self.kd_input)
        layout.addWidget(self.set_pid_button)

        layout.addWidget(self.alt_kp_label)
        layout.addWidget(self.alt_kp_input)
        layout.addWidget(self.alt_ki_label)
        layout.addWidget(self.alt_ki_input)
        layout.addWidget(self.alt_kd_label)
        layout.addWidget(self.alt_kd_input)
        layout.addWidget(self.alt_speed_label)
        layout.addWidget(self.alt_speed_input)
        layout.addWidget(self.alt_speedd_label)
        layout.addWidget(self.alt_speedd_input)
        layout.addWidget(self.alt_tolerance_label)
        layout.addWidget(self.alt_tolerance_input)
        layout.addWidget(self.set_alt_pid_button)

        layout.addWidget(self.alt_label)
        layout.addWidget(self.alt_input)
        layout.addWidget(self.set_alt_button)
        layout.addWidget(self.canvas)

        self.setLayout(layout)

    def set_vertical_speed(self):
        try:
            target_vs = float(self.vs_input.text())
            self.node.set_parameters([rclpy.parameter.Parameter('target_vertical_speed', rclpy.Parameter.Type.DOUBLE, target_vs)])
            self.node.get_logger().info(f"Target vertical speed set to: {target_vs} m/s")
        except ValueError:
            self.node.get_logger().error("Invalid input for vertical speed. Please enter a valid number.")

    def ok_to_set_fly(self):
        try:
            ISOK = self.ok_input.text()
            self.node.set_parameters([rclpy.parameter.Parameter('ok_to_set_fly', rclpy.Parameter.Type.STRING, ISOK)])
            self.node.get_logger().info("Is ready to set high ? :" + ISOK )
        except ValueError:
            self.node.get_logger().error("Invalid input for vertical speed. Please enter a valid number.")

    def set_altitude(self):
        try:
            # 獲取目標高度
            target_altitude = float(self.alt_input.text())
            self.node.set_parameters([rclpy.parameter.Parameter('target_altitude', rclpy.Parameter.Type.DOUBLE, target_altitude)])
            self.node.get_logger().info(f"Target altitude set to: {target_altitude} m")
        except ValueError:
            self.node.get_logger().error("Invalid altitude input. Please enter a valid number.")
        except AttributeError:
            self.node.get_logger().error("Failed to retrieve current altitude. Ensure the node is correctly subscribed to altitude data.")



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

    def set_alt_pid_parameters(self):
        try:
            kp_alt = float(self.alt_kp_input.text())
            ki_alt = float(self.alt_ki_input.text())
            kd_alt = float(self.alt_kd_input.text())
            max_vertical_speed = float(self.alt_speed_input.text())
            min_vertical_speed = float(self.alt_speedd_input.text())
            altitude_tolerance = float(self.alt_tolerance_input.text())

            self.node.set_parameters([
                rclpy.parameter.Parameter('kp_alt', rclpy.Parameter.Type.DOUBLE, kp_alt),
                rclpy.parameter.Parameter('ki_alt', rclpy.Parameter.Type.DOUBLE, ki_alt),
                rclpy.parameter.Parameter('kd_alt', rclpy.Parameter.Type.DOUBLE, kd_alt),
                rclpy.parameter.Parameter('max_vertical_speed', rclpy.Parameter.Type.DOUBLE, max_vertical_speed),
                rclpy.parameter.Parameter('min_vertical_speed', rclpy.Parameter.Type.DOUBLE, min_vertical_speed),
                rclpy.parameter.Parameter('altitude_tolerance', rclpy.Parameter.Type.DOUBLE, altitude_tolerance)
            ])

            self.node.get_logger().info(
                f"Altitude PID parameters set to: Kp={self.kp_alt}, Ki={self.ki_alt}, Kd={self.kd_alt}, "
                f"Max Speed={self.max_vertical_speed} m/s, Tolerance={self.altitude_tolerance} m"
            )
        except ValueError:
            self.node.get_logger().error("Invalid input for altitude PID parameters. Please enter valid numbers.")

def main(args=None):
    rclpy.init(args=args)
    node = VerticalSpeedControlNode()

    app = QApplication(sys.argv)
    gui = VerticalSpeedControlGUI(node)

    gui.show()

    def spin_node():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

    import threading
    ros_thread = threading.Thread(target=spin_node)
    ros_thread.start()

    sys.exit(app.exec_())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


