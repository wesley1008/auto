import rclpy
from vertical_speed_control import VerticalSpeedControlNode, VerticalSpeedControlGUI
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout
import threading
import time
import matplotlib.pyplot as plt
import sys


class CombinedControlGUI(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        # 預設參數(speed)
        self.kp_alt = 0.3
        self.ki_alt = 0.01
        self.kd_alt = 0.05
        self.max_vertical_speed = 3.0
        self.altitude_tolerance = 0.2

        self.timestamps = []  # 時間戳
        self.target_speeds = []  # target_vertical_speed 數據

        # 初始化 Matplotlib 圖表
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
        self.setWindowTitle("Combined Control: Vertical Speed & Altitude")

        layout = QVBoxLayout()

        # VS control---------------------------------------------------------------------------------
        # Target Vertical Speed Control
        self.vs_label = QLabel("Target Vertical Speed (m/s):")
        self.vs_input = QLineEdit()
        self.vs_input.setText(str(self.node.get_parameter('target_vertical_speed').value))
        self.set_vs_button = QPushButton("Set Vertical Speed")
        self.set_vs_button.clicked.connect(self.set_vertical_speed)

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

        # Altitude control---------------------------------------------------------------------------------
        # Altitude PID Control
        self.alt_kp_label = QLabel("Kp (Altitude):")
        self.alt_kp_input = QLineEdit()
        self.alt_kp_input.setText(str(self.kp_alt))
        self.alt_ki_label = QLabel("Ki (Altitude):")
        self.alt_ki_input = QLineEdit()
        self.alt_ki_input.setText(str(self.ki_alt))
        self.alt_kd_label = QLabel("Kd (Altitude):")
        self.alt_kd_input = QLineEdit()
        self.alt_kd_input.setText(str(self.kd_alt))
        self.alt_speed_label = QLabel("Max Vertical Speed (m/s):")
        self.alt_speed_input = QLineEdit()
        self.alt_speed_input.setText(str(self.max_vertical_speed))
        self.alt_tolerance_label = QLabel("Altitude Tolerance (m):")
        self.alt_tolerance_input = QLineEdit()
        self.alt_tolerance_input.setText(str(self.altitude_tolerance))
        self.set_alt_pid_button = QPushButton("Set Altitude Parameters")
        self.set_alt_pid_button.clicked.connect(self.set_alt_pid_parameters)

        # Altitude Control
        self.alt_label = QLabel("Target Altitude (m):")
        self.alt_input = QLineEdit()
        self.alt_input.setPlaceholderText("Enter target altitude...")
        self.set_alt_button = QPushButton("Set Altitude")
        self.set_alt_button.clicked.connect(self.set_altitude)

        # Target Vertical Speed Control
        self.vs_label = QLabel("Target Vertical Speed (m/s):")
        self.vs_input = QLineEdit()
        self.vs_input.setText(str(self.node.get_parameter('target_vertical_speed').value))
        self.set_vs_button = QPushButton("Set Vertical Speed")
        self.set_vs_button.clicked.connect(self.set_vertical_speed)

        # Add all widgets to the layout
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

        layout.addWidget(self.alt_kp_label)
        layout.addWidget(self.alt_kp_input)
        layout.addWidget(self.alt_ki_label)
        layout.addWidget(self.alt_ki_input)
        layout.addWidget(self.alt_kd_label)
        layout.addWidget(self.alt_kd_input)
        layout.addWidget(self.alt_speed_label)
        layout.addWidget(self.alt_speed_input)
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
            self.kp_alt = float(self.alt_kp_input.text())
            self.ki_alt = float(self.alt_ki_input.text())
            self.kd_alt = float(self.alt_kd_input.text())
            self.max_vertical_speed = float(self.alt_speed_input.text())
            self.altitude_tolerance = float(self.alt_tolerance_input.text())

            self.node.get_logger().info(
                f"Altitude PID parameters set to: Kp={self.kp_alt}, Ki={self.ki_alt}, Kd={self.kd_alt}, "
                f"Max Speed={self.max_vertical_speed} m/s, Tolerance={self.altitude_tolerance} m"
            )
        except ValueError:
            self.node.get_logger().error("Invalid input for altitude PID parameters. Please enter valid numbers.")

    def update_plot(self):
        """更新折線圖"""
        current_time = time.time()
        target_speed = self.node.get_parameter('target_vertical_speed').value

        # 初次記錄起始時間
        if not self.timestamps:
            self.start_time = current_time

        # 添加新數據
        self.timestamps.append(current_time - self.start_time)
        self.target_speeds.append(target_speed)

        # 保留最近 100 個數據點
        if len(self.timestamps) > 100:
            self.timestamps.pop(0)
            self.target_speeds.pop(0)

        # 更新圖表數據
        self.line.set_xdata(self.timestamps)
        self.line.set_ydata(self.target_speeds)
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()


    def set_altitude(self):
        try:
            # 獲取目標高度
            target_altitude = float(self.alt_input.text())
            self.node.get_logger().info(f"Target altitude set to: {target_altitude} m")

            # 獲取當前高度
            current_altitude = self.node.altitude

            # 計算高度誤差
            altitude_error = target_altitude - current_altitude

            # 積分項限幅
            self.node.error_sum += altitude_error * 0.05  # 0.05 為控制時間間隔 (秒)
            integral_limit = 10.0  # 限制積分項範圍
            self.node.error_sum = max(-integral_limit, min(integral_limit, self.node.error_sum))

            # 微分項計算
            altitude_derivative = (altitude_error - self.node.last_error) / 0.05
            self.node.last_error = altitude_error

            # PID 控制計算
            target_vertical_speed = (
                self.kp_alt * altitude_error +  # 比例項
                self.ki_alt * self.node.error_sum +  # 積分項
                self.kd_alt * altitude_derivative  # 微分項
            )

            # 限制目標垂直速度
            target_vertical_speed = max(-self.max_vertical_speed, min(self.max_vertical_speed, target_vertical_speed))

            # 判斷是否進入穩定範圍
            if abs(altitude_error) < self.altitude_tolerance:
                target_vertical_speed = 0.0
                self.node.get_logger().info("Altitude reached. Holding position.")

            # 更新節點中的目標垂直速度
            # self.node.set_parameters([
            #     rclpy.parameter.Parameter('target_vertical_speed', rclpy.Parameter.Type.DOUBLE, target_vertical_speed)
                
            # ])
            
            self.node.get_logger().info(
                f"Altitude Error: {altitude_error:.2f}, Target VS: {target_vertical_speed:.2f} m/s"
            )
        except ValueError:
            self.node.get_logger().error("Invalid altitude input. Please enter a valid number.")
        except AttributeError:
            self.node.get_logger().error("Failed to retrieve current altitude. Ensure the node is correctly subscribed to altitude data.")




def main():
    rclpy.init()
    node = VerticalSpeedControlNode()

    app = QApplication(sys.argv)
    gui = CombinedControlGUI(node)

    from PyQt5.QtCore import QTimer
    timer = QTimer()
    timer.timeout.connect(gui.update_plot)
    timer.start(100)  

    gui.show()

    def spin_node():
        while rclpy.ok():
            gui.set_altitude()
            rclpy.spin_once(node, timeout_sec=0.1)

    ros_thread = threading.Thread(target=spin_node)
    ros_thread.start()

    sys.exit(app.exec_())

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

