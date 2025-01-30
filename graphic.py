import sys
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout
)
from rclpy.node import Node
from mavros_msgs.msg import VfrHud
import rclpy
import time
import matplotlib.pyplot as plt
from threading import Thread


class UAVMonitorApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UAV Monitor and Control")

        # Initialize ROS Node
        rclpy.init()
        self.node = rclpy.create_node('uav_monitor_node')
        self.node.get_logger().info("ROS Node Initialized. Waiting for data..")
        self.node.create_subscription(VfrHud, '/mavros/vfr_hud', self.vfr_hud_callback, 10)

        # Altitude plot data
        self.altitude_data = []
        self.timestamps = []
        self.start_time = time.time()

        # Initialize the plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], '-o', label='Altitude (m)')
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Altitude (m)")
        self.ax.set_title("UAV Altitude Over Time")
        self.ax.legend()

        # Layouts
        main_layout = QVBoxLayout()
        data_layout = QGridLayout()
        input_layout = QVBoxLayout()
        button_layout = QHBoxLayout()

        # Data Display
        self.altitude_label = QLabel("Altitude:")
        self.vertical_speed_label = QLabel("Vertical Speed:")
        self.ground_speed_label = QLabel("Ground Speed:")
        self.heading_label = QLabel("Heading:")

        self.altitude_value = QLabel("0.0 m")
        self.vertical_speed_value = QLabel("0.0 m/s")
        self.ground_speed_value = QLabel("0.0 m/s")
        self.heading_value = QLabel("0.0°")

        # Adding data to grid layout
        data_layout.addWidget(self.altitude_label, 0, 0)
        data_layout.addWidget(self.altitude_value, 0, 1)
        data_layout.addWidget(self.vertical_speed_label, 1, 0)
        data_layout.addWidget(self.vertical_speed_value, 1, 1)
        data_layout.addWidget(self.ground_speed_label, 2, 0)
        data_layout.addWidget(self.ground_speed_value, 2, 1)
        data_layout.addWidget(self.heading_label, 3, 0)
        data_layout.addWidget(self.heading_value, 3, 1)

        # Input for desired position and altitude
        input_title = QLabel("Set Desired Position and Alt (delta to origin)")
        self.input_x = QLineEdit()
        self.input_x.setPlaceholderText("X:")
        self.input_y = QLineEdit()
        self.input_y.setPlaceholderText("Y:")
        self.input_alt = QLineEdit()
        self.input_alt.setPlaceholderText("Alt:")

        input_layout.addWidget(input_title)
        input_layout.addWidget(self.input_x)
        input_layout.addWidget(self.input_y)
        input_layout.addWidget(self.input_alt)

        # Buttons
        mavros_button = QPushButton("Mavros")
        mavros_button.clicked.connect(self.launch_mavros_in_new_terminal)
        px4_button = QPushButton("PX4_SITL")
        px4_button.clicked.connect(self.launch_px4_in_new_terminal)
        qgc_button = QPushButton("QGC")
        qgc_button.clicked.connect(self.launch_qgc_in_new_terminal)

        button_layout.addWidget(mavros_button)
        button_layout.addWidget(px4_button)
        button_layout.addWidget(qgc_button)

        # Adding all layouts to main layout
        main_layout.addLayout(data_layout)
        main_layout.addLayout(input_layout)
        main_layout.addLayout(button_layout)

        self.setLayout(main_layout)

    def vfr_hud_callback(self, msg):
        """Callback for the VFR_HUD topic."""
        try:
            # 更新顯示的數據
            self.altitude_value.setText(f"{msg.altitude:.2f} m")
            self.vertical_speed_value.setText(f"{msg.climb:.2f} m/s")
            self.ground_speed_value.setText(f"{msg.groundspeed:.2f} m/s")
            self.heading_value.setText(f"{msg.heading:.0f}°")

            # 更新折線圖數據
            current_time = time.time() - self.start_time
            self.timestamps.append(current_time)
            self.altitude_data.append(msg.altitude)

            # 保持數據範圍在 200 秒內
            time_window = 200  # seconds
            while self.timestamps and current_time - self.timestamps[0] > time_window:
                self.timestamps.pop(0)
                self.altitude_data.pop(0)

            # 更新圖表
            self.line.set_data(self.timestamps, self.altitude_data)
            self.ax.set_xlim(max(0, current_time - time_window), current_time)  # 固定 X 軸範圍
            self.ax.relim()
            self.ax.autoscale_view(scaley=True)  # 只縮放 Y 軸
            plt.draw()
        except AttributeError as e:
          self.node.get_logger().warn(f"Invalid VFR_HUD message: {e}")


    def launch_mavros_in_new_terminal(self):
        """Launch the MAVROS process in a new terminal."""
        try:
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c",
                "ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557"
            ])
        except Exception as e:
            self.node.get_logger().error(f"Error launching MAVROS: {e}")

    def launch_px4_in_new_terminal(self):
        """Launch the PX4 process in a new terminal."""
        try:
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c",
                "cd /home/dao/PX4-Autopilot; make px4_sitl gz_x500; exec bash"
            ])
        except Exception as e:
            self.node.get_logger().error(f"Error launching PX4: {e}")

    def launch_qgc_in_new_terminal(self):
        """Launch the QGC process in a new terminal."""
        try:
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c",
                "/home/dao/auto/QGroundControl.AppImage; exec bash"
            ])
        except Exception as e:
            self.node.get_logger().error(f"Error launching QGC: {e}")

    def closeEvent(self, event):
        """Ensure proper ROS cleanup on exit."""
        self.node.destroy_node()
        rclpy.shutdown()
        plt.close(self.fig)  # Close the plot
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    gui = UAVMonitorApp()

    def run_ros():
        while rclpy.ok():
            rclpy.spin_once(gui.node, timeout_sec=0.1)

    ros_thread = Thread(target=run_ros)
    ros_thread.start()

    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

