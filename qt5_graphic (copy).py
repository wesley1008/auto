import sys
import subprocess
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout
)
from rclpy.node import Node
from mavros_msgs.msg import VfrHud, GPSRAW
import rclpy
import time
import matplotlib.pyplot as plt
from threading import Thread


class UAVMonitorApp(QWidget):
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UAV Monitor and Control")
        
        self.is_get_first_alt = True
        self.get_first_alt = 0.0
        self.get_first_lon = 0.0
        self.get_first_lat = 0.0
        # Initialize ROS Node
        rclpy.init()
        self.node = rclpy.create_node('uav_monitor_node')
        self.node.get_logger().info("ROS Node Initialized. Waiting for data..")
        self.node.create_subscription(VfrHud, '/mavros/vfr_hud', self.vfr_hud_callback, 10)
        self.node.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self.gps_callback, 10)

        # Altitude and speed plot data
        self.altitude_data = []
        self.speed_data = []
        # self.laltitude_data = []
        # self.longtitude_data = []
        self.timestamps = []
        self.start_time = time.time()

        # Initialize the plots
        plt.ion()
        self.fig, (self.ax_altitude, self.ax_speed) = plt.subplots(2, 1, figsize=(10, 8))

        # Altitude plot setup
        self.line_altitude, = self.ax_altitude.plot([], [], '-o', label='Altitude (m)')
        self.ax_altitude.set_xlabel("Time (s)")
        self.ax_altitude.set_ylabel("Altitude (m)")
        self.ax_altitude.set_title("UAV Altitude Over Time")
        self.ax_altitude.legend()

        # Speed plot setup
        self.line_speed, = self.ax_speed.plot([], [], '-o', label='Vertical Speed (m/s)', color='orange')
        self.ax_speed.set_xlabel("Time (s)")
        self.ax_speed.set_ylabel("Speed (m/s)")
        self.ax_speed.set_title("UAV Speed Over Time")
        self.ax_speed.legend()

        # # Laltitude plot setup
        # self.line_laltitude, = self.ax_laltitude.plot([], [], '-o', label='Laltitude (m)', color='green')
        # self.ax_laltitude.set_xlabel("Time (s)")
        # self.ax_laltitude.set_ylabel("Laltitude (m)")
        # self.ax_laltitude.set_title("UAV Laltitude Over Time")
        # self.ax_laltitude.legend()

        # # Longtitude plot setup
        # self.line_longtitude, = self.ax_longtitude.plot([], [], '-o', label='Longtitude (m)', color='black')
        # self.ax_longtitude.set_xlabel("Time (s)")
        # self.ax_longtitude.set_ylabel("Longtitude (m)")
        # self.ax_longtitude.set_title("UAV Longtitude Over Time")
        # self.ax_longtitude.legend()

        # Layouts
        main_layout = QVBoxLayout()
        data_layout = QGridLayout()
        input_layout = QVBoxLayout()
        button_layout = QHBoxLayout()

        # Data Display
        self.altitude_label = QLabel("Altitude:")
        self.x_label = QLabel("x_position:")
        self.y_label = QLabel("y_position:")
        self.vertical_speed_label = QLabel("Vertical Speed:")
        self.ground_speed_label = QLabel("Ground Speed:")
        self.heading_label = QLabel("Heading:")

        self.altitude_value = QLabel("0.0 m")
        self.x_value = QLabel("0.0 m")
        self.y_value = QLabel("0.0 m")
        self.vertical_speed_value = QLabel("0.0 m/s")
        self.ground_speed_value = QLabel("0.0 m/s")
        self.heading_value = QLabel("0.0°")

        # Adding data to grid layout
        data_layout.addWidget(self.altitude_label, 0, 0)
        data_layout.addWidget(self.altitude_value, 0, 1)
        data_layout.addWidget(self.x_label, 0, 2)
        data_layout.addWidget(self.x_value, 0, 3)
        data_layout.addWidget(self.vertical_speed_label, 1, 0)
        data_layout.addWidget(self.vertical_speed_value, 1, 1)
        data_layout.addWidget(self.y_label, 1, 2)
        data_layout.addWidget(self.y_value, 1, 3)
        data_layout.addWidget(self.ground_speed_label, 2, 0)
        data_layout.addWidget(self.ground_speed_value, 2, 1)
        data_layout.addWidget(self.heading_label, 2, 2)
        data_layout.addWidget(self.heading_value, 2, 3)

        # Input for desired position and altitude
        # input_title = QLabel("Set Desired Position and Alt (delta to origin)")
        # self.input_x = QLineEdit()
        # self.input_x.setPlaceholderText("X:")
        # self.input_y = QLineEdit()
        # self.input_y.setPlaceholderText("Y:")
        # self.input_alt = QLineEdit()
        # self.input_alt.setPlaceholderText("Alt:")

        # input_layout.addWidget(input_title)
        # input_layout.addWidget(self.input_x)
        # input_layout.addWidget(self.input_y)
        # input_layout.addWidget(self.input_alt)

        # Buttons
        start_all = QPushButton("ALL")
        start_all.clicked.connect(self.launch_all)
        sim_button = QPushButton("Gazebo")
        sim_button.clicked.connect(self.launch_gazebo_sim_terminal)
        mavros_button = QPushButton("Mavros")
        mavros_button.clicked.connect(self.launch_mavros_in_new_terminal)
        mavros_service_button = QPushButton("Mavros Service call")
        mavros_service_button.clicked.connect(self.launch_mavros_service_in_new_terminal)
        sitl_button = QPushButton("SITL")
        sitl_button.clicked.connect(self.launch_sitl_in_new_terminal)
        qgc_button = QPushButton("QGC")
        qgc_button.clicked.connect(self.launch_qgc_in_new_terminal)

        button_layout.addWidget(start_all)
        button_layout.addWidget(sim_button)
        button_layout.addWidget(sitl_button)
        button_layout.addWidget(mavros_button)
        button_layout.addWidget(mavros_service_button)
        button_layout.addWidget(qgc_button)

        # Adding all layouts to main layout
        main_layout.addLayout(data_layout)
        main_layout.addLayout(input_layout)
        main_layout.addLayout(button_layout)

        # Add matplotlib figure to layout
        self.canvas = plt.get_current_fig_manager().canvas
        self.canvas.setParent(self)
        main_layout.addWidget(self.canvas)

        self.setLayout(main_layout)  


    def gps_callback(self, msg):
        if self.is_get_first_alt:
            self.get_first_alt = msg.alt
            self.get_first_lon = msg.lon
            self.get_first_lat = msg.lat
            self.is_get_first_alt = False
            self.node.get_logger().info(f"Home Position: Lat={self.get_first_lat:.6f}, Lon={self.get_first_lon:.6f}, Alt={self.get_first_alt:.2f}m")

        try:
            longitude = ((msg.lon) - (self.get_first_lon))/100
            latitude = (msg.lat-self.get_first_lat)/100
            self.x_value.setText(f"{latitude:.2f} m")
            self.y_value.setText(f"{longitude:.2f} m")

            # # 更新折線圖數據
            # current_time = time.time() - self.start_time
            # self.laltitude_data.append(latitude)
            # self.longtitude_data.append(longitude)
            # # 保持數據範圍在 200 秒內
            # time_window = 100  # seconds
            # while self.timestamps and current_time - self.timestamps[0] > time_window:
            #     self.timestamps.pop(0)
            #     self.laltitude_data.pop(0)
            #     self.longtitude_data.pop(0)
            # # 更新高度圖表
            # self.line_laltitude.set_data(self.timestamps, self.laltitude_data)
            # self.ax_laltitude.set_xlim(max(0, current_time - time_window), current_time)  # 固定 X 軸範圍
            # self.ax_laltitude.relim()
            # self.ax_laltitude.autoscale_view(scaley=True)  # 只縮放 Y 軸

            # # 更新速度圖表
            # self.line_longtitude.set_data(self.timestamps, self.longtitude_data)
            # self.ax_longtitude.set_xlim(max(0, current_time - time_window), current_time)  # 固定 X 軸範圍
            # self.ax_longtitude.relim()
            # self.ax_longtitude.autoscale_view(scaley=True)  # 只縮放 Y 軸

            # self.canvas.draw()
                        # 更新折線圖數據
            current_time = time.time() - self.start_time
            self.timestamps.append(current_time)
            self.altitude_data.append(latitude)
            self.speed_data.append(longitude)

            # 保持數據範圍在 200 秒內
            time_window = 100  # seconds
            while self.timestamps and current_time - self.timestamps[0] > time_window:
                self.timestamps.pop(0)
                self.altitude_data.pop(0)
                self.speed_data.pop(0)

            # 更新高度圖表
            self.line_altitude.set_data(self.timestamps, self.altitude_data)
            self.ax_altitude.set_xlim(max(0, current_time - time_window), current_time)  # 固定 X 軸範圍
            self.ax_altitude.relim()
            self.ax_altitude.autoscale_view(scaley=True)  # 只縮放 Y 軸

            # 更新速度圖表
            self.line_speed.set_data(self.timestamps, self.speed_data)
            self.ax_speed.set_xlim(max(0, current_time - time_window), current_time)  # 固定 X 軸範圍
            self.ax_speed.relim()
            self.ax_speed.autoscale_view(scaley=True)  # 只縮放 Y 軸

            self.canvas.draw()
        except AttributeError as e:
            self.node.get_logger().warn(f"Invalid GPS message: {e}")

    # 先退休的VFR_HUB

    def vfr_hud_callback(self, msg):
        """Callback for the VFR_HUD topic."""
        try:
            # 更新顯示的數據
            self.altitude_value.setText(f"{msg.altitude - 584.18:.2f} m")
            self.vertical_speed_value.setText(f"{msg.climb:.2f} m/s")
            self.ground_speed_value.setText(f"{msg.groundspeed:.2f} m/s")
            self.heading_value.setText(f"{msg.heading:.0f}°")

            # # 更新折線圖數據
            # current_time = time.time() - self.start_time
            # self.timestamps.append(current_time)
            # self.altitude_data.append(msg.altitude - 584 - 0.180)
            # self.speed_data.append(msg.climb)

            # # 保持數據範圍在 200 秒內
            # time_window = 100  # seconds
            # while self.timestamps and current_time - self.timestamps[0] > time_window:
            #     self.timestamps.pop(0)
            #     self.altitude_data.pop(0)
            #     self.speed_data.pop(0)

            # # 更新高度圖表
            # self.line_altitude.set_data(self.timestamps, self.altitude_data)
            # self.ax_altitude.set_xlim(max(0, current_time - time_window), current_time)  # 固定 X 軸範圍
            # self.ax_altitude.relim()
            # self.ax_altitude.autoscale_view(scaley=True)  # 只縮放 Y 軸

            # # 更新速度圖表
            # self.line_speed.set_data(self.timestamps, self.speed_data)
            # self.ax_speed.set_xlim(max(0, current_time - time_window), current_time)  # 固定 X 軸範圍
            # self.ax_speed.relim()
            # self.ax_speed.autoscale_view(scaley=True)  # 只縮放 Y 軸

            # self.canvas.draw()
        except AttributeError as e:
          self.node.get_logger().warn(f"Invalid VFR_HUD message: {e}")

   

    def launch_mavros_in_new_terminal(self):
        """Launch the MAVROS process in a new terminal."""
        try:
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c",
                "ros2 launch mavros apm.launch fcu_url:=TCP://:5762"
            ])
        except Exception as e:
            self.node.get_logger().error(f"Error launching MAVROS: {e}")
    
    def launch_mavros_service_in_new_terminal(self):
        """Launch the MAVROS Server."""
        try:
            subprocess.run([
                "gnome-terminal", "--", "bash", "-c",
                'ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: 1}"; exec bash'
            ])
        except Exception as e:
            self.node.get_logger().error(f"Error launching MAVROS Server: {e}")

    def launch_sitl_in_new_terminal(self):
        """Launch the sitl process in a new terminal."""
        try:
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c",
                "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console; exec bash"
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
        
    def launch_gazebo_sim_terminal(self):
        """Launch the MAVROS process in a new terminal."""
        try:
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c",
                "gz sim -v4 -r iris_runway.sdf"
            ])
        except Exception as e:
            self.node.get_logger().error(f"Error launching MAVROS: {e}")
    
    def launch_all(self):
        """Launch the all process."""
        try:
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c",
                "gz sim -v4 -r iris_runway.sdf"
            ])
        except Exception as e:
            self.node.get_logger().error(f"Error launching MAVROS: {e}")
        time.sleep(3)
        try:
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c",
                "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console; exec bash"
            ])
        except Exception as e:
            self.node.get_logger().error(f"Error launching PX4: {e}")
        time.sleep(2.2)
        try:
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c",
                "ros2 launch mavros apm.launch fcu_url:=TCP://:5762"
            ])
        except Exception as e:
            self.node.get_logger().error(f"Error launching MAVROS: {e}")
            time.sleep(0.5)
        try:
            subprocess.Popen([
                "gnome-terminal", "--", "bash", "-c",
                'ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: 1}"; exec bash'
            ])
        except Exception as e:
            self.node.get_logger().error(f"Error launching MAVROS: {e}")
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
