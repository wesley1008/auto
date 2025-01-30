import rclpy
from vertical_speed_control import VerticalSpeedControlNode, VerticalSpeedControlGUI
from PyQt5.QtWidgets import QApplication
import sys

def main():
    rclpy.init()
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


if __name__ == "__main__":
    main()
