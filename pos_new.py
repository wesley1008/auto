import time
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget     
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped as PS,TwistStamped as TW

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error):
        dt = 0.05  # 控制周期
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class XYPositionController:
    def __init__(self, attitude_publisher):
        self.vx_pid = PID(kp=0.05, ki=0.0, kd=0.0)  # X 方向速度 PID
        self.vy_pid = PID(kp=0.05, ki=0.0, kd=0.0)  # Y 方向速度 PID
        self.attitude_publisher = attitude_publisher

    def control_xy(self, vx, vy):
        """ 计算 Pitch/Roll 角度，让 UAV XY 速度趋近 0 """
        pitch_correction = -self.vx_pid.update(vx)  # 前后倾斜 (Pitch)
        roll_correction = self.vy_pid.update(vy)  # 左右倾斜 (Roll)

        max_angle = np.deg2rad(10)  # 限制最大 10° 翻滚角度
        pitch_correction = np.clip(pitch_correction, -max_angle, max_angle)
        roll_correction = np.clip(roll_correction, -max_angle, max_angle)

        # 转换为四元数
        quat = R.from_euler('xyz', [roll_correction, pitch_correction, 0]).as_quat()
        print(f"ox: {float(quat[0]):.10f}, oy: {float(quat[1]):.10f}, oz: {float(quat[2]):.10f}, ow: {float(quat[3]):.10f}")
        
        # 发送姿态控制指令
        attitude_target = AttitudeTarget()
        attitude_target.orientation.x = quat[0]
        attitude_target.orientation.y = quat[1]
        attitude_target.orientation.z = quat[2]
        attitude_target.orientation.w = quat[3]
        attitude_target.type_mask = 7
        attitude_target.thrust = 0.5  # 保持当前高度

        self.attitude_publisher.publish(attitude_target)
        # print(f"Roll: {roll_correction}, Pitch: {pitch_correction}, vx: {vx: 4f}, vy: {vy: 4f}")

class UAVXYHoverNode(Node):
    def __init__(self):
        super().__init__('uav_xy_hover')

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.attitude_publisher = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', qos_profile)
        self.imu_subscription = self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback, qos_profile)
        self.pos_subscription = self.create_subscription(PS, '/mavros/local_position/pose', self.pose_callback, qos_profile)
        self.vel_subscription = self.create_subscription(TW, '/mavros/local_position/velocity_local', self.vel_callback, qos_profile)

        
        self.vx = 0.0
        self.vy = 0.0
        self.ax = 0.0
        self.ay = 0.0
        self.last_time = time.time()
        self.xy_controller = XYPositionController(self.attitude_publisher)

        self.timer = self.create_timer(0.05, self.control_xy_hover)

    def imu_callback(self, imu_msg):
        """ 计算 UAV XY 速度 """
        self.ax = imu_msg.linear_acceleration.x
        self.ay = imu_msg.linear_acceleration.y

        current_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        # current_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec
        dt = current_time - self.last_time
        # if dt < 0.165:
        #     return

        self.last_time = current_time

        self.vx += self.ax * dt
        self.vy += self.ay * dt

    def pose_callback(self, msg):
        ""

    def vel_callback(self, msg):
        ""
        # self.vx = float(f"{msg.twist.linear.x:.4f}")
        # self.vy = float(f"{msg.twist.linear.y:.4f}")


    def control_xy_hover(self):
        """ 计算 Pitch/Roll 角度，稳定 XY 位置 """
        self.xy_controller.control_xy(self.vx, self.vy)
        # self.get_logger().info(f"vx: {self.vx}, vy: {self.vy}, ax: {self.ax}, ay: {self.ay}")
        print(f"vx: {self.vx}, vy: {self.vy}")


def main(args=None):
    rclpy.init(args=args)
    node = UAVXYHoverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
