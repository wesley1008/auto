import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import FluidPressure  # 訂閱氣壓計數據
import time

class BarometerAltitudeControl(Node):
    def __init__(self):
        super().__init__('barometer_altitude_control_node')
        
        # 發佈原始位置控制訊息
        self.raw_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

        # 訂閱氣壓計數據
        self.barometer_sub = self.create_subscription(
            FluidPressure,
            '/mavros/imu/static_pressure',
            self.barometer_callback,
            10
        )



        self.current_pressure = None  # 紀錄目前氣壓值

        # 服務客戶端：用來解鎖 (arming) 和設置飛行模式
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # 等待服務啟動
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 arming 服務啟動...')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 set_mode 服務啟動...')

        # 設定飛行模式為 GUIDED_NOGPS 並解鎖
        self.set_mode('GUIDED_NOGPS')
        self.arm()

        # 給無人機一點時間啟動
        time.sleep(2)

        # 控制飛行高度
        #self.set_altitude(30.0)  # 上升到 10 公尺
        #time.sleep(10)
        #self.set_altitude(5.0)   # 降到 5 公尺
        #time.sleep(5)
        #self.set_altitude(1.0)   # 降到 1 公尺 (準備降落)
        self.create_timer(1.5, lambda: self.set_altitude(30.0))
        


    def barometer_callback(self, msg):
        """接收氣壓計數據"""
        self.current_pressure = msg.fluid_pressure
        self.get_logger().info(f'目前氣壓值: {self.current_pressure:.2f} Pa')

    def set_mode(self, mode):
        """設定 UAV 飛行模式為 GUIDED_NOGPS"""
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().mode_sent:
            self.get_logger().info(f'飛行模式已切換到 {mode}')
        else:
            self.get_logger().error(f'無法切換飛行模式到 {mode}')

    def arm(self):
        """解鎖 UAV"""
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('UAV 已解鎖')
        else:
            self.get_logger().error('解鎖失敗')

    def set_altitude(self, target_altitude):
        """使用 setpoint_raw 設定 UAV 目標高度"""
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'

        # 設定要控制的高度，忽略位置與速度
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )

        # 保持當前的 x, y 位置，僅調整 z (高度)
        target.position.x = 0.0
        target.position.y = 0.0
        target.position.z = target_altitude  # 以公尺為單位指定目標高度

        self.raw_pub.publish(target)
        self.get_logger().info(f'目標高度設定為 {target_altitude} 公尺')

def main(args=None):
    rclpy.init(args=args)
    barometer_altitude_control = BarometerAltitudeControl()
    rclpy.spin(barometer_altitude_control)
    barometer_altitude_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

