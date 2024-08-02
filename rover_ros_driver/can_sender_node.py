import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from canlib import canlib
from rover_ros_driver.rover import rover, servo, battery
import time

class CANControlNode(Node):
    def __init__(self):
        super().__init__('can_control_node')
        self.subscription_throttle = self.create_subscription(
            Float32,
            'throttle_send',
            self.throttle_callback,
            1)
        self.subscription_steering = self.create_subscription(
            Float32,
            'steering_send',
            self.steering_callback,
            1)
        self.subscription_battery = self.create_subscription(
            Bool,
            'battery_send',
            self.battery_callback,
            1)
        self.can_channel = self.init_can_channel()
        self.battery_status = False  # バッテリーの初期状態をOFFとする
        self.last_battery_toggle_time = 0  # 最後にバッテリーを切り替えた時間を記録
        
        # 初期状態のスロットルとステアリングをニュートラルに設定
        self.latest_throttle_ = 50.0  # スロットルのニュートラル値を50%に設定（パルス幅1500）
        self.latest_steering_ = 0.0  # ステアリングのニュートラル値を0度に設定

        # 初期状態のスロットルとステアリングの値を送信
        self.send_initial_values()
        
    def init_can_channel(self):
        ch = canlib.openChannel(
            channel=0,
            flags=canlib.Open.REQUIRE_INIT_ACCESS,
            bitrate=canlib.Bitrate.BITRATE_125K,
        )
        ch.setBusOutputControl(canlib.Driver.NORMAL)
        ch.busOn()
        rover.start(ch)
        return ch

    def send_initial_values(self):
        # 初期状態のスロットルとステアリングの値を送信
        frame_throttle = servo.set_throttle_pulse_frame(self.convert_throttle_to_pulse(self.latest_throttle_))
        frame_steering = servo.set_steering_angle_frame(self.latest_steering_)
        self.can_channel.writeWait(frame_throttle, -1)
        self.can_channel.writeWait(frame_steering, -1)
    
    def throttle_callback(self, msg):
        throttle_value = msg.data
        # self.get_logger().info(f'Received throttle value: {throttle_value}')
        # Convert the throttle value to the appropriate pulse width
        pulse_width = self.convert_throttle_to_pulse(throttle_value)
        frame = servo.set_throttle_pulse_frame(pulse_width)
        self.can_channel.writeWait(frame, -1)
    
    def steering_callback(self, msg):
        steering_angle = msg.data
        # self.get_logger().info(f'Received steering angle: {steering_angle}')
        frame = servo.set_steering_angle_frame(steering_angle)
        self.can_channel.writeWait(frame, -1)
    
    def battery_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_battery_toggle_time > 1:  # 1秒の間隔を設ける
            if msg.data:
                self.battery_status = not self.battery_status
                self.get_logger().info(f'Toggling battery status: {"ON" if self.battery_status else "OFF"}')
                if self.battery_status:
                    frame = battery.set_pwr_on_frame
                else:
                    frame = battery.set_pwr_off_frame
                self.can_channel.writeWait(frame, -1)
                self.last_battery_toggle_time = current_time
    
    def convert_throttle_to_pulse(self, throttle_value):
        # Convert throttle percentage (0 to 100) to pulse width (1000 to 2000)
        return int(1000 + (throttle_value / 100.0) * 1000)

def main(args=None):
    rclpy.init(args=args)
    can_control_node = CANControlNode()
    rclpy.spin(can_control_node)
    can_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
