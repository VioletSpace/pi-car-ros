import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64MultiArray

from .hardware import RobotHatHardware


class RobotHatNode(Node):

    def __init__(self):
        super().__init__("robot_hat")

        self.hw = RobotHatHardware()

        self.declare_parameter("max_motor_percent", 100.0)
        self.max_motor_percent = (
            self.get_parameter("max_motor_percent")
            .get_parameter_value()
            .double_value
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.servo_sub = self.create_subscription(
            Float64MultiArray,
            "servo_angles",
            self.servo_callback,
            10,
        )

        self.battery_pub = self.create_publisher(
            BatteryState,
            "battery_state",
            10,
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_battery,
        )

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        left = linear - angular
        right = linear + angular

        left *= self.max_motor_percent
        right *= self.max_motor_percent

        self.hw.set_motor_speeds(left, right)

    def servo_callback(self, msg: Float64MultiArray):
        if len(msg.data) > 0:
            self.hw.set_servo(1, msg.data[0])

        if len(msg.data) > 1:
            self.hw.set_servo(2, msg.data[1])

    def publish_battery(self):
        battery = BatteryState()
        battery.voltage = self.hw.battery_voltage()

        self.battery_pub.publish(battery)

    def destroy_node(self):
        self.hw.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init()
    node = RobotHatNode()

    try:
        with rclpy.init(args=args):
            rh_node = RobotHatNode()
            rclpy.spin(rh_node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()