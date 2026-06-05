# Copyright (c) 2026, Johanna Pluschke
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64MultiArray

from .hardware import RobotHatHardware


class RobotHatNode(Node):

    def __init__(self):
        super().__init__("robot_hat_node")
        
        self.declare_parameter("max_motor_percent", 100.0)
        self.declare_parameter("motor_left_id", 1)
        self.declare_parameter("motor_right_id", 2)
        self.declare_parameter("motor_left_reversed", False)
        self.declare_parameter("motor_right_reversed", False)
        self.declare_parameter("servo_channels", ["-1"])
        params = {
            "mmaxpercent": self.get_parameter("max_motor_percent").get_parameter_value().double_value,
            "lmid": self.get_parameter('motor_left_id').get_parameter_value().integer_value,
            "rmid": self.get_parameter('motor_right_id').get_parameter_value().integer_value,
            "lmrev": self.get_parameter('motor_left_reversed').get_parameter_value().bool_value,
            "rmrev": self.get_parameter('motor_right_reversed').get_parameter_value().bool_value,
            "servo_channels": self.get_parameter('servo_channels').get_parameter_value().string_array_value
        }
        self.get_logger().info(
            'Starting Robot Hat Driver with max_motor_percent: %f, motor_left_id: %d, motor_right_id: %d, motor_left_reversed: %r, motor_right_reversed: %r'
            % (params["mmaxpercent"], params["lmid"], params["rmid"], params["lmrev"], params["rmrev"])
            )
        
        self.hw = RobotHatHardware(params, self.get_logger())

        self.servo_sub = self.create_subscription(
            Float64MultiArray,
            "servo_target_angles",
            self.servo_callback,
            10,
        )

        self.battery_pub = self.create_publisher(
            BatteryState,
            "battery_state",
            10,
        )
        self.timer = self.create_timer(1.0, self.publish_battery)

        self.servo_angle_pub = self.create_publisher(
            Float64MultiArray,
            "servo_angles",
            10
        )
        self.timer = self.create_timer(0.01, self.publish_servo_angles)
        self.get_logger().info('Node ready')

    def servo_callback(self, msg: Float64MultiArray):
        if len(msg.data) != len(self.hw.servos):
            self.get_logger().warn(
                "Received mismatching servo target angles: %d angles for %d servos. Ignoring."
                % (len(msg.data), len(self.hw.servos))
            )
            return
        for i,angle in enumerate(msg.data):
            self.hw.set_servo(i, angle)

    def publish_battery(self):
        battery = BatteryState()
        battery.voltage = self.hw.battery_voltage()

        self.battery_pub.publish(battery)

    def publish_servo_angles(self):
        self.servo_angle_pub.publish([float(s.target_angle) for s in self.hw.servos])

    def destroy_node(self):
        self.hw.stop()
        super().destroy_node()


def main(args=None):
    try:
        with rclpy.init(args=args):
            rh_node = RobotHatNode()
            rclpy.spin(rh_node)
    except KeyboardInterrupt:
        pass

    rh_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()