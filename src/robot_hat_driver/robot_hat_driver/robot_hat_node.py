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
        params = {
            "mmaxpercent": self.get_parameter("max_motor_percent").get_parameter_value().double_value,
            "lmid": self.get_parameter('motor_left_id').get_parameter_value().integer_value,
            "rmid": self.get_parameter('motor_right_id').get_parameter_value().integer_value,
            "lmrev": self.get_parameter('motor_left_reversed').get_parameter_value().bool_value,
            "rmrev": self.get_parameter('motor_right_reversed').get_parameter_value().bool_value
        }
        self.get_logger().info(
            'Starting Robot Hat Driver with max_motor_percent: %f, motor_left_id: %d, motor_right_id: %d, motor_left_reversed: %r, motor_right_reversed: %r'
            % (params["mmaxpercent"], params["lmid"], params["rmid"], params["lmrev"], params["rmrev"])
            )
        
        self.hw = RobotHatHardware(params, self.get_logger())

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        #self.servo_sub = self.create_subscription(
        #    Float64MultiArray,
        #    "servo_angles",
        #    self.servo_callback,
        #    10,
        #)

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

        left *= self.mmaxpercent
        right *= self.mmaxpercent

        self.hw.set_motor_speeds(left, right)

    #def servo_callback(self, msg: Float64MultiArray):
    #    if len(msg.data) > 0:
    #        self.hw.set_servo(1, msg.data[0])
    #    if len(msg.data) > 1:
    #        self.hw.set_servo(2, msg.data[1])

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