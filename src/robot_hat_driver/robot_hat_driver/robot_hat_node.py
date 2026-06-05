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

import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Image, Range
from std_msgs.msg import Bool, Float64, Float64MultiArray

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
        self.declare_parameter("ultrasonic_sensor", False)
        self.declare_parameter("ultrasonic_pins", ["-1"])
        self.declare_parameter("grayscale_sensor", False)
        self.declare_parameter("grayscale_pins", ["-1"])
        params = {
            "mmaxpercent": self.get_parameter("max_motor_percent").get_parameter_value().double_value,
            "lmid": self.get_parameter('motor_left_id').get_parameter_value().integer_value,
            "rmid": self.get_parameter('motor_right_id').get_parameter_value().integer_value,
            "lmrev": self.get_parameter('motor_left_reversed').get_parameter_value().bool_value,
            "rmrev": self.get_parameter('motor_right_reversed').get_parameter_value().bool_value,
            "servo_channels": self.get_parameter('servo_channels').get_parameter_value().string_array_value,
            "us_s": self.get_parameter('ultrasonic_sensor').get_parameter_value().bool_value,
            "us_pins": self.get_parameter('ultrasonic_pins').get_parameter_value().string_array_value,
            "gs_s": self.get_parameter('grayscale_sensor').get_parameter_value().bool_value,
            "gs_pins": self.get_parameter('grayscale_pins').get_parameter_value().string_array_value
        }
        self.get_logger().info(
            'Starting Robot Hat Driver with max_motor_percent: %f, motor_left_id: %d, motor_right_id: %d, motor_left_reversed: %r, motor_right_reversed: %r'
            % (params["mmaxpercent"], params["lmid"], params["rmid"], params["lmrev"], params["rmrev"])
            )
        if params["mmaxpercent"] > 100.0:
            self.get_logger().warn("max_motor_percent %f exceeds maximum of 100.0" % params["mmaxpercent"])
        
        self.hw = RobotHatHardware(params, self.get_logger())
        self.sensors_active = True

        self.led_sub = self.create_subscription(
            Bool,
            "robot_hat_led",
            self.led_callback,
            10,
        )
        
        self.servo_sub = self.create_subscription(
            Float64MultiArray,
            "servo_target_angles",
            self.servo_callback,
            10,
        )

        self.motor_sub = self.create_subscription(
            Float64,
            "motor_speed",
            self.motor_callback,
            10,
        )

        self.battery_pub = self.create_publisher(
            BatteryState,
            "battery_state",
            10,
        )
        self.timer = self.create_timer(1.0, self.publish_battery)

        if params["us_s"]:
            self.us_pub = self.create_publisher(Range, "sonar_range", 10)
            self.timer = self.create_timer(0.5, self.publish_sonar)
        
        if params["gs_s"]:
            self.gs_pub = self.create_publisher(Image, "grayscale", 10)
            self.timer = self.create_timer(0.1, self.publish_grayscale)

        self.servo_angle_pub = self.create_publisher(
            Float64MultiArray,
            "servo_angles",
            10
        )
        self.timer = self.create_timer(0.01, self.publish_servo_angles)
        self.get_logger().info('Node ready')

    def led_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info("Activating Robot HAT LED")
        else:
            self.get_logger().info("Deactivating Robot HAT LED")
        self.hw.led(msg.data)

    def servo_callback(self, msg: Float64MultiArray):
        if len(msg.data) != len(self.hw.servos):
            self.get_logger().warn(
                "Received mismatching servo target angles: %d angles for %d servos. Ignoring."
                % (len(msg.data), len(self.hw.servos))
            )
            return
        for i,angle in enumerate(msg.data):
            self.hw.set_servo(i, angle)

    def motor_callback(self, msg: Float64):
        self.hw.set_motor_speeds(msg.data, msg.data)

    def publish_battery(self):
        volt = self.hw.battery_voltage()
        perc = min(1.0, (volt - 4.0) / 4.4)
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.header.frame_id = 'base_link'
        battery.voltage = volt
        battery.charge = 6.24 * perc
        battery.design_capacity = 6.24
        battery.percentage = perc

        self.battery_pub.publish(battery)

    def publish_servo_angles(self):
        msg = Float64MultiArray()
        msg.data = [float(s.target_angle) for s in self.hw.servos]
        self.servo_angle_pub.publish(msg)

    def publish_sonar(self):
        if not self.sensors_active:
            return
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sonar_sensor'
        msg.radiation_type = Range.ULTRASOUND # Or 0
        msg.field_of_view = 0.52  # ~30 degrees
        msg.min_range = 0.02
        msg.max_range = 6.0
        msg.range = self.hw.ultrasonic.read()
        
        self.us_pub.publish(msg)

    def publish_grayscale(self):
        if not self.sensors_active:
            return
        data = self.hw.grayscale.read()
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'grayscale_sensor'
        msg.height = 1
        msg.width = 3
        msg.encoding = 'mono16'
        msg.is_bigendian = True
        msg.step = 6
        msg.data = struct.pack('>3H', *data)
        
        self.gs_pub.publish(msg)


    def destroy_node(self):
        self.hw.stop()
        super().destroy_node()


def main(args=None):
    try:
        with rclpy.init(args=args):
            rh_node = RobotHatNode()
            rclpy.spin(rh_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    rh_node.destroy_node()


if __name__ == "__main__":
    main()