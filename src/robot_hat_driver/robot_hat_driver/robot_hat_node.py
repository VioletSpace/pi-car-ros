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
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Image, Range
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray
from std_srvs.srv import Trigger, SetBool
from .hardware import RobotHatHardware

from picar_interfaces.msg import ServoCmd


class RobotHatNode(Node):

    def __init__(self):
        super().__init__("robot_hat_node")
        # Parameters
        self.declare_parameter("max_motor_percent", 100.0)
        self.declare_parameter("motor_left_id", 1)
        self.declare_parameter("motor_right_id", 2)
        self.declare_parameter("motor_left_reversed", False)
        self.declare_parameter("motor_right_reversed", False)
        self.declare_parameter("servo_channels", ["-1"])
        self.declare_parameter("servo_correction", [0.0])
        self.declare_parameter("ultrasonic_sensor", False)
        self.declare_parameter("ultrasonic_pins", ["-1"])
        self.declare_parameter("grayscale_sensor", False)
        self.declare_parameter("grayscale_pins", ["-1"])
        self.declare_parameter("grayscale_calibration", [1495, 1481, 1457, 1429, 1378, 1067])
        self.params = {
            "mmaxpercent": self.get_parameter("max_motor_percent").value,
            "lmid": self.get_parameter('motor_left_id').value,
            "rmid": self.get_parameter('motor_right_id').value,
            "lmrev": self.get_parameter('motor_left_reversed').value,
            "rmrev": self.get_parameter('motor_right_reversed').value,
            "servo_channels": self.get_parameter('servo_channels').value,
            "servo_corr": self.get_parameter('servo_correction').value,
            "us_s": self.get_parameter('ultrasonic_sensor').value,
            "us_pins": self.get_parameter('ultrasonic_pins').value,
            "gs_s": self.get_parameter('grayscale_sensor').value,
            "gs_pins": self.get_parameter('grayscale_pins').value,
            "gs_cal": self.get_parameter('grayscale_calibration').value
        }
        # Sanity check parameters:
        # Motor precentage range
        if self.params["mmaxpercent"] > 100.0 or self.params["mmaxpercent"] < 0.0:
            self.get_logger().warn("max_motor_percent %f exceeds maximum of 100.0. Clamping." % self.params["mmaxpercent"])
            self.params["mmaxpercent"] = max(0.0, min(100.0, self.params["mmaxpercent"]))
        # Correct servo setup
        lch, lcorr = len(self.params["servo_channels"]), len(self.params["servo_corr"])
        if lch > lcorr:
            self.get_logger().warn("Insufficient number of servo correction entries for %d servos. Filling with 0." % lch)
            self.params["servo_corr"].extend([0.0] * (lch-lcorr))
        # Correct sensor setup
        if self.params["us_s"] and not len(self.params["us_pins"]) == 2:
            self.get_logger().warn(f"Sonar sensor active but misconfigured: {len(self.params["us_pins"])} out of 2 channel entries. Deactivating.")
            self.params["us_s"] = False
        if self.params["gs_s"] and not len(self.params["gs_pins"]) == 3:
            self.get_logger().warn(f"Grayscale sensor active but misconfigured: {len(self.params["gs_pins"])} out of 3 channel entries. Deactivating.")
            self.params["gs_s"] = False
        
        self.get_logger().info(f'Starting Robot HAT Driver with {self.params}')
        
        # Initialise hardware with parameters
        try:
            self.hw = RobotHatHardware(self.params, self.get_logger())
        except RuntimeError as err:
            self.get_logger().error(f"Critical error in hardware init, all rhdriver functionality disabled: {err}")
            self.hw = None
            return
        
        # Variables
        self.sensors_active = True

        # Subscribers
        self.led_sub = self.create_subscription(Bool, "robot_hat_led", self.led_callback, 10)
        self.servo_sub = self.create_subscription(ServoCmd, "servo_targets", self.servo_callback, 10)
        self.motor_sub = self.create_subscription(Float64, "motor_speed", self.motor_callback, 10)
        
        # Publishers with timers
        self.battery_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self.battery_timer = self.create_timer(1.0, self.publish_battery)
        self.servo_angle_pub = self.create_publisher(Float64MultiArray, "servo_angles", 10)
        self.servo_angle_timer = self.create_timer(0.01, self.publish_servo_angles)
        self.usr_button_pub = self.create_publisher(Empty, "usr_button", 10)
        self.usr_button_timer = self.create_timer(0.05, self.publish_usr_button)
        self.rst_button_pub = self.create_publisher(Empty, "rst_button", 10)
        self.rst_button_timer = self.create_timer(0.05, self.publish_rst_button)
        if self.params["us_s"]:
            self.us_pub = self.create_publisher(Range, "sonar_range", 10)
            self.us_timer = self.create_timer(0.5, self.publish_sonar)
        if self.params["gs_s"]:
            self.gs_pub = self.create_publisher(Image, "grayscale", 10)
            self.gs_timer = self.create_timer(0.05, self.publish_grayscale)
            self.gs_cal_srv = self.create_service(Trigger, 'calibrate_grayscale', self.cal_gs_callback)

        # Enable/Disable sensors service
        self.sensor_srv = self.create_service(SetBool, 'set_sensors', self.sensor_srv_callback)

        self.get_logger().info('Node ready')


    def led_callback(self, msg: Bool):
        """ Callback handling the /robot_hat_led topic subscriber """
        if msg.data:
            self.get_logger().info("Activating Robot HAT LED")
        else:
            self.get_logger().info("Deactivating Robot HAT LED")
        self.hw.led(msg.data)

    def servo_callback(self, msg: ServoCmd):
        """ Callback handling the /servo_targets topic subscriber """
        self.hw.set_servo(msg.channel, msg.value)

    def motor_callback(self, msg: Float64):
        """ Callback handling the /motor_speed topic subscriber """
        left = msg.data
        right = msg.data
        mp = self.get_parameter("max_motor_percent").get_parameter_value().double_value
        if left < -mp or left > mp or right < -mp or right > mp:
            self.get_logger().warn("Motor speed out of range: l:%f r:%f /%f" % (left, right, mp))
        lefts = max(-mp, min(mp, left))
        rights = max(-mp, min(mp, right))
        self.hw.set_motor_speeds(lefts, rights)

    def publish_battery(self):
        """ Publishing callback for the /battery_state topic """
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
        """ Publishing callback for the /servo_angles topic """
        msg = Float64MultiArray()
        msg.data = [float(s.target_angle) for s in self.hw.servos]
        self.servo_angle_pub.publish(msg)

    def publish_sonar(self):
        """ Publishing callback for the /sonar_range topic """
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
        """ Publishing callback for the /grayscale topic """
        if not self.sensors_active:
            return
        data = self.hw.grayscale.read()
        cal = self.get_parameter('grayscale_calibration').value
        data = [
            max(0, min(65535, round((data[0]-cal[3])/(cal[0]-cal[3])*65535))),
            max(0, min(65535, round((data[1]-cal[4])/(cal[1]-cal[4])*65535))),
            max(0, min(65535, round((data[2]-cal[5])/(cal[2]-cal[5])*65535)))
        ]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'grayscale_sensor'
        msg.height = 1
        msg.width = 3
        msg.encoding = 'mono16'
        msg.is_bigendian = False
        msg.step = 6
        msg.data = struct.pack('<3H', *data)
        
        self.gs_pub.publish(msg)

    def publish_usr_button(self):
        """ Publishing callback for the /usr_button topic """
        if self.hw.usr_button_pressed():
            self.get_logger().info("User button pressed.")
            self.usr_button_pub.publish(Empty())

    def publish_rst_button(self):
        """ Publishing callback for the /rst_button topic """
        if self.hw.rst_button_pressed():
            self.get_logger().info("Reset button pressed.")
            self.rst_button_pub.publish(Empty())

    def cal_gs_callback(self, request, response):
        """ Service callback for the /calibrate_grayscale Trigger service """
        if not self.sensors_active:
            response.success = False
            response.message = "Sensors are not active. Calibration unsuccessful."
        elif not self.params['gs_s']:
            response.success = False
            response.message = "Grayscale sensor not present. Calibration unsuccessful."
        else:
            self.hw.led(True) # flash LED, beginning
            time.sleep(0.1)
            self.hw.led(False)
            # Read high signals for 0.5s
            high = []
            for _ in range(50):
                high.append(self.hw.grayscale.read())
                time.sleep(0.01)
            self.hw.led(True) # Turn on LED for 5 seconds
            time.sleep(5)
            self.hw.led(False)
            # Read low signals for 0.5s
            low = []
            for _ in range(50):
                low.append(self.hw.grayscale.read())
                time.sleep(0.01)
            # combine data sequences into averages and append
            cal = [round(sum(col) / len(col)) for col in zip(*high)] + [round(sum(col) / len(col)) for col in zip(*low)]
            # set ROS parameter
            gs_cal_par = rclpy.parameter.Parameter(
                'grayscale_calibration',
                rclpy.Parameter.Type.INTEGER_ARRAY,
                cal
            )
            self.set_parameters([gs_cal_par])
            self.params['gs_cal'] = cal
            self.hw.led(True) # flash LED, done
            time.sleep(0.1)
            self.hw.led(False)
            self.get_logger().info("Grayscale sensor calibrated with {}.".format(cal))
            response.success = True
            response.message = "Grayscale sensor calibrated with {}.".format(cal)
        
        return response
    
    def sensor_srv_callback(self, req, res):
        if (self.sensors_active != req.data):
            self.sensors_active = req.data
            res.message = f"Sensors {'enabled' if req.data else 'disabled'}."
        else:
            res.message = "Already in requested state."
        res.success = True
        return res

    def destroy_node(self):
        if not self.hw is None:
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