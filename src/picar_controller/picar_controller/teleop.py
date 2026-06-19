from math import sin, cos, pi
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Twist

from picar_interfaces.srv import Utility


class Teleop(Node):

    def __init__(self):
        super().__init__('teleop_node')

        # Parameters
        self.declare_parameter("timeout_sec", 0.5) # Timeout limit for joystick input until E-STOP
        self.timeout_sec = self.get_parameter("timeout_sec").value

        # Variables
        self.enabled = False

        # Subscribers
        self.joy_cmd_sub = self.create_subscription(Twist, "/joy_teleop/cmd_vel", self.joy_cmd_callback, 10)

        # Command publishers
        self.motor_pub = self.create_publisher(Float64, "motor_speed", 10)
        self.servo_pub = self.create_publisher(Float64MultiArray, "servo_target_angles", 10)
        self.led_pub = self.create_publisher(Bool, "robot_hat_led", 10)

        # Enable/Disable service
        self.line_srv = self.create_service(SetBool, 'teleop_control', self.enable_srv_callback)

        # Timeout watchdog for Joystick input
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(0.05, self.watchdog_cb)
        
        self.get_logger().info("{0} started.".format(self.get_name()))

    def joy_cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        if self.estopped or not self.enabled:
            return
        self.publish_cmd(0.0, 0.0)

    def watchdog_cb(self):
        """
        This is a watchdog timer to automatically stop the vehicle if no grayscale
        photodiode input has been received for longer than parameter `timeout_sec` seconds.

        Automatically sends stop command and disables line following until new data received.
        """
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9

        if dt > self.timeout_sec:
            if not self.estopped:
                self.get_logger().error("E-STOP: Joystick timeout!")
            self.estopped = True
            if self.enabled: # Stop on joystick disconnect only if module active
                self.publish_cmd(0.0, 0.0)
        else:
            if self.estopped:
                self.get_logger().info("Joystick input received, resuming.")
            self.estopped = False

    def enable_srv_callback(self, request, response):
        """
        Service to enable/disable teleoperation. Disabling stops vehicle.
        """
        self.enable(request.data)
        response.success = True
        return response
        
    def enable(self, state):
        self.enabled = state
        if self.enabled:
            self.get_logger().info("Teleoperation enabled.")
        else:
            self.publish_cmd(0.0, 0.0)
            ledmsg = Bool()
            ledmsg.data = False
            self.led_pub.publish(ledmsg)
            self.get_logger().info("Teleoperation disabled.")

    def publish_cmd(self, motor_speed, servo_angle):
        """publishing helper"""
        mmsg = Float64()
        mmsg.data = float(motor_speed)
        smsg = Float64MultiArray()
        smsg.data = [float(servo_angle)]
        self.motor_pub.publish(mmsg)
        self.servo_pub.publish(smsg)

def main():
    try:
        with rclpy.init():
            node = Teleop()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()