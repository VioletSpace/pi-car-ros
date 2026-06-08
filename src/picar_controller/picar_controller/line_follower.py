from math import sin, cos, pi
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, Float64MultiArray
from std_srvs.srv import SetBool


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')
        # Parameters
        self.declare_parameter("timeout_sec", 0.5)
        self.declare_parameter("max_steer_angle", 15.0)
        self.declare_parameter("line_inverted", False)
        self.declare_parameter("direction_history_length", 5)
        self.timeout_sec = self.get_parameter("timeout_sec").value
        self.k = self.get_parameter("max_steer_angle").value
        self.line_inv = self.get_parameter("line_inverted").value
        self.hist_l = self.get_parameter("direction_history_length").value

        # Variables
        self.enabled = False
        self.estopped = False
        self.dirs = [0.0 for _ in range(0, self.hist_l)]
        self.recovering = False


        # Command publishers
        self.motor_pub = self.create_publisher(Float64, "motor_speed", 10)
        self.servo_pub = self.create_publisher(Float64MultiArray, "servo_target_angles", 10)
        self.led_pub = self.create_publisher(Bool, "robot_hat_led", 10)

        # Enable/Disable service
        self.line_srv = self.create_service(SetBool, 'follow_line', self.enable_callback)

        # Timeout watchdog for Grayscale input
        self.last_gs_time = self.get_clock().now()
        self.create_timer(0.05, self.watchdog_cb)

        # Grayscale subscriber
        self.gs_sub = self.create_subscription(Image, "grayscale", self.follow_callback, 10)
        
        self.get_logger().info("{0} started.".format(self.get_name()))

    def watchdog_cb(self):
        """
        This is a watchdog timer to automatically stop the vehicle if no grayscale
        photodiode input has been received for longer than parameter `timeout_sec` seconds.

        Automatically sends stop command and disables line following until new data received.
        """
        now = self.get_clock().now()
        dt = (now - self.last_gs_time).nanoseconds * 1e-9

        if dt > self.timeout_sec:
            if not self.estopped:
                self.get_logger().error("E-STOP: Grayscale timeout!")
            self.estopped = True
            self.publish_cmd(0.0, 0.0)
        else:
            if self.estopped:
                self.get_logger().info("Grayscale input received, resuming.")
            self.estopped = False
    
    def enable_callback(self, request, response):
        """
        Service to enable/disable the line following. Disabling stops vehicle.
        """
        self.enabled = request.data
        if self.enabled:
            self.get_logger().info("Line following enabled.")
        else:
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info("Line following disabled.")
        response.success = True
        return response
        
    def follow_callback(self, msg: Image):
        """
        On receiving data from the grayscale photodiodes, computes new course for vehicle and
        publishes new `target_servo_angles` and `motor_speed`. Does nothing if e-stop active or
        line following disabled.

        Grayscale data is a mono16 image with 3 pixels from 0-65535. 0 means black, 65535 means
        white. Index 0 left, 1 middle, 2 right.
        """
        self.last_gs_time = self.get_clock().now()
        if self.estopped or not self.enabled:
            return
        data = msg.data
        data = [
            data[0]+256*data[1],
            data[2]+256*data[3],
            data[4]+256*data[5]
        ]
        if self.get_parameter("line_inverted").value:
            data = [65535-x for x in data]

        # Line present? If not, signal via LED and stop
        avgd = sum(data) / 3
        line_present = False
        for d in data:
            if abs(d - avgd) > 8000:
                line_present = True
        if not line_present:
            if not self.recovering:
                self.get_logger().warn("Line lost. Recover.")
                ledmsg = Bool()
                ledmsg.data = True
                self.led_pub().publish(ledmsg)
            self.recovering = True
            self.publish_cmd(0.0, self.hist_dir * -self.k)
            return
        else:
            if self.recovering:
                self.get_logger().info("Recovered.")
                ledmsg = Bool()
                ledmsg.data = False
                self.led_pub().publish(ledmsg)
            self.recovering = False
        
        
        # -1 left - 0 forward - 1 right
        dir = max(-1.0, min(1.0, (data[2] - data[0]) / (data[0] + data[1] + data[2] + 1e-6)))
        # avg of last direction_history_length directions to reduce noise
        self.dirs.pop(0)
        self.dirs.append(dir)
        self.hist_dir = sum(self.dirs) / self.hist_l

        servo_angle = self.hist_dir * -self.k
        self.publish_cmd(80.0, servo_angle)


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
            node = LineFollower()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()