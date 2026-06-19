from math import sin, cos, pi
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray
from std_srvs.srv import SetBool, Trigger


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')
        # Parameters
        self.declare_parameter("timeout_sec", 0.5) # Timeout limit for sensor input until E-STOP
        self.declare_parameter("max_steer_angle", 15.0) # Maximum steering servo angle
        self.declare_parameter("line_inverted", False) # Is the line inverted (white on black)?
        self.declare_parameter("history_length", 5) # Amount of line detection history to keep
        self.declare_parameter("button_toggle", False) # Should this node listen to the USR/RST button topics for control?
        self.timeout_sec = self.get_parameter("timeout_sec").value
        self.k = self.get_parameter("max_steer_angle").value
        self.line_inv = self.get_parameter("line_inverted").value
        self.hist_l = self.get_parameter("history_length").value
        self.btn = self.get_parameter("button_toggle").value

        # Variables
        self.enabled = False
        self.estopped = False
        self.obstructed = False
        self.hist_dir = 0.0
        self.line_hist = [True for _ in range(0, self.hist_l)]
        # PID
        self.pr_err = 0.0
        self.p = self.i = self.d = 0.0
        self.kp, self.ki, self.kd = 1.0, 0.0, 0.7


        # Command publishers
        self.motor_pub = self.create_publisher(Float64, "motor_speed", 10)
        self.servo_pub = self.create_publisher(Float64MultiArray, "servo_target_angles", 10)
        self.led_pub = self.create_publisher(Bool, "robot_hat_led", 10)

        # Enable/Disable service
        self.line_srv = self.create_service(SetBool, 'follow_line', self.enable_srv_callback)

        # Timeout watchdog for Grayscale input
        self.last_gs_time = self.get_clock().now()
        self.create_timer(0.05, self.watchdog_cb)

        # Grayscale subscriber
        self.gs_sub = self.create_subscription(Image, "grayscale", self.follow_callback, 10)
        self.sr_sub = self.create_subscription(Range, "sonar_range", self.sonar_callback, 10)
        if self.btn:
            self.usr_btn_sub = self.create_subscription(Empty, "usr_button", self.enable_trig_callback, 10)
            self.rst_btn_sub = self.create_subscription(Empty, "rst_button", self.cal_callback, 10)

        # Grayscale calibration client
        self.cal_client = self.create_client(Trigger, 'calibrate_grayscale')
        
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
    
    def enable_srv_callback(self, request, response):
        """
        Service to enable/disable the line following. Disabling stops vehicle.
        """
        self.enable(request.data)
        response.success = True
        return response
    
    def enable_trig_callback(self, msg):
        """
        Callback to trigger the line following. Disabling stops vehicle.
        """
        self.enable(not self.enabled)
        
    def enable(self, state):
        self.enabled = state
        if self.enabled:
            self.get_logger().info("Line following enabled.")
        else:
            self.publish_cmd(0.0, 0.0)
            self.recovering = False
            ledmsg = Bool()
            ledmsg.data = False
            self.led_pub.publish(ledmsg)
            self.get_logger().info("Line following disabled.")

    def follow_callback(self, msg: Image):
        """
        On receiving data from the grayscale photodiodes, computes new course for vehicle and
        publishes new `target_servo_angles` and `motor_speed`. Does nothing if e-stop active or
        line following disabled.

        Grayscale data is a mono16 image with 3 pixels from 0-65535. 0 means black, 65535 means
        white. Index 0 left, 1 middle, 2 right.
        """
        self.last_gs_time = self.get_clock().now()
        if self.estopped or not self.enabled or self.obstructed:
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
        line_present = max(data) - min(data) > 8192
        self.line_hist.pop(0)
        self.line_hist.append(line_present)
        if not line_present:
            if not any(self.line_hist):
                self.publish_cmd(0.0, 0.0)
                return
            servo_angle = self.hist_dir * -self.k
            self.publish_cmd(20.0, servo_angle)
            return
        
        # -1 left - 0 forward - 1 right
        err = max(-1.0, min(1.0, (data[2] - data[0]) / (data[0] + data[1] + data[2] + 1e-6)))
        dir = self.pid_controller(err)
        self.hist_dir = dir
        servo_angle = dir * -self.k
        self.publish_cmd(40.0, servo_angle)

    def sonar_callback(self, msg: Range):
        if msg.range > msg.min_range and msg.range < msg.max_range:
            obs = msg.range < 0.15
            if obs and not self.obstructed:
                self.get_logger().info("Robot obstructed: Range %f" % msg.range)
                self.obstructed = True
                self.publish_cmd(0.0, 0.0)
            elif not obs and self.obstructed:
                self.get_logger().info("Robot free: Range %f" % msg.range)
                self.obstructed = False
        else:
            self.get_logger().warn("Received invalid range: %f not in [%f,%f]" % (msg.range, msg.min_range, msg.max_range))

    def cal_callback(self, msg):
        self.get_logger().info("Requesting calibration.")
        req = Trigger.Request()
        future = self.cal_client.call_async(req)
        future.add_done_callback(self.cal_response_callback)

    def cal_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service result: success={response.success}, message={response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

    def publish_cmd(self, motor_speed, servo_angle):
        """publishing helper"""
        mmsg = Float64()
        mmsg.data = float(motor_speed)
        smsg = Float64MultiArray()
        smsg.data = [float(servo_angle)]
        self.motor_pub.publish(mmsg)
        self.servo_pub.publish(smsg)

    def pid_controller(self, err):
        self.p = err
        self.i = self.i + err
        self.d = err - self.pr_err
        pv = self.kp*self.p
        iv = self.ki*self.i
        dv = self.kd*self.d
        self.pr_err = err

        return pv + iv + dv

def main():
    try:
        with rclpy.init():
            node = LineFollower()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()