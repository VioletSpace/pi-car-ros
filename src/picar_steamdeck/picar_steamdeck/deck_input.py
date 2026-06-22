from math import sin, cos, pi
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy

from picar_interfaces.srv import Utility


class DeckInput(Node):

    def __init__(self):
        super().__init__('deck_input_node')
        # Parameters
        self.declare_parameter("timeout_sec", 5.0) # Timeout limit for robot connection until warning
        self.declare_parameter("cmds", ["1:drivemode teleop"])
        self.timeout_sec = self.get_parameter("timeout_sec").value
        cmd_list = self.get_parameter("cmds").value

        # Variables
        self.lbtns = [0 for _ in range(21)]
        self.cmds = {
            0: "drivemode line_follow", # A
            1: "drivemode teleop",      # B
            2: "sensors activate",      # X
            3: "sensors deactivate"     # Y
        }
        if len(cmd_list) > 0:
            for item in cmd_list:
                key, value = item.split(":", 1)
                self.cmds[int(key)] = value

        # Subscribers
        self.joy_sub = self.create_subscription(Joy,        "steamdeck_joy_teleop/joy", self.joy_callback, 10)
        self.con_sub = self.create_subscription(JointState, "joint_states",             self.con_callback, 1)

        # Clients
        self.utility_cli = self.create_client(Utility, 'picar_utility')

        # Connection watchdog
        self.last_con_time = self.get_clock().now()
        self.create_timer(0.05, self.watchdog_cb)
        
        self.get_logger().info("{0} started.".format(self.get_name()))

    
    def call_utility(self, cmd, timeout=0.2):
        if not self.utility_cli.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"Utility unreachable")
            return
        req = Utility.Request()
        req.cmd = cmd

        future = self.utility_cli.call_async(req)
        def done_callback(fut):
            try:
                res = fut.result()
                self.get_logger().info(f"Utility succeded: {res.success}")
            except Exception as e:
                self.get_logger().error(f"Utility failed: {e}")
        future.add_done_callback(done_callback)
    
    def joy_callback(self, msg: Joy):
        for btn in msg.buttons:
            if btn == 1 and btn in self.cmds.keys() and self.lbtns[btn] != 1:
                self.call_utility(self.cmds[btn])
        self.lbtns = msg.buttons

    def con_callback(self, _: JointState):
        self.last_con_time = self.get_clock().now()

    def watchdog_cb(self):
        """
        This is a watchdog timer to detect if the controller is not connected to the robot.
        """
        now = self.get_clock().now()
        dt = (now - self.last_gs_time).nanoseconds * 1e-9
        if dt > self.timeout_sec:
            if not self.connected: self.get_logger().warn("Robot connection timeout!")
            self.connected = True
        else:
            if self.connected: self.get_logger().info("Robot reconnected.")
            self.connected = False

def main():
    try:
        with rclpy.init():
            node = DeckInput()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()