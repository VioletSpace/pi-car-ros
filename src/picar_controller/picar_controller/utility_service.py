from math import sin, cos, pi
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from picar_interfaces.srv import Utility


class UtilityService(Node):

    def __init__(self):
        super().__init__('utility_service_node')

        self.utility_srv = self.create_service(Utility, 'picar_utility', self.utility_callback)

        # Clients
        self.line_cli = self.create_client(SetBool, 'follow_line')
        self.teleop_cli = self.create_client(SetBool, 'teleop_control')
        self.sensors_cli = self.create_client(SetBool, 'set_sensors')
        self.cal_client = self.create_client(Trigger, 'calibrate_grayscale')
        
        self.get_logger().info("{0} started.".format(self.get_name()))

    def utility_callback(self, req, res):
        cmd = req.cmd.split()
        res.success = False
        if not cmd:
            self.get_logger().warn("Utility service received empty command. Ignoring.")
            res.message = "Empty command"
            return res

        match cmd[0]:
            case "drivemode": 
                if len(cmd) < 2:
                    res.message = ("No arg specified")
                    return
                match cmd[1]:
                    case "teleop":
                        res.success = self.set_drive_mode(0)
                    case "line_follow":
                        res.success = self.set_drive_mode(1)
                    case _:
                        res.message = ("Unknown drive mode")
                        return
                res.message = ("Drive mode set" if res.success else "Failed to set drive mode")
            case "sensors":
                if len(cmd) < 2:
                    res.message = ("No arg specified")
                    return
                match cmd[1]:
                    case "activate":
                        res.success = self.call_setbool(self.sensors_cli, True)
                    case "deactivate":
                        res.success = self.call_setbool(self.sensors_cli, False)
                    case _:
                        res.message = ("Unknown sensor state")
                        return
                res.message = ("Sensors set" if res.success else "Failed to set sensors")
            case "calibrate":
                if len(cmd) < 2:
                    res.message = ("No arg specified")
                    return
                match cmd[1]:
                    case "grayscale":
                        res.success = self.call_trigger(self.sensors_cli)
                    case _:
                        res.message = ("Unknown target")
                        return
                res.message = ("Calibration successful" if res.success else "Failed to calibrate")
            case _:
                res.message = "Not yet implemented/Unknown"
        
        self.get_logger().info(f"cmd: {cmd}, res: {res.message}")
        return res

    def set_drive_mode(self, mode: int):
        res_line = self.call_setbool(self.line_cli, mode == 1)
        res_tele = self.call_setbool(self.teleop_cli, mode == 0)
        return res_line and res_tele
    
    def srv_done_callback(self, fut):
        try:
            res = fut.result()
            self.get_logger().info(f"Service succeded: {res.success}")
        except Exception as e:
            self.get_logger().error(f"Service failed: {e}")
    
    def call_setbool(self, client, value, timeout=0.2):
        if not client.wait_for_service(timeout_sec=timeout):
            return False
        req = SetBool.Request()
        req.data = value
        future = client.call_async(req)
        future.add_done_callback(self.srv_done_callback)
        return True
    
    def call_trigger(self, client, timeout=0.2):
        if not client.wait_for_service(timeout_sec=timeout):
            return False
        req = Trigger.Request()
        future = client.call_async(req)
        future.add_done_callback(self.srv_done_callback)
        return True

def main():
    try:
        with rclpy.init():
            node = UtilityService()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()