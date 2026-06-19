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
        
        self.get_logger().info("{0} started.".format(self.get_name()))

    def utility_callback(self, request, response):
        cmd = request.cmd.split()
        response.success = False
        if not cmd:
            response.message = "Empty command"
            return response

        match cmd[0]:
            case "drivemode-manual": 
                response.success = self.set_drive_mode(False)
                response.message = (
                    "Drive mode set to manual"
                    if response.success
                    else "Failed to set drive mode"
                )
            case "drivemode-auto": 
                response.success = self.set_drive_mode(True)
                response.message = (
                    "Drive mode set to automatic"
                    if response.success
                    else "Failed to set drive mode"
                )
            case _:
                response.message = "Not yet implemented"
        
        return response

    def set_drive_mode(self, mode: bool):
        req = SetBool.Request()
        req.data = mode
        res = self.line_cli.call(req)
        return res.success

def main():
    try:
        with rclpy.init():
            node = UtilityService()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()