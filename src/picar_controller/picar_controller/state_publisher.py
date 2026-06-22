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

from math import sin, cos, pi
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.timer = self.create_timer(1/30, self.update)

        self.servo_hw_sub = self.create_subscription(Float64MultiArray, 'servo_angles', self.servo_hw_callback, 10)

        self.degree = pi / 180.0

        # robot state
        self.steering_angle = 0.0

        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'axis'
        self.joint_state = JointState()

        self.get_logger().info("{0} started".format(self.get_name()))

    def update(self):
        # update joint_state
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = [
            'left_steering_joint',
            'right_steering_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
            ]
        self.joint_state.position = [self.steering_angle*self.degree, self.steering_angle*self.degree, 0.0, 0.0, 0.0, 0.0]

        # update transform
        self.odom_trans.header.stamp = now.to_msg()

        # send the joint state and transform
        self.joint_pub.publish(self.joint_state)
        self.broadcaster.sendTransform(self.odom_trans)
        

    def servo_hw_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 1:
            self.get_logger().warn("Received empty servo angles. Ignoring.")
            return
        self.steering_angle = msg.data[0] # TODO: parameterise

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    try:
        with rclpy.init():
            node = StatePublisher()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()