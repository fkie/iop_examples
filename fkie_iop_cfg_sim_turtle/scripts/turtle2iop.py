#!/usr/bin/env python
###########################################################################fkie
## FRAUNHOFER INTERNAL                                                       ##
###############################################################################
##                                                                           ##
## Copyright 2021 Fraunhofer FKIE                                       ##
## All rights reserved                                                       ##
##                                                                           ##
## RIGHT OF USE. The contents of this file may neither be passed on to third ##
## parties nor utilized or divulged without the expressed prior permission   ##
## of Fraunhofer.                                                            ##
##                                                                           ##
## DEFENCE MATERIAL. This file may contain information which is subject to   ##
## export control.                                                           ##
##                                                                           ##
## FRAUNHOFER INTERNAL. The contents of this file are restricted to the use  ##
## at Fraunhofer, sponsoring German agencies, and other project partners in  ##
## accordance with contract clauses and/or written agreements.               ##
##                                                                           ##
###############################################################################

import math
import sys

from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
import rclpy

__author__ = "Alexander Tiderko (Alexander.Tiderko@fkie.fraunhofer.de)"
__copyright__ = "Copyright (c) 2021 Alexander Tiderko, Fraunhofer FKIE/CMS"
__license__ = "proprietary"
__version__ = "0.1"
__date__ = "2021-05-06"
__doc__ = '''
          Creates a node to convert turtlesim/Pose to nav_msgs/Odometry
          '''

from rclpy.node import Node

# fix as long not available in ROS2
def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

# import numpy as np

# def euler_from_quaternion(quaternion):
#     """
#     Converts quaternion (w in last place) to euler roll, pitch, yaw
#     quaternion = [x, y, z, w]
#     Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
#     """
#     x = quaternion.x
#     y = quaternion.y
#     z = quaternion.z
#     w = quaternion.w

#     sinr_cosp = 2 * (w * x + y * z)
#     cosr_cosp = 1 - 2 * (x * x + y * y)
#     roll = np.arctan2(sinr_cosp, cosr_cosp)

#     sinp = 2 * (w * y - z * x)
#     pitch = np.arcsin(sinp)

#     siny_cosp = 2 * (w * z + x * y)
#     cosy_cosp = 1 - 2 * (y * y + z * z)
#     yaw = np.arctan2(siny_cosp, cosy_cosp)

#     return roll, pitch, yaw

class Converter(Node):

  ORI_OFFSET = 0.
  ORI_INVERT = 1.0

  def __init__(self):
    super().__init__('iop_turtle_pose_converter')
    self.declare_parameter('ori_offset', Converter.ORI_OFFSET)
    self.declare_parameter('ori_invert', Converter.ORI_INVERT)
    self.ORI_OFFSET = self.get_parameter('ori_offset').get_parameter_value().double_value
    self.get_logger().info(f'ori_offset: {self.ORI_OFFSET}')
    ori_invert = self.get_parameter('ori_invert').get_parameter_value().double_value
    if ori_invert not in [-1.0, 1.0]:
      self.get_logger().warn(f'Invalid value {ori_invert} for ori_invert, expected: {1., -1.}; use default: {self.ORI_INVERT}')
      ori_invert = 1.
    self.ORI_INVERT = ori_invert
    self._pub_odom = self.create_publisher(Odometry, 'odom', 1)
    self.get_logger().info(f'Publisher `{self._pub_odom.topic_name}` created')
    self._sub_pose = self.create_subscription(Pose, '/turtle1/pose', self._on_turtle_pose, 1)
    self.get_logger().info(f'Subscriber `{self._sub_pose.topic_name}` created')

  def _on_turtle_pose(self, msg):
    new_msg = Odometry()
    new_msg.header.stamp = self.get_clock().now().to_msg()
    new_msg.header.frame_id = 'odom'
    #new_msg.child_frame_id = 'base_link'
    new_msg.twist.twist.linear.x = msg.linear_velocity
    new_msg.twist.twist.angular.x = msg.angular_velocity
    new_msg.pose.pose.position.x = msg.x
    new_msg.pose.pose.position.y = msg.y
    quat = quaternion_from_euler(0, 0, (msg.theta * self.ORI_INVERT) + self.ORI_OFFSET)
    new_msg.pose.pose.orientation.x = quat[0]
    new_msg.pose.pose.orientation.y = quat[1]
    new_msg.pose.pose.orientation.z = quat[2]
    new_msg.pose.pose.orientation.w = quat[3]
    self._pub_odom.publish(new_msg)

def set_process_name(name):
  # change the process name
  try:
    from ctypes import cdll, byref, create_string_buffer
    libc = cdll.LoadLibrary('libc.so.6')
    buff = create_string_buffer(len(name) + 1)
    buff.value = name
    libc.prctl(15, byref(buff), 0, 0, 0)
  except:
    pass


def setTerminalName(name):
  '''
  Change the terminal name.
  @param name: New name of the terminal
  @type name:  C{str}
  '''
  sys.stdout.write("".join(["\x1b]2;", name, "\x07"]))

if __name__ == '__main__':
  rclpy.init(args=None)
  converter = Converter()
  set_process_name(converter.get_name())
  # set the terminal name
  setTerminalName(converter.get_name())
  try:
    rclpy.spin(converter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    converter.destroy_node()
    rclpy.shutdown()
  except KeyboardInterrupt:
    converter.destroy_node()
    rclpy.shutdown()


