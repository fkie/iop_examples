#!/usr/bin/env python
###########################################################################fkie
## FRAUNHOFER INTERNAL                                                       ##
###############################################################################
##                                                                           ##
## Copyright 2006-2016 Fraunhofer FKIE                                       ##
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
import roslib.names
import rospy
from tf.transformations import quaternion_from_euler

__author__ = "Alexander Tiderko (Alexander.Tiderko@fkie.fraunhofer.de)"
__copyright__ = "Copyright (c) 2019 Alexander Tiderko, Fraunhofer FKIE/CMS"
__license__ = "proprietary"
__version__ = "0.1"
__date__ = "2019-11-21"
__doc__ = '''
          Creates a node to convert turtlesim/Pose to nav_msgs/Odometry
          '''


class Converter():

  ORI_OFFSET = 0.
  ORI_INVERT = 1.0

  def __init__(self):
    self.ORI_OFFSET = rospy.get_param('~ori_offset', Converter.ORI_OFFSET)
    rospy.loginfo("~ori_offset: '%f'", self.ORI_OFFSET)
    ori_invert = rospy.get_param('~ori_invert', Converter.ORI_INVERT)
    if ori_invert not in [-1., 1.]:
      rospy.logwarn("Invalid value for ~ori_invert %s, expected: {1., -1.}; use default: %s" % (ori_invert, self.ORI_INVERT))
      ori_invert = 1.
    self.ORI_INVERT = ori_invert
    self._pub_odom = rospy.Publisher("odom", Odometry, queue_size=1)
    rospy.loginfo("Publisher `%s` created", self._pub_odom.name)
    self._sub_pose = rospy.Subscriber("/turtle1/pose", Pose, self._on_turtle_pose, queue_size=1)
    rospy.loginfo("Subscriber `%s` created", self._sub_pose.name)

  def _on_turtle_pose(self, msg):
    new_msg = Odometry()
    new_msg.header.stamp = rospy.Time.now()
    new_msg.header.frame_id = "base_link"
    new_msg.child_frame_id = "base_link"
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
  rospy.init_node('convert_turtle_pose_to_odom')
  set_process_name(rospy.get_name())
  # set the terminal name
  setTerminalName(rospy.get_name())
  cl = Converter()
  rospy.spin()

