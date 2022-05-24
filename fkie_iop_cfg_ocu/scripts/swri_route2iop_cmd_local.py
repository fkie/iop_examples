#!/usr/bin/env python3
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

import sys

from geodesy import utm
from geometry_msgs.msg import PoseStamped
from marti_nav_msgs.msg import Route
from nav_msgs.msg import Path
import rospy
from tf.transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs

__author__ = "Alexander Tiderko (Alexander.Tiderko@fkie.fraunhofer.de)"
__copyright__ = "Copyright (c) 2019 Alexander Tiderko, Fraunhofer FKIE/CMS"
__license__ = "proprietary"
__version__ = "0.1"
__date__ = "2021-10-14"
__doc__ = '''
          Converts swri route to global GeoPath for IOP Bridge
          '''


class RouteToGeoPath():

  def __init__(self):
    self._pub_path = rospy.Publisher("cmd_local_path", Path, queue_size=1)
    rospy.loginfo("Publisher `%s` created", self._pub_path.name)
    self._sub_route = rospy.Subscriber("cmd_local_route", Route, self._on_swri_route, queue_size=1)
    rospy.loginfo("Subscriber `%s` created", self._sub_route.name)
    self.tf_frame_word = rospy.get_param('~frame_world', 'world')
    rospy.loginfo("~frame_word: %s" % self.tf_frame_word)
    self.tf_frame_robot = rospy.get_param('~frame_robot', 'base_link')
    rospy.loginfo("~frame_robot: %s" % self.tf_frame_robot)
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

  def _on_swri_route(self, msg):
    rospy.loginfo('convert path with %d RoutePoints to Path' % len(msg.route_points))
    result = Path()
    result.header = msg.header
    result.header.frame_id = self.tf_frame_word
    for point in msg.route_points:
      utm_point = utm.fromLatLong(point.pose.position.y, point.pose.position.x).toPoint()
      ps = PoseStamped()
      ps.header = result.header
      ps.pose.position.x = utm_point.x
      ps.pose.position.y = utm_point.y
      ps.pose.position.z = point.pose.position.z
      ps.pose.orientation.x = point.pose.orientation.x
      ps.pose.orientation.y = point.pose.orientation.y
      ps.pose.orientation.z = point.pose.orientation.z
      ps.pose.orientation.w = point.pose.orientation.w
      try:
        transform = self.tf_buffer.lookup_transform(self.tf_frame_robot,
                                                    ps.header.frame_id,
                                                    ps.header.stamp,
                                                    rospy.Duration(1.0))
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(e)
        return
      pose_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)
      result.poses.append(pose_transformed)
    self._pub_path.publish(result)
    
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
  rospy.init_node('swri_route2iop_cmd_local')
  set_process_name(rospy.get_name())
  # set the terminal name
  setTerminalName(rospy.get_name())
  pr = RouteToGeoPath()
  rospy.spin()
