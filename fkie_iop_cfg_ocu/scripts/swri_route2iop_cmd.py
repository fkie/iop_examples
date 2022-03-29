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

from marti_nav_msgs.msg import Route
from geographic_msgs.msg import GeoPath, GeoPoseStamped, GeoPose, GeoPoint
import rospy
from tf.transformations import quaternion_from_euler

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
    self._pub_geopath = rospy.Publisher("cmd_global_geopath", GeoPath, queue_size=1)
    rospy.loginfo("Publisher `%s` created", self._pub_geopath.name)
    self._sub_route = rospy.Subscriber("cmd_route", Route, self._on_swri_route, queue_size=1)
    rospy.loginfo("Subscriber `%s` created", self._sub_route.name)

  def _on_swri_route(self, msg):
    rospy.loginfo('convert path with %d RoutePoints to GeoPath' % len(msg.route_points))
    result = GeoPath()
    result.header = msg.header
    for point in msg.route_points:
      gps = GeoPoseStamped()
      gps.header = msg.header
      gps.pose.position.latitude = point.pose.position.y
      gps.pose.position.longitude = point.pose.position.x
      gps.pose.position.altitude = point.pose.position.z
      gps.pose.orientation.x = point.pose.orientation.x
      gps.pose.orientation.y = point.pose.orientation.y
      gps.pose.orientation.z = point.pose.orientation.z
      gps.pose.orientation.w = point.pose.orientation.w
      result.poses.append(gps)
    self._pub_geopath.publish(result)

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
  rospy.init_node('swri_route2iop_cmd')
  set_process_name(rospy.get_name())
  # set the terminal name
  setTerminalName(rospy.get_name())
  pr = RouteToGeoPath()
  rospy.spin()
