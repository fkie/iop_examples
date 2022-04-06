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

from marti_nav_msgs.srv import PlanRoute, PlanRouteResponse
from marti_nav_msgs.msg import RoutePoint
import rospy

__author__ = "Alexander Tiderko (Alexander.Tiderko@fkie.fraunhofer.de)"
__copyright__ = "Copyright (c) 2021 Alexander Tiderko, Fraunhofer FKIE/CMS"
__license__ = "proprietary"
__version__ = "0.1"
__date__ = "2021-10-14"
__doc__ = '''
          A very simple node to plan the route for mapviz
          '''


class SwriPlanRoute():

  def __init__(self):
    s = rospy.Service('plan_route', PlanRoute, self._plan_route)
    rospy.loginfo("create service plan_route")

  def _plan_route(self, req):
    rospy.logdebug('received path with %d waypoints' % len(req.waypoints))
    result = PlanRouteResponse()
    idx = 1
    for pose in req.waypoints:
      rp = RoutePoint()
      rp.pose = pose
      rp.pose.orientation.w = 1.0
      rp.pose.orientation.x = 0.0
      rp.pose.orientation.y = 0.0
      rp.pose.orientation.z = 0.0
      rp.id = str(idx)
      idx += 1
      result.route.route_points.append(rp)
    result.success = True
    return result

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
  rospy.init_node('swri_plan_route')
  set_process_name(rospy.get_name())
  # set the terminal name
  setTerminalName(rospy.get_name())
  pr = SwriPlanRoute()
  rospy.spin()
