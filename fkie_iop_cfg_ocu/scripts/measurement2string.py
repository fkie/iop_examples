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

from fkie_iop_msgs.msg import Measurement
from std_msgs.msg import String
import rospy
from tf.transformations import quaternion_from_euler

__author__ = "Alexander Tiderko (Alexander.Tiderko@fkie.fraunhofer.de)"
__copyright__ = "Copyright (c) 2019 Alexander Tiderko, Fraunhofer FKIE/CMS"
__license__ = "proprietary"
__version__ = "0.1"
__date__ = "2022-05-20"
__doc__ = '''
          Converts first MeasurementValue with single_value to astring. For visualization in Mapviz 
          '''


class Measurement2String():

  def __init__(self):
    self._pub_string = rospy.Publisher("measurement_str", String, queue_size=1)
    rospy.loginfo("Publisher `%s` created", self._pub_string.name)
    self._sub_measurement = rospy.Subscriber("measurement", Measurement, self._on_measurement, queue_size=1)
    rospy.loginfo("Subscriber `%s` created", self._sub_measurement.name)

  def _on_measurement(self, msg):
    strmsg = String()
    strmsg.data = '{}: '.format(msg.device_name)
    count_val = 0
    for val in msg.values:
      if len(val.value):
        if count_val:
          strmsg.data += ', '
        strmsg.data += '%.3f %s' % (val.value[0], val.unit)
        count_val += 1
    self._pub_string.publish(strmsg)

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
  pr = Measurement2String()
  rospy.spin()
