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

from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import roslib.names
import rospy
import tf
from geodesy import utm

__author__ = "Alexander Tiderko (Alexander.Tiderko@fkie.fraunhofer.de)"
__copyright__ = "Copyright (c) 2012 Alexander Tiderko, Fraunhofer FKIE/CMS"
__license__ = "proprietary"
__version__ = "0.1"
__date__ = "2017-03-10"
__doc__ = '''
          The code in this file creates a node to publish a
          sensor_msgs/NavSatFix and sensor_msgs/Imu messages generated from tf.
          Parameters:
            ~world_frame
            ~robot_frame
            ~hz
            ~utm_zone
            ~utm_band
          '''


class TfPose():

  PUB_INTERVAL = 1.0
  ORI_OFFSET = 0.
  ORI_INVERT = 1.0
  UTM_ZONE = 32
  UTM_BAND = 'U'


  def __init__(self):
    self._robot_utm = None  # is needed to translate utm to lat lon position

    self._world_frame = roslib.names.ns_join(rospy.get_param('tf_prefix', '/'), rospy.get_param('~world_frame', '/world'))
    rospy.loginfo("~world_frame: '%s'", self._world_frame)
    self._robot_frame = roslib.names.ns_join(rospy.get_param('tf_prefix', '/'), rospy.get_param('~robot_frame', 'base_link'))
    rospy.loginfo(" ~robot_frame: '%s'", self._robot_frame)
    self.UTM_ZONE = rospy.get_param('utm_zone', self.UTM_ZONE)
    rospy.loginfo("utm_zone: '%d'", self.UTM_ZONE)
    self.UTM_BAND = rospy.get_param('utm_band', self.UTM_BAND)
    rospy.loginfo("utm_band: '%s'", self.UTM_BAND)
    self.PUB_INTERVAL = 1. / rospy.get_param('~hz', 1. / TfPose.PUB_INTERVAL)
    rospy.loginfo("~hz: '%s'", 1. / self.PUB_INTERVAL)
    self.ORI_OFFSET = rospy.get_param('~ori_offset', TfPose.ORI_OFFSET)
    rospy.loginfo("~ori_offset: '%f'", self.ORI_OFFSET)
    ori_invert = rospy.get_param('~ori_invert', TfPose.ORI_INVERT)
    if ori_invert not in [-1., 1.]:
      rospy.logwarn("Invalid value for ~ori_invert %s, expected: {1., -1.}; use default: %s" % (ori_invert, self.ORI_INVERT))
      ori_invert = 1.
    self.ORI_INVERT = ori_invert
    rospy.loginfo("~ori_invert: '%f'", self.ORI_INVERT)
    self._pub_gps = rospy.Publisher("gps_info", NavSatFix, queue_size=1)
    rospy.loginfo("Publisher `%s` created", self._pub_gps.name)
    self._pub_imu = rospy.Publisher("imu", Imu, queue_size=1)
    rospy.loginfo("Publisher `%s` created", self._pub_imu.name)
    self._tfl = tf.TransformListener()
    self._print_transform_error = True
    self._pub_timer = rospy.Timer(rospy.Duration(self.PUB_INTERVAL), self._on_publish_pose, False)

  def _on_publish_pose(self, msg):
    try:
      point, quat = self._tfl.lookupTransform(self._world_frame, self._robot_frame, rospy.Time(0))
      gps_msg = NavSatFix()
#      result.header.frame_id = self._world_frame
      gps_msg.status.status = 0
      gps_msg.status.service = 1
      gps_msg.header.stamp = rospy.Time.now()
      utm_point = utm.UTMPoint(point[0], point[1], point[2], zone=self.UTM_ZONE, band=self.UTM_BAND)
      latlon = utm_point.toMsg()
      gps_msg.latitude = latlon.latitude
      gps_msg.longitude = latlon.longitude
      gps_msg.altitude = latlon.altitude

      imu_msg = Imu()
      imu_msg.header.stamp = rospy.Time.now()
      imu_msg.header.frame_id = self._world_frame
      roll, pitch, yaw = euler_from_quaternion(quat)
      quat = quaternion_from_euler(roll, pitch, (yaw * self.ORI_INVERT) + self.ORI_OFFSET)
      imu_msg.orientation.x = quat[0]
      imu_msg.orientation.y = quat[1]
      imu_msg.orientation.z = quat[2]
      imu_msg.orientation.w = quat[3]
      #print "EULER", math.degrees(roll), math.degrees(pitch), math.degrees(yaw), "mit offset:", math.degrees(yaw + self.ORI_OFFSET)
      self._pub_gps.publish(gps_msg)
      self._pub_imu.publish(imu_msg)
      self._print_transform_error = True
    except tf.LookupException as le:
      if self._print_transform_error:
        self._print_transform_error = False
        rospy.logwarn("Error while looup transform: %s" % le)
    except Exception as ee:
      if self._print_transform_error:
        self._print_transform_error = False
        rospy.logwarn("Error while transform: %s" % ee)


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
  rospy.init_node('bml_tfpose')
  set_process_name(rospy.get_name())
  # set the terminal name
  setTerminalName(rospy.get_name())
  cl = TfPose()
  rospy.spin()

