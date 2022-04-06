#!/usr/bin/env python
# based on https://github.com/afdaniele/rtsp-ros-driver.git
import os
import cv2
import sys
import time
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from fkie_iop_cfg_ocu.camera_info_manager import *
from threading import Thread
from time import sleep


resource = ''
verbose = False
camera_name = ''
camera_frame = ''
camera_info_manager = None
camera_info_publisher = None
is_shutdown = False
image_pub = None


def rtsp_url_callback(msg):
    global resource
    resource = msg.data
    rospy.loginfo("set resource to '%s'" % resource)


def playback_fnc(_):
    global resource, image_pub, camera_info_publisher, is_shutdown, camera_name, camera_frame
    cap = None
    camera_info_manager = None
    while True:
        if is_shutdown or rospy.is_shutdown():
            return
        # open RTSP stream
        try:
            if resource:
                rospy.loginfo("Open resource %s" % resource)
                cap = cv2.VideoCapture(resource)
                if not cap.isOpened():
                    rospy.logerr("Error opening resource `%s`. Please check." % resource)
                    sleep(5.0)
                else:
                    rospy.loginfo("Resource successfully opened")

                    # initialize ROS_CV_Bridge
                    ros_cv_bridge = CvBridge()

                    # initialize Camera Info Manager
                    if camera_info_manager is None:
                        camera_info_manager = CameraInfoManager(cname=camera_name, namespace=camera_name)
                        camera_info_manager.loadCameraInfo()
                        if not camera_info_manager.isCalibrated():
                            rospy.logwarn("No calibration found for the current camera")

                    # initialize variables
                    print('Correctly opened resource, starting to publish feed.')
                    rval, cv_image = cap.read()
                    last_t = time.time()
                    last_print_t = time.time()
                    t_buffer = []

                    # process frames
                    while rval and resource:
                        # get new frame
                        rval, cv_image = cap.read()
                        # handle Ctrl-C
                        key = cv2.waitKey(20)
                        if rospy.is_shutdown() or key == 27 or key == 1048603:
                            break
                        # convert CV image to ROS message
                        image_msg = ros_cv_bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                        image_msg.header.frame_id = camera_frame
                        image_msg.header.stamp = rospy.Time.now()
                        if image_pub is not None:
                            image_pub.publish( image_msg )
                        # publish camera calibration in case of sync publisher
                        if camera_info_manager is not None:
                            camera_info_msg = camera_info_manager.getCameraInfo()
                            camera_info_msg.header.frame_id = camera_frame
                            camera_info_msg.header.stamp = image_msg.header.stamp
                            if camera_info_manager is not None:
                                camera_info_publisher.publish( camera_info_msg )
                        # compute frequency
                        cur_t = time.time()
                        t_buffer.append( cur_t - last_t )
                        # print frequency (if verbose)
                        if cur_t - last_print_t > 1:
                            wait_avg_sec = float(sum(t_buffer))/float(len(t_buffer))
                            hz = 1. / wait_avg_sec
                            if verbose: rospy.loginfo('Streaming @ %.1f Hz' % hz)
                            last_print_t = cur_t
                        last_t = cur_t
            else:
                if cap is not None:
                    rospy.loginfo("release current resource")
                    cap.release()
                    cap = None
                sleep(1.0)
        except Exception:
            pass


if __name__ == '__main__':
    # initialize ROS node
    print('Initializing ROS node... ')
    rospy.init_node('rtsp_camera_driver_node')
    print('Done!')

    # get ROS parameters
    resource = rospy.get_param('~rtsp_resource', '')
    image_raw_topic = rospy.get_param('~image_raw', 'image_raw')
    camera_info_topic = rospy.get_param('~camera_info', 'camera_info')

    # create publishers
    image_pub = rospy.Publisher(image_raw_topic, Image, queue_size=1)
    camera_info_publisher = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=1)
    rtsp_topic = rospy.Subscriber('current_video_url', String, rtsp_url_callback, queue_size=1)

    playback_thread = Thread(target=playback_fnc, args=(camera_info_manager, ))
    playback_thread.start()

    rospy.spin()