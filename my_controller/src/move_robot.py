#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from shapely.geometry import Polygon
import tensorflow as tf
from tensorflow.keras import models
from tensorflow.python.keras.backend import set_session
from tensorflow.python.keras.models import load_model

import string
import os
import re
from numpy import asarray
import math

def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

def get_num(x):
	return int(''.join(ele for ele in x if ele.isdigit()))

def callback(data):
	global count
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='bgr8')
	shape = cv_image.shape
	cv_image = cv2.resize(cv_image,(300,300))
	cv2.imwrite("/home/fizzer/ros_ws/Label/"+str(count)+".png",cv_image)
	count = count + 1




count = 0
rospy.init_node('topic_subscriber', anonymous=True)
pub = rospy.Publisher('/R1/cmd_vel',Twist,queue_size=1)
sub = rospy.Subscriber("/R1/pi_camera/image_raw", Image, callback, queue_size=1)
rospy.spin()





