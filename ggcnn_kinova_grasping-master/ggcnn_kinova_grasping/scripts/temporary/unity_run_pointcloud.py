#! /usr/bin/env python
#! get pointcloud from unity image
from __future__ import division
import time

import numpy as np
import torch
import cv2
from helpers.cv_bridge import CvBridge,TimeIt
import scipy.ndimage as ndimage
from skimage.draw import circle
from skimage.feature import peak_local_max
from models.ggcnn import GGCNN
import rospy
from helpers.cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
import sys
bridge = CvBridge()
def depth_callback(depth_message):
    global model
    global graph
    global prev_mp
    global ROBOT_Z
    global fx, cx, fy, cy
    with TimeIt('Crop'):
        depth = bridge.imgmsg_to_cv2(depth_message)
        depth = 0

depth_sub = rospy.Subscriber('/camera/depth/image_meters', Image, depth_callback, queue_size=1)
# robot_pos_sub = rospy.Subscriber('/j2n6s300_driver/out/tool_pose', PoseStamped, robot_pos_callback, queue_size=1)
while not rospy.is_shutdown():
    rospy.spin()