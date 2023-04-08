#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np

from imu_driver.srv import ConvertToQuaternions
from geometry_msgs.msg import *
import rospy

def handle_quaternion_conversion(req):
	quat = Quaternion()
	quat.x = np.sin(req.roll/2) * np.cos(req.pitch/2) * np.cos(req.yaw/2) - np.cos(req.roll/2) * np.sin(req.pitch/2) * np.sin(req.yaw/2)
	quat.y = np.cos(req.roll/2) * np.sin(req.pitch/2) * np.cos(req.yaw/2) + np.sin(req.roll/2) * np.cos(req.pitch/2) * np.sin(req.yaw/2)
	quat.z = np.cos(req.roll/2) * np.cos(req.pitch/2) * np.sin(req.yaw/2) - np.sin(req.roll/2) * np.sin(req.pitch/2) * np.cos(req.yaw/2)
	quat.w = np.cos(req.roll/2) * np.cos(req.pitch/2) * np.cos(req.yaw/2) + np.sin(req.roll/2) * np.sin(req.pitch/2) * np.sin(req.yaw/2)
	return quat

def to_quaternions_server():
    rospy.init_node('to_quaternions_server')
    s = rospy.Service('convert_to_quaternion', ConvertToQuaternions, handle_quaternion_conversion)
    print("Ready to convert Euler to Quaternions.")
    rospy.spin()

if __name__ == "__main__":
    to_quaternions_server()
