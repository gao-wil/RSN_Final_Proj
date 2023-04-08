#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import serial
import utm
import argparse
import binascii
from imu_driver.msg import Vectornav
from imu_driver.msg import Echolocation
from imu_driver.srv import ConvertToQuaternions
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import *
from geometry_msgs.msg import *
def quaternion_client(yaw, pitch, roll):
	rospy.wait_for_service('convert_to_quaternion')
	convert_to_quaternion = rospy.ServiceProxy('convert_to_quaternion', ConvertToQuaternions)
	response = convert_to_quaternion(yaw, pitch, roll)
	return response.quat
if __name__ == '__main__':
	rospy.init_node('Echolocation Driver')
	port=rospy.get_param('us_port')
	rospy.loginfo('port:' + port)
	pub = rospy.Publisher('echo', Echolocation, queue_size=2)
	serial_port = rospy.get_param('~port', port)
	serial_baud = rospy.get_param('~baudrate', 57600)
	port = serial.Serial(serial_port, serial_baud, timeout=1.)
	rospy.loginfo("using Ultrasonic on port " +serial_port + " at " +str(serial_baud))
	message = Echolocation()
	message_seq_id = int(0) 
	while not rospy.is_shutdown():
		line = port.read_until(b'\r')
		rospy.loginfo(line)
		if str(line).find('R') > 0:
			rospy.loginfo(line)
			message.distance = float(str(line)[3:7])
			message.header.seq = message_seq_id
			message.header.frame_id = "echo1_frame"
			message.header.stamp = rospy.Time.now()
			message_seq_id = message_seq_id + int(1)
			rospy.loginfo(message)			
			pub.publish(message)
						
		else: 
			rospy.loginfo(line)			
					
					
	
