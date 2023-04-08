#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
import argparse
from gps_driver.msg import gps_msg
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Header


if __name__ == '__main__':
	rospy.init_node('GPGGA_TO_UTM')
	port=rospy.get_param('gps_port')
	rospy.loginfo('port:' + port)
	pub = rospy.Publisher('gps', gps_msg, queue_size=2)
	serial_port = rospy.get_param('~port', port)
	serial_baud = rospy.get_param('~baudrate', 4800)
	sample_rate = rospy.get_param('~sampling_rate', 1.0)
	port = serial.Serial(serial_port, serial_baud, timeout=1.)
	rospy.loginfo("using GPS Puck on port " +serial_port + " at " +str(serial_baud))
	message = gps_msg()
	message_seq_id = int(0) 
	while not rospy.is_shutdown():
		line = port.readline()
		if line == '':
			rospy.loginfo("GPGGA_TO_UTM: no data")
		else: 
			if str(line).find('$GPGGA') > 0:
				rospy.loginfo(line)
				line_parsed = str(line).split(",")
				rospy.loginfo("latitude degree:" + line_parsed[2][0:2] + "latitude minutes" + line_parsed[2][2:4] + "latitude decimal minutes" + line_parsed[2][4:9])
				rospy.loginfo("Parsed Latitude:" + line_parsed[2])
				
				latitude = float(line_parsed[2][0:2])+ (float(line_parsed[2][2:4]) + float(line_parsed[2][4:9]))/60
				if line_parsed[3] == 'S':
					latitude = latitude*(-1)
				rospy.loginfo("Decimal Degree Latitude:" + str(latitude))
				rospy.loginfo("Parsed Longitude:" + line_parsed[4])
				longitude = float(line_parsed[4][0:3])+(float(line_parsed[4][3:5]) + float(line_parsed[4][5:10]))/60
				if line_parsed[5]=='W':
					longitude = longitude*(-1)
				rospy.loginfo("Decimal Degree Longitude:" + str(longitude))
				utm_data = utm.from_latlon(latitude,longitude)
				rospy.loginfo("UTM convert:" + str(utm_data[0]) + ", " + str(utm_data[1]) + ", " + str(utm_data[2]) + ", " + str(utm_data[3]))
				utc = float(line_parsed[1][0:2])*(3600) + float(line_parsed[1][2:4])*60 + float(line_parsed[1][4:6])
				nano_utc = float(line_parsed[1][6:10])*10**9
				rospy.loginfo("nano_sec" + str(nano_utc))
				message.Header.seq = message_seq_id
				message.Header.stamp.secs = int(utc)
				message.Header.stamp.nsecs= int(nano_utc)
				message.Header.frame_id = "GPS1_frame" 
				message.Latitude = latitude
				message.Longitude = longitude
				message.Altitude = float(line_parsed[9])
				message.UTC.secs = int(utc)
				message.UTC.nsecs= int(nano_utc)
				message.UTM_easting = float(utm_data[0])
				message.UTM_northing = float(utm_data[1])
				message.Zone = float(utm_data[2])
				message.Letter = str(utm_data[3])
				message.HDOP = float(line_parsed[8])
				rospy.loginfo(message)
				pub.publish(message)
				message_seq_id = message_seq_id + int(1)
			else: 
				rospy.loginfo("not GPGGA msg")
					
					
	
