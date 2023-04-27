#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import serial
import utm
import argparse
import binascii
from multiprocessing import Process, Value
from imu_driver.msg import Vectornav
from imu_driver.msg import ImuEcho
from imu_driver.srv import ConvertToQuaternions
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import *
from geometry_msgs.msg import *
range = 0
def echo_loop(shared_variable):
	while not rospy.is_shutdown():
		echo_line = port1.read_until(b'\r')
		rospy.loginfo(echo_line)
		if str(echo_line).find('R') > 0:
			range = int(str(echo_line)[3:7])
			shared_variable.value = range

def quaternion_client(yaw, pitch, roll):
	rospy.wait_for_service('convert_to_quaternion')
	convert_to_quaternion = rospy.ServiceProxy('convert_to_quaternion', ConvertToQuaternions)
	response = convert_to_quaternion(yaw, pitch, roll)
	return response.quat
if __name__ == '__main__':

	#create shared variable
	shared_variable = Value('i', 0)
	
	rospy.init_node('VectorNav and Ultrasonic Driver')
	port=rospy.get_param('imu_port')
	rospy.loginfo('port:' + port)
	pub = rospy.Publisher('fusion', ImuEcho, queue_size=2)
	serial_port = rospy.get_param('~port', port)
	serial_baud = rospy.get_param('~baudrate', 115200)
	port = serial.Serial(serial_port, serial_baud, timeout=1.)
	rospy.loginfo("using VectorNav on port " +serial_port + " at " +str(serial_baud))
	
	port1=rospy.get_param('echo_port')
	rospy.loginfo('port:' + port1)
	serial_port1 = rospy.get_param('~port', port1)
	serial_baud1 = rospy.get_param('~baudrate', 57600)
	port1 = serial.Serial(serial_port1, serial_baud1, timeout=1.)
	rospy.loginfo("using Ultrasonic on port " +serial_port1 + " at " +str(serial_baud1))

	#port.write(binascii.a2b_qp("$VNWRG,06,17*XX"))
	port.write(binascii.a2b_qp("$VNWRG,07,40*XX"))
	message = ImuEcho()
	message.range = 0
	message_seq_id = int(0) 
	p = Process(target=echo_loop, args=(shared_variable,)).start()
				#first time, we don't integrate
	t1= float(0)
	t2= float(0)
	accel_x1 = float(0)
	accel_y1 = float(0)
	Xframe_velocity = float(0)
	Yframe_velocity = float(0)
				
	world_velocity_x_prev = float(0)
	world_velocity_y_prev = float(0)
	world_velocity_x_curr = float(0)
	world_velocity_y_curr = float(0)				

				
	Sensor_position_x = float(0)
	Sensor_position_y = float(0)
	
	while not rospy.is_shutdown():		
		line = port.readline()
		if str(line).find('$VNYIA') > 0:
			rospy.loginfo(line)
			line = str(line)
			line = line.replace("\x00", "")
			line_parsed = str(line).split(",")
			try:
				yaw = np.deg2rad(float(line_parsed[1]))
				pitch = np.deg2rad(float(line_parsed[2]))
				roll = np.deg2rad(float(line_parsed[3]))
				#magx = float(line_parsed[4])/10000
				#magy = float(line_parsed[5])/10000
				#magz = float(line_parsed[6])/10000
				#accelx = float(line_parsed[7])
				accelx = float(line_parsed[4])
				#accely = float(line_parsed[8])
				accely = float(line_parsed[5])
				#accelz = float(line_parsed[9])
				accelz = float(line_parsed[6])
				#gyrox = float(line_parsed[10])
				#gyroy = float(line_parsed[11])
				#line_temp = line_parsed[12].split("*")
				#gyroz = float(line_temp[0])
			except ValueError:
				continue
			q = quaternion_client(yaw, pitch, roll)
			message.yaw=yaw
			message.pitch=pitch
			message.roll=roll

			message.imu.orientation.x= q.x
			message.imu.orientation.y= q.y
			message.imu.orientation.z= q.z
			message.imu.orientation.w= q.w
			message.IMU_string = str(line)
			
			#message.imu.angular_velocity.x=gyrox
			#message.imu.angular_velocity.y=gyroy
			#message.imu.angular_velocity.z=gyroz
			
			message.imu.linear_acceleration.x=accelx
			message.imu.linear_acceleration.y=accely
			message.imu.linear_acceleration.z=accelz
			
			#message.mag_field.magnetic_field.x=magx
			#message.mag_field.magnetic_field.y=magy
			#message.mag_field.magnetic_field.z=magz
			message.header.seq = message_seq_id
			message.header.frame_id = "imu_echo_frame"
			message.header.stamp = rospy.Time.now()
			message.range = int(shared_variable.value)
			if message_seq_id == 0:
				#first time, we don't integrate
				t1= float(message.header.stamp.secs) + float(message.header.stamp.nsecs)*(1e-9)
				t2= 0
				accel_x1 = accelx
				accel_y1 = accely
				Xframe_velocity = 0
				Yframe_velocity = 0
				
				world_velocity_x_prev = 0
				world_velocity_y_prev = 0
				world_velocity_x_curr = 0
				world_velocity_y_curr = 0				

				
				Sensor_position_x = 0
				Sensor_position_y = 0
			else:
				t2 = float(message.header.stamp.secs) + float(message.header.stamp.nsecs)*(1e-9)
				#integration acceleration for velocity
				Xframe_velocity = Xframe_velocity+(t2-t1)*(1/2)*(accel_x1+accelx)
				Yframe_velocity = Yframe_velocity+(t2-t1)*(1/2)*(accel_y1+accely)
				#break velocity into x y world components
				# world_velocity_x_curr = Xframe_velocity*np.sin(yaw)+Yframe_velocity*np.cos(yaw)
				# world_velocity_y_curr = Xframe_velocity*np.cos(yaw)+Yframe_velocity*np.sin(yaw)
				# Sensor_position_x = Sensor_position_x + (t2-t1)*(1/2)*(world_velocity_x_curr + world_velocity_x_prev)
				# Sensor_position_y = Sensor_position_y + (t2-t1)*(1/2)*(world_velocity_y_curr + world_velocity_y_prev)

				#the below lines indicate that Note that the code now simply integrates the acceleration
				#  to get the X and Y frame velocities, and then integrates these velocities to obtain 
				# the position of the sensor without breaking down the velocity into x and y components 
				# in the world frame using the current yaw angle.
				Sensor_position_x = Sensor_position_x + (t2-t1)*(1/2)*(Xframe_velocity + world_velocity_x_prev)
				Sensor_position_y = Sensor_position_y + (t2-t1)*(1/2)*(Yframe_velocity + world_velocity_y_prev)
				t1=t2
				accel_x1 = accelx
				accel_y1 = accely
				# world_velocity_x_prev = world_velocity_x_curr
				world_velocity_x_prev = Xframe_velocity

				# world_velocity_y_prev = world_velocity_y_curr
				world_velocity_y_prev = Yframe_velocity

				message.sensor_x =Sensor_position_x
				message.sensor_y =Sensor_position_y
				rospy.loginfo(message)
				pub.publish(message)
			message_seq_id = message_seq_id + int(1)
				
				
						
		else: 
			rospy.loginfo("not VNYMR msg")
			rospy.loginfo(line)
					
					
	
