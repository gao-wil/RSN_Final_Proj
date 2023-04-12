#!/usr/bin/env python3
from __future__ import print_function
from serial import Serial
import rospy
from imu_driver.srv import conversion
from imu_driver.msg import ImuEcho
import sys
import struct

def sensor_fusion():
    argument = rospy.myargv(argv=sys.argv)
    vec_arg = argument[1]
    ultra_arg = argument[2]
    vec_port = Serial(vec_arg, 115200, timeout=10.0)
    ultra_port = Serial(ultra_arg, 57600, timeout=10.0)
    msg = ImuEcho()
    
    rospy.init_node('sensor_fusion', anonymous=True)
    pub = rospy.Publisher('/fusion', ImuEcho, queue_size=100)
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        vec_data = vec_port.readline()
        ultra_data = ultra_port.read_until(b'\r')
        if b'VNYMR' in vec_data:
            vec_data = vec_data.decode('utf-8')
            # print(f"before: {vec_data}")
            vec_data = vec_data.replace("\x00", "")
            # print(f"after: {vec_data}")
            values = vec_data.split(',')
            try: 
                yaw = float(values[1])
                pitch = float(values[2])
                roll = float(values[3])
                mag_x = float(values[4])
                mag_y = float(values[5])
                mag_z = float(values[6])
                accel_x = float(values[7])
                accel_y = float(values[8])
                accel_z = float(values[9])
                # print(len(values[10]))
                gyro_x = float(values[10])
                # print(values[11])
                gyro_y = float(values[11])
                gyro_z = float(values[-1].split('*')[0])

                rospy.wait_for_service('conversion')
                try:
                    convert = rospy.ServiceProxy('conversion', conversion)
                    response = convert(roll,pitch,yaw)
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)

                msg.imu.header.frame_id = 'imu1_frame'
                msg.imu.header.seq +=1
                msg.imu.header.stamp.secs=rospy.get_rostime().secs
                msg.imu.header.stamp.nsecs=rospy.get_rostime().nsecs
                            
                msg.imu.orientation.x= float(response.x)
                msg.imu.orientation.y= float(response.y)
                msg.imu.orientation.z= float(response.z)
                msg.imu.orientation.w= float(response.w)

                msg.imu.angular_velocity.x= gyro_x
                msg.imu.angular_velocity.y= gyro_y
                msg.imu.angular_velocity.z= gyro_z
                
                msg.imu.linear_acceleration.x= accel_x
                msg.imu.linear_acceleration.y= accel_y
                msg.imu.linear_acceleration.z= accel_z
                
                msg.mag_field.header.frame_id = 'imu1_frame'
                msg.mag_field.header.seq +=1
                msg.mag_field.header.stamp.secs=rospy.get_rostime().secs
                msg.mag_field.header.stamp.nsecs=rospy.get_rostime().nsecs
                msg.mag_field.magnetic_field.x= mag_x/10000
                msg.mag_field.magnetic_field.y= mag_y/10000
                msg.mag_field.magnetic_field.z= mag_z/10000

                msg.header.frame_id = 'echo_frame'
                msg.header.seq +=1
                msg.header.stamp.secs=rospy.get_rostime().secs
                msg.header.stamp.nsecs=rospy.get_rostime().nsecs
                msg.IMU_string= str(vec_data)
                msg.yaw=yaw
                msg.pitch=pitch
                msg.roll=roll
                if str(ultra_data).find('R') > 0:
                    msg.range = int(str(ultra_data)[3:7])
                    print(msg)
                    pub.publish(msg)
            except ValueError:
                pass
        rate.sleep()

        

if  __name__ == "__main__":
    try:
        sensor_fusion()
    except rospy.ROSInterruptException:
        pass
        