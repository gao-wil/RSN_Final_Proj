#!/usr/bin/env python3
import rospy
from imu_driver.msg import ImuEcho
import tf_conversions
import tf2_ros
import numpy as np
import geometry_msgs.msg
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

msg = PointCloud()
msg.points = []


def callback(data):

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "sensor_frame"
    # print(f"xxxxxxxxx={int(data.sensor_x)}")
    #sensor_x = 1
    #sensor_y = 1

    sensor_x = data.sensor_x
    sensor_y = data.sensor_y 
    range = (data.range)/1000
    t.transform.translation.x = sensor_x
    t.transform.translation.y = sensor_y
    # t.transform.translation.x = 0.5
    # t.transform.translation.y = 0.4
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, data.yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    # print(f"t: ={t}")
    br.sendTransform(t)

    pub = rospy.Publisher('/pointcloud_topic', PointCloud, queue_size=10)
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.points.append(Point32(range*np.cos(data.yaw)+sensor_x, range*np.sin(data.yaw)+sensor_y, 0.0))
    pub.publish(msg)

if __name__ == '__main__':

    i = 0
    rospy.init_node('transform', anonymous=True)
    rospy.Subscriber("/fusion",ImuEcho,callback)
    rospy.spin()
