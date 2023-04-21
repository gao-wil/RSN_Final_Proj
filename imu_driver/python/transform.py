#!/usr/bin/env python3
import rospy
from imu_driver.msg import ImuEcho
import tf_conversions
import tf2_ros
import numpy as np
import geometry_msgs.msg

def callback(data):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()


    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "sensor_frame"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, np.rad2deg(data.yaw))
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)


def transform():
    rospy.init_node('transform', anonymous=True)
    rospy.Subscriber("/fusion",ImuEcho,callback)
    rospy.spin()

if __name__ == '__main__':
    transform()
