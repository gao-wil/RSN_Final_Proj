#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from imu_driver.msg import ImuEcho
import math
import struct

def callback(data):
    

    print(f"data.yaw: {data.yaw}, data.range: {data.range}")
    x = float(data.range * math.cos(data.yaw)/1000)
    y = float(data.range * math.sin(data.yaw)/1000)
    print(f"x: {x}, y: {y}")

    pub = rospy.Publisher('my_pointcloud_topic', PointCloud2, queue_size=10)
    msg = PointCloud2()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "my_frame_id"

    # set the fields of the message
    msg.fields.append(PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1))
    msg.fields.append(PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1))
    msg.fields.append(PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1))

    # create some sample points
    points = [(x, y, 0.0)]

    # set the point cloud data
    msg.data = []
    for p in points:
        msg.data.extend(struct.pack('<fff', *p))

    # set the message metadata
    msg.point_step = 12
    msg.row_step = len(msg.data)
    msg.width = len(points)
    msg.height = 1
    msg.is_dense = True

    # publish the message
    pub.publish(msg)


def listener():

    print("hi")
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/fusion", ImuEcho, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
