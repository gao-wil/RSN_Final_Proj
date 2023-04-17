
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud
from imu_driver.msg import ImuEcho
import numpy as np
from geometry_msgs.msg import Point32

msg = PointCloud()

msg.points = []

def callback(data):
    

    print(f"data.yaw: {data.yaw}, data.range: {data.range}")
    x = float(data.range * np.cos(data.yaw)/500)
    y = float(data.range * np.sin(data.yaw)/500)
    print(f"x: {x}, y: {y}")

    pub = rospy.Publisher('my_pointcloud_topic', PointCloud, queue_size=10)

    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    msg.points.append(Point32(x, y, 0.0))
    # print(len(msg.points))

    # publish the message
    # if len(msg.points) == 50:
    pub.publish(msg)
    # msg.points = []


def listener():

    print("hi")
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/fusion", ImuEcho, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
