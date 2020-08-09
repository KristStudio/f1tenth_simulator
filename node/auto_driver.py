#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped


FILLER_VALUE = 100.0
def get_range(data, angle, deg=True):
    """
    this method outputs the distance to a certain angle

    Args:
        data: the LaserScan data outputed by Lidar
        angle: the angle in range (-pi, +pi) in radiens,
            0 is front and positive is to the left.
            For example, 90 will be directly to the left of the Lidar
        deg: (default: True) whether you input degree or radians
    Returns:
        the distance in meter
    """
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILLER_VALUE
    return dis


def callback(data):
    front_dis = get_range(data, 0)
    if front_dis > 1:
        speed = 1 # meter per second
    else:
        speed = 0
    steering_angle = 0 # radian in range [-0.4189, 0.4189]
    drive_msg = AckermannDrive(steering_angle=steering_angle, speed=speed)
    drive_st_msg = AckermannDriveStamped(drive=drive_msg)
    drive_pub.publish(drive_st_msg)


rospy.init_node("auto_driver")

scan_topic = '/scan'
scan_sub = rospy.Subscriber(scan_topic, LaserScan, callback)

drive_topic = 'auto_drive'
drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

rospy.spin()

