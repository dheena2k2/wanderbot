#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def scan_callback(msg):
    range_ahead = msg.ranges[90]
    print('range ahead: %0.1f'%range_ahead)

def odo_callback(msg):
    quat = msg.pose.pose.orientation
    orient_list = [quat.x, quat.y, quat.z, quat.w]
    r, p, y = euler_from_quaternion(orient_list)
    print('%f %f %f'%(r, p, y))

rospy.init_node('range_ahead')
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
#odom_sub = rospy.Subscriber('odom', Odometry, odo_callback)
rospy.spin()
