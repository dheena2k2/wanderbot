#! /usr/bin/env python3.9

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import time

class Wander:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.min_dist = 0.8
        self.tollerance = 1
        self.stop_msg = Twist()
        self.move_msg = Twist()
        self.move_msg.linear.x = 0.5
        self.turn_direction = 1
        self.red_light = False
        self.obstracle_scan = None
        self.range_ahead = 0
        self.execute()

    def scan_callback(self,scan):
        range_ahead = scan.ranges[0]
        self.range_ahead = range_ahead
        if not self.red_light:
            print(range_ahead)
            if range_ahead <= self.min_dist:
                #print('within Limit')
                inf = float('inf')
                self.max_length = self.min_dist
                max_index = 0
                for i in range(len(scan.ranges)):
                    x = scan.ranges[i]
                    if x != inf:
                        if x > self.max_length:
                            self.max_length = x
                            max_index = i
                self.turn_direction = 1 if max_index < len(scan.ranges)/2 else -1
                self.red_light = True

    def change_path(self):
        print('Changing path')
        turn = Twist()
        turn.angular.z = 1 * self.turn_direction
        
        tollerance = self.tollerance
        req_range = self.max_length
        while not (req_range - tollerance <= self.range_ahead <= req_range + tollerance):
            self.cmd_vel_pub.publish(turn)
            print(req_range, '===', self.range_ahead)
        
        self.red_light = False

    def execute(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.red_light:
                self.cmd_vel_pub.publish(self.move_msg)
            else:
                self.cmd_vel_pub.publish(self.stop_msg)
                time.sleep(2)
                self.change_path()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('wander')
    wander = Wander()
