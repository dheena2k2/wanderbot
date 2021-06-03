#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class Wanderer:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.min_dist = 0.8  # (Meters) Minimum obstacle distance
        self.field_of_vision = 2  # (Degrees) Multiply this with 2 to get the angle range in front of robot where obstacle is expected
        self.max_length = 2  # (Meters) Length of opening robot looks for
        self.collusion_dist = 0.2  # (Meters) Distance when colided
        self.turn_dir = 1
        self.red_light = False  # An obstacle indicator
        self.nearest_dist = self.min_dist  # Just some value to start with
        self.range_ahead = self.min_dist  # Just some value to start with
        self.execute()

    def scan_callback(self, scan):
        self.range_ahead = scan.ranges[0]
        self.nearest_dist = min(scan.ranges)
        if not self.red_light:  # This code block is executed only when no obstacle is encountered
            ranges_ahead = scan.ranges[len(scan.ranges) - self.field_of_vision: ]
            ranges_ahead += scan.ranges[: self.field_of_vision]
            print 'Range ahead: %f'%self.range_ahead
            if min(ranges_ahead) <= self.min_dist:  # This code block is executed when minimum obstacle distance is reached
                max_index = 0
                max_index_score = len(scan.ranges)
                for i in range(len(scan.ranges)):  # Determinimg the direction of nearest opening
                    if scan.ranges[i] >= self.max_length:
                        if i > len(scan.ranges) / 2 :
                            score = len(scan.ranges) - i
                        else:
                            score = i
                        if score < max_index_score:
                            max_index = i
                            max_index_score = score
                            
                if max_index > len(scan.ranges)//2:  # Setting the direction to turn
                    self.turn_dir = -1
                else:
                    self.turn_dir = 1
                self.red_light = True

    def change_dir(self):
        time.sleep(1)
        turn_msg = Twist()
        turn_msg.angular.z = 0.5 * self.turn_dir
        back_msg = Twist()
        back_msg.linear.x = -0.5
        max_l = self.max_length

        while self.range_ahead <= max_l:  # Turn till the opening is seen
            self.cmd_vel_pub.publish(turn_msg)
            while self.nearest_dist <= self.collusion_dist:  # If collided move back
                self.cmd_vel_pub.publish(back_msg)
            print '%f <= %f'%(max_l, self.range_ahead)

        time.sleep(1)
        self.red_light = False

    def execute(self):
        move_msg = Twist()
        move_msg.linear.x = 0.5
        stop_msg = Twist()
        while not rospy.is_shutdown():
            if not self.red_light:
                self.cmd_vel_pub.publish(move_msg)
            else:
                self.cmd_vel_pub.publish(stop_msg)
                self.change_dir()


if __name__ == '__main__':
    rospy.init_node('wander_v2')
    wanderer = Wanderer()
