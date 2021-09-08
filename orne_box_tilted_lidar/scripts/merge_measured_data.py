#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('orne_box_tilted_lidar')
import rospy
from sensor_msgs.msg import LaserScan
import copy

class merge_measured_data_node:
    def __init__(self):
        rospy.init_node('merge_measured_data_node', anonymous=True)
        self.tilted_lidar_center_sub = rospy.Subscriber("/tilted_scan_center", LaserScan, self.callback_tilted_lidar_center)
        self.tilted_lidar_left_sub = rospy.Subscriber("/tilted_scan_left", LaserScan, self.callback_tilted_lidar_left)
        self.tilted_lidar_right_sub = rospy.Subscriber("/tilted_scan_right", LaserScan, self.callback_tilted_lidar_right)
        self.merged_lidar_pub = rospy.Publisher("/tilted_scan", LaserScan, queue_size=1)
        self.lidar_center = None
        self.lidar_left = None
        self.lidar_right = None

    def callback_tilted_lidar_center(self, data):
        self.lidar_center = copy.deepcopy(data)
        if self.lidar_center != None and self.lidar_left != None and self.lidar_right != None:
            self.merge()

    def callback_tilted_lidar_left(self, data):
        self.lidar_left = copy.deepcopy(data)
        if self.lidar_center != None and self.lidar_left != None and self.lidar_right != None:
            self.merge()

    def callback_tilted_lidar_right(self, data):
        self.lidar_right = copy.deepcopy(data)
        if self.lidar_center != None and self.lidar_left != None and self.lidar_right != None:
            self.merge()

    def merge(self):
        merged_lidar = copy.deepcopy(self.lidar_center)
        merged_lidar.angle_min = self.lidar_right.angle_min
        merged_lidar.angle_max = self.lidar_left.angle_max
        merged_lidar.ranges = self.lidar_right.ranges + self.lidar_center.ranges + self.lidar_left.ranges
        merged_lidar.intensities = self.lidar_right.intensities + self.lidar_center.intensities + self.lidar_left.intensities
        self.merged_lidar_pub.publish(merged_lidar)
        self.lidar_center = None
        self.lidar_left = None
        self.lidar_right = None

if __name__ == '__main__':
    mmd = merge_measured_data_node()
    DURATION = 1
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        r.sleep()
