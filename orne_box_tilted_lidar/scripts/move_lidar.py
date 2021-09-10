#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('orne_box_tilted_lidar')
import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from std_msgs.msg import Float64
import math
import time

class move_lidar:
    def __init__(self):
        rospy.init_node('mmove_lidar_node', anonymous=True)
        self.tilted_lidar_mirror1_pub   = rospy.Publisher('/tilted_lidar_mirror1_controller/command'  , Float64, queue_size=1)
        self.tilted_lidar_mirror2_pub   = rospy.Publisher('/tilted_lidar_mirror2_controller/command'  , Float64, queue_size=1)
        self.tilted_lidar_prismatic_pub = rospy.Publisher('/tilted_lidar_prismatic_controller/command', Float64, queue_size=1)
        self.tilted_lidar_pitch_pub     = rospy.Publisher('/tilted_lidar_pitch_controller/command'    , Float64, queue_size=1)
        self.tilted_lidar_roll_pub      = rospy.Publisher('/tilted_lidar_roll_controller/command'     , Float64, queue_size=1)
        time.sleep(0.5)
        self.tilted_lidar_mirror1_pub.publish(0.0)
        self.tilted_lidar_mirror2_pub.publish(0.0)
        self.tilted_lidar_prismatic_pub.publish(-0.7)
        self.tilted_lidar_pitch_pub.publish(0.0)
        self.tilted_lidar_roll_pub.publish(0.0)
        self.seq_no = 0
        self.prev_seq_no = -1

    def loop(self):
        if self.seq_no != self.prev_seq_no:
            self.is_first = True
        else:
            self.is_first = False
        self.prev_seq_no = self.seq_no
        if self.seq_no == 0:
            if self.is_first:
                self.angle_deg = 0.0
            self.angle_deg += math.pi*3/8/50
            self.tilted_lidar_mirror1_pub.publish(self.angle_deg)
            self.tilted_lidar_mirror2_pub.publish(self.angle_deg)
            if self.angle_deg >= math.pi*3/8:
                self.seq_no += 1
        elif self.seq_no == 1:
            if self.is_first:
                self.angle_deg = 0.0
            self.angle_deg += math.pi/9/50
            self.tilted_lidar_pitch_pub.publish(self.angle_deg)
            if self.angle_deg >= math.pi/9:
                self.seq_no += 1
        elif self.seq_no == 2:
            if self.is_first:
                self.height = -0.7
            self.height += 0.1/25
            self.tilted_lidar_prismatic_pub.publish(self.height)
            if self.height >= -0.6:
                self.seq_no += 1
        elif self.seq_no == 3:
            self.height -= 0.1/25
            self.tilted_lidar_prismatic_pub.publish(self.height)
            if self.height <= -0.7:
                self.seq_no += 1
        elif self.seq_no == 4:
            if self.is_first:
                self.angle_deg = 0.0
            self.angle_deg += math.pi/10/25
            self.tilted_lidar_roll_pub.publish(self.angle_deg)
            if self.angle_deg >= math.pi/10:
                self.seq_no += 1
        elif self.seq_no == 5:
            self.angle_deg -= math.pi/10/25
            self.tilted_lidar_roll_pub.publish(self.angle_deg)
            if self.angle_deg <= -math.pi/10:
                self.seq_no += 1
        elif self.seq_no == 6:
            self.angle_deg += math.pi/10/25
            self.tilted_lidar_roll_pub.publish(self.angle_deg)
            if self.angle_deg >= 0:
                self.seq_no += 1
        elif self.seq_no == 7:
            return False
        return True

if __name__ == '__main__':
    ml = move_lidar()
    DURATION = 0.05
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        if not ml.loop():
            break
        r.sleep()


