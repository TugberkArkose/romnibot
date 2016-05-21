#!/usr/bin/env python
import rospy
import sys
import ros_numpy
import numpy as np
from scipy import misc
from sensor_msgs.msg import Image

class CalculateDistance:

    def __init__(self):
        rospy.init_node("calculate_distance")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        rospy.Subscriber("/camera/hd/image_depth_rect", Image, self.distanceCallback)

    def distanceCallback(self, msg):
        deserialized = ros_numpy.numpify(msg)
        middleX = len(deserialized) / 2;
        middleY = len(deserialized[0]) / 2;
        distance = deserialized[middleX][middleY]


    def spin(self):
        while not rospy.is_shutdown():
            pass


if __name__ == '__main__':
    """main"""
    CalculateDistance = CalculateDistance()
    CalculateDistance.spin()
