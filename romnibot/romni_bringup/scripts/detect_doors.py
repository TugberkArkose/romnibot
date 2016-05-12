#!/usr/bin/env python
import rospy
import sys
import time
import math
from sensor_msgs.msg import Image

class DetectDoors:

    def __init__(self):

        rospy.init_node("detect_doors")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)

        rospy.Subscriber("/kinect2/hd/image_color", Image, self.imageCallback)

    def imageCallback(self, msg):
        pass

    def spin(self):
        while not rospy.is_shutdown():
            pass

if __name__ == '__main__':
    """main"""
    DetectDoors = DetectDoors()
    DetectDoors.spin()
