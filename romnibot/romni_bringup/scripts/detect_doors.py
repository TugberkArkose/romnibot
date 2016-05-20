#!/usr/bin/env python
import rospy
import sys
import time
import math
import ros_numpy
import numpy as np
from scipy import misc
from sensor_msgs.msg import Image
import pickle

class DetectDoors:

    def __init__(self):

        rospy.init_node("detect_doors")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        classifierPath = '/home/ck/classifier.pkl';
        self.classifier = self.loadClassifier(classifierPath)

        rospy.Subscriber("/camera/hd/image_color", Image, self.imageCallback)
        rospy.Sybscriber("odom", Odometry, self.odometryCallback)


    def imageCallback(self, msg):
        arr = ros_numpy.numpify(msg)
        rospy.logwarn(type(msg))
        rospy.logwarn(type(arr))
        tp = (60,45)
        resizedImage = misc.imresize(arr, tp,interp='bilinear', mode=None)
        grayImage = self.rgb2gray(resizedImage)
        gray2D = grayImage.ravel()

        response = self.classifier.predict(gray2D)
        prob = self. classifier.predict_proba(gray2D)
        print(response)
        print(prob)
        print(len(resizedImage))
        print(len(resizedImage[0]))

    def spin(self):
        while not rospy.is_shutdown():
            pass

    """
        This method loads the trained classifier from given path.
    """
    def loadClassifier(self,classifierPath):
        with open(classifierPath, 'rb') as f:
            clf = pickle.load(f)
            return clf;

    """
        This method converts the input RGB image to Gray scale image
    """
    def rgb2gray(self, rgb):
        return np.dot(rgb[...,:3], [0.299, 0.587, 0.114])

    def odometryCallback(self, odom):
        return null;

if __name__ == '__main__':
    """main"""
    DetectDoors = DetectDoors()
    DetectDoors.spin()
