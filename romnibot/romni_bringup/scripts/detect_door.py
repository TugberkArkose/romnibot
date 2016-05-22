#!/usr/bin/env python
import rospy
import sys
import time
import math
import ros_numpy
import numpy as np
from scipy import misc
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import pickle
import warnings
import message_filters

warnings.filterwarnings('ignore')

class DetectDoors:

    def __init__(self):

        rospy.init_node("detect_door")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        classifierPath = '/home/ck/classifier.pkl';
        self.classifier = self.loadClassifier(classifierPath)

        #rospy.Subscriber("/camera/hd/image_color", Image, self.imageCallback)
        #rospy.Subscriber("odom", Odometry, self.odometryCallback)
        #rospy.Subscriber("/camera/hd/image_depth_rect", Image, self.distanceCallback)

        self.colorImage = message_filters.Subscriber('/camera/hd/image_color', Image)
        self.depthImage = message_filters.Subscriber('/camera/hd/image_depth_rect', Image)
        self.odometry = message_filters.Subscriber('odom', Odometry)

        self.previousOdom = None

        ts = message_filters.ApproximateTimeSynchronizer([self.colorImage, self.depthImage, self.odometry], 10, 3)
        ts.registerCallback(self.imageCallback)


    def imageCallback(self, msg, depthImage, odometry):
        warnings.filterwarnings('ignore')
        arr = ros_numpy.numpify(msg)
        tp = (60,45)
        resizedImage = misc.imresize(arr, tp,interp='bilinear', mode=None)
        grayImage = self.rgb2gray(resizedImage)
        gray2D = grayImage.ravel()

        response = self.classifier.predict(gray2D)
        prob = self. classifier.predict_proba(gray2D)
        print(response)
        print(prob)

        if(response == 'door'):
            if(self.previousOdom is not None):
                deserialized = ros_numpy.numpify(depthImage)
                middleX = len(deserialized) / 2;
                middleY = len(deserialized[0]) / 2;
                distance = deserialized[middleX][middleY]
                #print(distance)
                distanceX = abs(self.previousOdom.pose.pose.position.x - odometry.pose.pose.position.x)
                distanceY = abs(self.previousOdom.pose.pose.position.y - odometry.pose.pose.position.y)
                distanceHipotenus = math.sqrt((distanceX * distanceX) + (distanceY * distanceY))
                unitDistance = distance / distanceHipotenus
                if(math.isinf(unitDistance) == False and math.isnan(unitDistance) == False):
                    totalDistanceX = distanceX + (unitDistance * distanceX)
                    totalDistanceY = distanceY + (unitDistance * distanceY)
                    print("X Distance -> {}".format(totalDistanceX))
                    print("Y Distance -> {}".format(totalDistanceY))
                    with open('doorCoordinates.txt','ab+') as f:
                        check = True
                        for line in f:
                            coordinates = line.split(',')
                            coordinateX = int(coordinates[0])
                            coordinateY = int(coordinates[1])
                            if ((coordinateX >= totalDistanceX - 10 and coordinateX <= totalDistanceX + 10)
                                or (coordinateY >= totalDistanceY - 10 and coordinateY <= totalDistanceY + 1)):
                                check = False
                                break
                        if (check):
                            f.write('%d,%d \n' % (int(totalDistanceX),int(totalDistanceY)))
                        #f.write('%d \n' % totalDistanceY)

        self.previousOdom = odometry

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


if __name__ == '__main__':
    """main"""
    DetectDoors = DetectDoors()
    DetectDoors.spin()
