#!/usr/bin/env python
#coding: utf-8
"""
Created on Tue Mar 10 17:37:34 2015

@author: ferran
"""

import sys
import time
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Time

import numpy as np

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule


NAO_IP = "192.168.1.12"


class emotion_manager():
    
    def __init__(self):
        # Variables to store module instances
        self.nb_features = 4
        self.features = np.zeros(self.nb_features)
        
        rospy.init_node('emotion_manager', anonymous=True)
    	   
        # TODO: figure out the msg types
        #rospy.Subscriber("lookRobot", Int16, look_robot_callback)
        #rospy.Subscriber("timeSpendActivity", Time, time_callback)
        #rospy.Subscriber("numberRepetitions", Int16, repetitions_callback)
        rospy.Subscriber("smileRobot", Int16, self.smile_robot_callback)
    
        # Normalizes the values comming from the different feature nodes
        self.normFeatures = self.normalize(self.features)
    
        # Weight the features depending on its importance
        self.weightedFeatures = self.weighting(self.normFeatures, self.normFeatures)
    
        # Simply keeps python from exiting until this node is stopped
        rospy.spin()
        
        
    def look_robot_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "The children is looking at the robot: %s", data.data)
        self.features[0] = data.data
    
    def time_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Time spend in the same activity: %s", data.data)
        self.features[1] = data.data
        
    def repetitions_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Repetitions performed by the children: %s", data.data)
        self.features[2] = data.data
        
    def smile_robot_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "The children is smiling at the robot: %s", data.data)
        self.features[3] = data.data
        
       
    def normalize(self, features):
        normFeatures = np.zeros(self.nb_features)
        return normFeatures 
                 
    def weighting(self, features = [0, 0, 0, 0], weights = [0.25, 0.25, 0.25, 0.25]):
        
        # Weights order: looking_to_robot|activity_time|repetitions|recognition
        # by default the same 0.25
        
        weightedSumFeatures = 0
        
        # let us sum all features considering the weights
        for i in range(0,len(weights)):
            weightedSumFeatures = weightedSumFeatures + features[i] * weights[i] 
        
        return weightedSumFeatures
        
    
    def map_valece_arousal():    
        pass
    
        
def main(): 
    try:
        emotion_manager().run()

    except rospy.ROSInterruptException:
        print "ROS interruption exception, shutting down"
        sys.exit(0)
    
    
if __name__ == "__main__":
    main()  
