#!/usr/bin/env python
#coding: utf-8
"""
Created on Tue Mar 10 17:37:34 2015

@author: ferran
"""

import sys
import rospy
from std_msgs.msg import Int16, Int32, String, Empty
from geometry_msgs.msg import Point

from naoqi import ALProxy
from naoqi import ALBroker

NAO_IP = "192.168.1.12"


class emotion_manager():
    
    def __init__(self):
        
        # Set up the dictionary to be able to calculate the vectors
        self.emotional_dictionary ={'happiness':{'x':1.00,'y':0.75},
                                    'bored':{'x':-0.75,'y':-0.75},
                                    'anger':{'x':-0.75,'y':0.75},
                                    'fear':{'x':-1.00,'y':0.00},
                                    'surprise':{'x':0.00,'y':0.50},
                                    'disgust':{'x':-0.4,'y':0.25},
                                    'thinking':{'x':0.25,'y':0.00},
                                    'neutral':{'x':0.00,'y':0.00},
                                    'activation':{'x':0.00,'y':1.00},
                                    'deactivation':{'x':0.00,'y':-1.00},
                                    'pleasant':{'x':1.00,'y':0.00},
                                    'unpleasant':{'x':-1.00,'y':0.00}}
                             
        self.current_position = {'x':0,'y':0}
                    
        self.features = {'lookAt':{'x':0.00,'y':0.00},
                         'time_activity':{'x':0.00,'y':0.00}, 
                         'nb_repetitions':{'x':0.00,'y':0.00}, 
                         'smile':{'x':0.00,'y':0.00},                          
                         'movement':{'x':0.00,'y':0.00},
                         'sizeHead':{'x':0.00, 'y':0.00}}
        
        self.prev_time = 1
        self.pub_direction = rospy.Publisher('update_position', Point, queue_size=10)
        self.nb_features = 4
        
        rospy.Subscriber("lookAt", String, self.look_robot_callback)
        rospy.Subscriber("timeSpendActivity", Int32, self.time_callback)
        rospy.Subscriber("numberRepetitions", Int16, self.repetitions_callback)
        rospy.Subscriber("smile", Empty, self.smile_robot_callback)
        rospy.Subscriber("movement", Int16, self.movement_callback)
        rospy.Subscriber("sizeHead", Int16, self.proximity_callback)
        
        # Boundaries of the map
        self.max_n = 1
        self.min_n = -1
                
        self.step_size = 0.1    # Size of the step of the current position towards an emotion
        
        # Normalizes the values comming from the different feature nodes
        #self.normFeatures = self.normalize(self.features)
        
        # Simply keeps python from exiting until this node is stopped
        rospy.spin()
            
    def buildMessage(self):        
        point = Point()
        point.x = self.current_position['x']
        point.y = self.current_position['y']
        
        return point
           
    def weighting(self, weights = [0.25, 0.25, 0.25, 0.25, 0.25, 0.25]):        
        # Weights order: looking_to_robot|activity_time|repetitions|recognition        
        weightedSumFeatures = {'x':0,'y':0}
        # let us sum all features considering the weights
        i = -1
        for point in self.features.itervalues():
            weightedSumFeatures['x'] = weightedSumFeatures['x'] + (point['x'] * weights[i+1])
            weightedSumFeatures['y'] = weightedSumFeatures['y'] + (point['y'] * weights[i+1])
            i = i+1
        
        self.current_position = weightedSumFeatures
        
    
    def clampRatio(self, cx, cy):
        clamped_number = max(self.min_n,  min(cx, self.max_n));
        ratio_x = 1.0*float(clamped_number)/cx
        clamped_number = max(self.min_n,  min(cy, self.max_n));
        ratio_y = 1.0*float(clamped_number)/cy
        ratio = min (ratio_x, ratio_y);
        
        return (ratio*cx)*self.step_size, (ratio*cy)*self.step_size
    
    
    #----------------------------------------------CALLLBACKS----------------------------------------------

    def look_robot_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " The children is looking: %s", data.data)
        #If the child does not give attention to the robot we assume is bored
        
        looking = data.data
        if looking == "right" or looking == "left" or looking == "up":
            #Calculate the new vector to move towards
            direction_x = self.emotional_dictionary['bored']['x'] - self.current_position['x']
            direction_y = self.emotional_dictionary['bored']['y'] - self.current_position['y']                       
            
            sx, sy = self.clampRatio(direction_x, direction_y)
            
            self.features['lookAt']['x'] = self.features['lookAt']['x'] + sx
            self.features['lookAt']['y'] = self.features['lookAt']['y'] + sy
            rospy.loginfo("X feature value: %s",sx)
            rospy.loginfo("Y feature value: %s", sy)

        #If not we assume is happy        
        else:
            #Calculate the new vector to move towards
            direction_x = self.emotional_dictionary['happiness']['x'] - self.current_position['x']
            direction_y = self.emotional_dictionary['happiness']['y'] - self.current_position['y']   
            
            sx, sy = self.clampRatio(direction_x, direction_y)
            
            self.features['lookAt']['x'] = self.features['lookAt']['x'] + sx
            self.features['lookAt']['y'] = self.features['lookAt']['y'] + sy
            
        # Weight the features depending on its importance
        self.weighting(self)
        msg = self.buildMessage()
        self.pub_direction.publish(msg)

    
    def time_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " Time spend in the same activity: %s", data.data)
        
        #Calculate the new vector to move towards
        direction_x = self.emotional_dictionary['bored']['x'] - self.current_position['x']
        direction_y = self.emotional_dictionary['bored']['y'] - self.current_position['y']
        
        sx, sy = self.clampRatio(direction_x, direction_y)
            
        #When the counter is resetted allows to start again from the beginning
        if data.data > self.prev_time:
            self.features['time_activity']['x'] = self.features['time_activity']['x'] + sx
            self.features['time_activity']['y'] = self.features['time_activity']['y'] + sy
            self.prev_time = data.data
        else:
            #Reset parameter
            self.features['time_activity']['x'] = 0
            self.features['time_activity']['y'] = 0
            self.prev_time = data.data
                        
        # Weight the features depending on its importance, pack and send
        self.weighting(self)
        msg = self.buildMessage()
        self.pub_direction.publish(msg)


    def repetitions_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Repetitions performed by the children: %s", data.data)
        
        #Calculate the new vector to move towards
        direction_x = self.emotional_dictionary['bored']['x'] - self.current_position['x']
        direction_y = self.emotional_dictionary['bored']['y'] - self.current_position['y']
        
        sx, sy = self.clampRatio(direction_x, direction_y)
            
        #When the counter is resetted allows to start again from the beginning
        if data.data > self.prev_time:
            self.features['nb_repetitions']['x'] = self.features['nb_repetitions']['x'] + sx
            self.features['nb_repetitions']['y'] = self.features['nb_repetitions']['y'] + sy
            self.prev_time = data.data
            
        # Weight the features depending on its importance, pack and send
        self.weighting(self)
        msg = self.buildMessage()
        self.pub_direction.publish(msg)

        
    def smile_robot_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "The children is smiling at the robot: %s", data.data)
        
        #Calculate the new vector to move towards
        direction_x = self.emotional_dictionary['happiness']['x'] - self.current_position['x']
        direction_y = self.emotional_dictionary['happiness']['y'] - self.current_position['y']
        
        sx, sy = self.clampRatio(direction_x, direction_y)
        
        self.features['smile']['x'] = self.features['smile']['x'] + sx
        self.features['smile']['y'] = self.features['smile']['y'] + sy
        
        # Weight the features depending on its importance, pack and send
        self.weighting(self)
        msg = self.buildMessage()
        self.pub_direction.publish(msg)
        
        
    def movement_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "The children is moving: %s", data.data)
        
        #Calculate the new vector to move towards
        direction_x = self.emotional_dictionary['activation']['x'] - self.current_position['x']
        direction_y = self.emotional_dictionary['activation']['y'] - self.current_position['y']
        
        sx, sy = self.clampRatio(direction_x, direction_y)
        
        self.features['movement']['x'] = self.features['movement']['x'] + sx
        self.features['movement']['y'] = self.features['movement']['y'] + sy
        
        # Weight the features depending on its importance, pack and send
        self.weighting(self)
        msg = self.buildMessage()
        self.pub_direction.publish(msg)

              
    def proximity_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "The children is close to the robot: %s", data.data)
        
        #Calculate the new vector to move towards
        direction_x = self.emotional_dictionary['fear']['x'] - self.current_position['x']
        direction_y = self.emotional_dictionary['fear']['y'] - self.current_position['y']
        
        sx, sy = self.clampRatio(direction_x, direction_y)
        
        self.features['sizeHead']['x'] = self.features['sizeHead']['x'] + sx
        self.features['sizeHead']['y'] = self.features['sizeHead']['y'] + sy
        
        # Weight the features depending on its importance, pack and send
        self.weighting(self)
        msg = self.buildMessage()
        self.pub_direction.publish(msg)

          
def main():
    
    myBroker = ALBroker("myBroker",
        "0.0.0.0",   
        0,           
        NAO_IP,         
        9559) 
    
    # Initialize the node and name it.       
    rospy.init_node('emotional_manager', anonymous = True)
    # Go to the main loop.
    emotion_manager()

    try:
        pass

    except KeyboardInterrupt:
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)
    
# Main function.   
if __name__ == "__main__":
    main()  
