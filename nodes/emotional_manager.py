#!/usr/bin/env python
#coding: utf-8
"""
Created on Tue Mar 10 17:37:34 2015

@author: ferran
"""

import sys
import rospy
from std_msgs.msg import Int16, Int32, String, Empty, Float32
from geometry_msgs.msg import Point

from naoqi import ALProxy
from naoqi import ALBroker

from IRT import engagement_model

NAO_IP = "192.168.1.12"
#global callback
#callback = True

class emotion_manager():
    
    def __init__(self):
        
        # Set up the dictionary to be able to calculate the vectors
        self.emotional_dictionary ={'happiness':{'x':0.90,'y':0.75},
                                    'bored':{'x':-0.75,'y':-0.75},
                                    'anger':{'x':-0.75,'y':0.75},
                                    'fear':{'x':-0.90,'y':0.00},
                                    'surprise':{'x':0.00,'y':0.50},
                                    'disgust':{'x':-0.4,'y':0.25},
                                    'thinking':{'x':0.25,'y':0.00},
                                    'neutral':{'x':0.00,'y':0.00},
                                    'activation':{'x':0.00,'y':0.90},
                                    'deactivation':{'x':0.00,'y':-0.90},
                                    'pleasant':{'x':0.90,'y':0.00},
                                    'unpleasant':{'x':-0.90,'y':0.00}}
                             
        self.current_position = {'x':0,'y':0}
        self.weights = {'lookAt':0.14, 'time_activity':0.14, 'nb_repetitions':0.14, 'smile':0.14, 'movement':0.14, 'sizeHead':0.14, 'novelty':0.14, 'neutral':0.14}
                    
        self.features = {'lookAt':{'x':0.00,'y':0.00},
                         'time_activity':{'x':0.00,'y':0.00}, 
                         'nb_repetitions':{'x':0.00,'y':0.00}, 
                         'smile':{'x':0.00,'y':0.00},                          
                         'movement':{'x':0.00,'y':0.00},
                         'sizeHead':{'x':0.00, 'y':0.00},
                         'novelty':{'x':0.00, 'y':0.00},
                         'neutral':{'x':0.00, 'y':0.00}}
        
        self.prev_time = 1
        self.pub_direction = rospy.Publisher('update_position', Point, queue_size=10)
        self.nb_features = 7
        
        rospy.Subscriber("lookAt", String, self.look_robot_callback)
        rospy.Subscriber("timeSpendActivity", Int32, self.time_callback)
        rospy.Subscriber("numberRepetitions", Int16, self.repetitions_callback)
        rospy.Subscriber("smile", Empty, self.smile_robot_callback)
        rospy.Subscriber("movement", Int16, self.movement_callback)
        rospy.Subscriber("sizeHead", Int16, self.proximity_callback)
        rospy.Subscriber("novelty", Float32, self.novelty_callback)
        rospy.Subscriber("responseTime", Float32, self.response_callback)
        
        self.pub_engagement = rospy.Publisher('level_engagement', Point, queue_size=10)
        
        
        # Boundaries of the map
        self.max_n = 10
        self.min_n = -10
                
        self.step_size = 1    # Size of the step of the current position towards an emotion
        
#        while callback:
#            self.neutralPos()
#            rospy.sleep(0.3)
        
            
        # Simply keeps python from exiting until this node is stopped
        rospy.spin()
        
        
#    def neutralPos(self):
#        direction_x = self.emotional_dictionary['neutral']['x'] - self.current_position['x']
#        direction_y = self.emotional_dictionary['neutral']['y'] - self.current_position['y']
#        
#        sx, sy = self.clampRatio(direction_x, direction_y)
#            
#        self.features['neutral']['x'] = self.features['neutral']['x'] + sx
#        self.features['neutral']['y'] = self.features['neutral']['y'] + sy
#        
#        self.weighting()
#        msg = self.buildMessage()
#        self.pub_direction.publish(msg)
            
    def buildMessage(self):        
        point = Point()
        point.x = self.current_position['x']
        point.y = self.current_position['y']
        
        return point
    
#TODO: Less importance to the previous events passing the feature and the weight  
    def weighting(self):        
        # Weights order: looking_to_robot|activity_time|repetitions|recognition        
        weightedSumFeatures = {'x':0,'y':0}
       
       # let us sum all features considering the weights
        for point, key in zip(self.features.itervalues(), self.weights.iterkeys()):
            weightedSumFeatures['x'] = weightedSumFeatures['x'] + point['x'] * self.weights[key]
            weightedSumFeatures['y'] = weightedSumFeatures['y'] + point['y'] * self.weights[key]
        
        self.current_position = weightedSumFeatures
        
    
    def clampRatio(self, cx, cy):
        clamped_number = max(self.min_n,  min(cx, self.max_n));
        if cx == 0.0:
            cx = 0.00001
        if cy == 0.0:
            cy = 0.00001
        ratio_x = 1.0*float(clamped_number)/cx
        clamped_number = max(self.min_n,  min(cy, self.max_n));
        ratio_y = 1.0*float(clamped_number)/cy
        ratio = min (ratio_x, ratio_y);
        
        return (ratio*cx)*self.step_size, (ratio*cy)*self.step_size
    
    
    #----------------------------------------------CALLLBACKS----------------------------------------------
    
    def look_robot_callback(self, data):
        global callback
        callback = False
        rospy.loginfo(rospy.get_caller_id() + " The children is looking: %s", data.data)
        #If the child does not give attention to the robot we assume is bored
        featureName = 'lookAt'
        looking = data.data
        if looking == "right" or looking == "left" or looking == "up":
            #Calculate the new vector to move towards
            direction_x = self.emotional_dictionary['bored']['x'] - self.current_position['x']
            direction_y = self.emotional_dictionary['bored']['y'] - self.current_position['y']                       
            
            sx, sy = self.clampRatio(direction_x, direction_y)
            
            self.features['lookAt']['x'] = self.features['lookAt']['x'] + sx
            self.features['lookAt']['y'] = self.features['lookAt']['y'] + sy

        #If not we assume is happy        
        else:
            #Calculate the new vector to move towards
            direction_x = self.emotional_dictionary['neutral']['x'] - self.current_position['x']
            direction_y = self.emotional_dictionary['neutral']['y'] - self.current_position['y']   
            
            sx, sy = self.clampRatio(direction_x, direction_y)
            
            self.features['lookAt']['x'] = self.features['lookAt']['x'] + sx
            self.features['lookAt']['y'] = self.features['lookAt']['y'] + sy
            
        # Weight the features depending on its importance
        self.weighting()
        msg = self.buildMessage()
        self.pub_direction.publish(msg)
        callback = True

    
    def time_callback(self, data):
        global callback
        callback = False
        rospy.loginfo(rospy.get_caller_id() + " Time spend in the same activity: %s", data.data)
        featureName = 'time_activity'
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
        self.weighting()
        msg = self.buildMessage()
        self.pub_direction.publish(msg)
        callback = True


    def repetitions_callback(self, data):
        global callback
        callback = False
        rospy.loginfo(rospy.get_caller_id() + "Repetitions performed by the children: %s", data.data)
        featureName = 'nb_repetitions'
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
        self.weighting()
        msg = self.buildMessage()
        self.pub_direction.publish(msg)
        callback = True

        
    def smile_robot_callback(self, data):
        global callback
        callback = False
        rospy.loginfo(rospy.get_caller_id() + "The children is smiling at the robot")
        featureName = 'smile'
        #Calculate the new vector to move towards
        direction_x = self.emotional_dictionary['happiness']['x'] - self.current_position['x']
        direction_y = self.emotional_dictionary['happiness']['y'] - self.current_position['y']
        
        sx, sy = self.clampRatio(direction_x, direction_y)
        
        self.features['smile']['x'] = self.features['smile']['x'] + sx
        self.features['smile']['y'] = self.features['smile']['y'] + sy
        
        # Weight the features depending on its importance, pack and send
        self.weighting()
        msg = self.buildMessage()
        self.pub_direction.publish(msg)
        callback = True
        
        
    def movement_callback(self, data):
        global callback
        callback = False
        rospy.loginfo(rospy.get_caller_id() + "The children is moving: %s", data.data)
        featureName = 'movement'
        #Calculate the new vector to move towards
        direction_x = self.emotional_dictionary['activation']['x'] - self.current_position['x']
        direction_y = self.emotional_dictionary['activation']['y'] - self.current_position['y']
        
        sx, sy = self.clampRatio(direction_x, direction_y)
        
        self.features['movement']['x'] = self.features['movement']['x'] + sx
        self.features['movement']['y'] = self.features['movement']['y'] + sy
        
        # Weight the features depending on its importance, pack and send
        self.weighting()
        msg = self.buildMessage()
        self.pub_direction.publish(msg)
        callback = True

              
    def proximity_callback(self, data):
        global callback
        callback = False
        featureName = 'sizeHead'
        #Calculate the new vector to move towards
        if data.data > 30:
            rospy.loginfo(rospy.get_caller_id() + "The children is close to the robot")
            direction_x = self.emotional_dictionary['fear']['x'] - self.current_position['x']
            direction_y = self.emotional_dictionary['fear']['y'] - self.current_position['y']
        
            sx, sy = self.clampRatio(direction_x, direction_y)
        
            self.features['sizeHead']['x'] = self.features['sizeHead']['x'] + sx
            self.features['sizeHead']['y'] = self.features['sizeHead']['y'] + sy
        
            # Weight the features depending on its importance, pack and send
            self.weighting()
            msg = self.buildMessage()
            self.pub_direction.publish(msg)
            callback = True

            
    def novelty_callback(self, data):
        global callback
        callback = False
        rospy.loginfo(rospy.get_caller_id() + "The robot saw a novelty!")
        featureName = 'novelty'
        #Calculate the new vector to move towards     
        direction_x = self.emotional_dictionary['surprise']['x'] - self.current_position['x']
        direction_y = self.emotional_dictionary['surprise']['y'] - self.current_position['y']
        
        sx, sy = self.clampRatio(direction_x, direction_y)
        
        self.features['novelty']['x'] = self.features['novelty']['x'] + sx
        self.features['novelty']['y'] = self.features['novelty']['y'] + sy
        
        
        # Weight the features depending on its importance, pack and send
        self.weighting()
        msg = self.buildMessage()
        self.pub_direction.publish(msg)        
        callback = True
    
    # In order to test with a time response of 5 sec: rostopic pub -1 /responseTime std_msgs/Float32 '5'    
    def response_callback(self, data):
        myModel = engagement_model()
        myModel.train_model()
        myModel.plot_data()
        P_results = myModel.get_engagement(data.data)
        
        # Rescale to have a positive and negative engagement where 0 is 50
        engagement_level = int(P_results[0]*100)
        engagement_level = (engagement_level * 2) - 100
        
        rospy.loginfo(engagement_level)
        
        # Publish to the map
        point = Point()
        point.x = engagement_level
        
        self.pub_engagement.publish(point)
        
        
def main():
    
#    myBroker = ALBroker("myBroker",
#        "0.0.0.0",   
#        0,           
#        NAO_IP,         
#        9559) 
    
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
