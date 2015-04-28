#!/usr/bin/env python
#coding: utf-8
"""
Created on Tue Mar 10 17:37:34 2015

@author: ferran
"""

import sys
import rospy
import numpy as np
from std_msgs.msg import Int16, Int32, String, Empty, Float32
from geometry_msgs.msg import PointStamped, Point

from naoqi import ALBroker

from IRT import engagement_model

NAO_IP = "192.168.1.12"

class emotion_manager():
    
    def __init__(self):
        
        # Set up the dictionary to be able to calculate the vectors
        self.emotional_dictionary ={'happiness':{'x':0.90,'y':0.75},
                                    'boredom':{'x':-0.75,'y':-0.75},
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
        self.weights = {'lookAt':0.05, 'time_activity':0.01, 'nb_repetitions':0.01, 'smile':0.65, 'movement':0.14, 'sizeHead':0.07, 'novelty':0.07, 'neutral':0.14}
                    
        self.features = {'lookAt':{'x':0.00,'y':0.00},
                         'time_activity':{'x':0.00,'y':0.00}, 
                         'nb_repetitions':{'x':0.00,'y':0.00}, 
                         'smile':{'x':0.00,'y':0.00},                          
                         'movement':{'x':0.00,'y':0.00},
                         'sizeHead':{'x':0.00, 'y':0.00},
                         'novelty':{'x':0.00, 'y':0.00},
                         'neutral':{'x':0.00, 'y':0.00}}
        
        self.prev_time = 1
        self.pub_direction = rospy.Publisher('update_position', PointStamped, queue_size=10)
        self.nb_features = 9
        
        # Cues to evaluate:
        rospy.Subscriber("lookAt", String, self.look_robot_callback)            # Where the child is looking at
        rospy.Subscriber("smile", Empty, self.smile_robot_callback)             # The child is smiling
        rospy.Subscriber("movement", Int16, self.movement_callback)             # The child is moving while sitting
        rospy.Subscriber("sizeHead", Int16, self.proximity_callback)            # The child is getting closer
        rospy.Subscriber("novelty", Float32, self.novelty_callback)             # Something new happen in the scenario
        rospy.Subscriber("activity_time", Int32, self.time_callback)            # For how long the activity was done
        rospy.Subscriber("nb_repetitions", Int16, self.repetitions_callback)    # The number of word repetitions
        rospy.Subscriber("time_response", Float32, self.response_callback)      # Response time till the child writes
        rospy.Subscriber("time_writing", Float32, self.writing_callback)        # Writing time during demostration
                
        rospy.Subscriber('stop_learning', Empty, self.stop_request_callback)    #listen for when to stop
        
        self.pub_engagement = rospy.Publisher('level_engagement', Point, queue_size=10)
        self.pub_activity = rospy.Publisher('activity', String, queue_size=10)  #Publishes the current activity which is performed

        # Boundaries of the map
        self.max_n = 10
        self.min_n = -10
        self.key = "key"
        self.step_size = 1    # Size of the step of the current position towards an emotion
        self.current_time= 0  # Current time in the activity
        
        self.engagement_history = []
        self.activity_turn = True
        
        # Simply keeps python from exiting until this node is stopped
        rospy.spin()
        
    
            
    def buildMessage(self):        
        state = PointStamped()
        state.header.frame_id = self.key
        state.point.x = self.current_position['x']
        state.point.y = self.current_position['y']
        
        return state       
    
#TODO: Less importance to the previous events passing the feature and the weight  
    def weighting(self):        
      
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
        
    """
    If we consider the last 3 probabilities provided by the model
    but also the time spend in the activity we can reach a decision
    """
    def decision_function(self):
        last_three = self.engagement_history[-3:]
        mean = sum(last_three)/3
        
        # Let at least do 3 trials in 5 min with a bad mean
        #if mean < 10 and self.current_time > 300 and np.size(self.engagement_history) > 3:
        if self.current_time > 360:
            self.engagement_history = self.engagement_history[0:-3]
            msg = String()
            if self.activity_turn:
                msg.data = "drawing_nao"
                self.activity_turn = False
            else:
                pass
                #msg.data = "joke_nao"
                #self.activity_turn = True
            self.pub_activity.publish(msg)
            
    
    
    #----------------------------------------------CALLLBACKS----------------------------------------------
    
    def look_robot_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + " The children is looking: %s", data.data)
        #If the child does not give attention to the robot we assume is bored
        looking = data.data
        if looking == "right" or looking == "left" or looking == "up":
            #Calculate the new vector to move towards
            direction_x = self.emotional_dictionary['boredom']['x'] - self.current_position['x']
            direction_y = self.emotional_dictionary['boredom']['y'] - self.current_position['y']                       
            
            sx, sy = self.clampRatio(direction_x, direction_y)
            
            self.features['lookAt']['x'] = self.features['lookAt']['x'] + sx
            self.features['lookAt']['y'] = self.features['lookAt']['y'] + sy

        #If not we assume is happy        
        else:
            #Calculate the new vector to move towards
            direction_x = self.emotional_dictionary['happiness']['x'] - self.current_position['x']
            direction_y = self.emotional_dictionary['happiness']['y'] - self.current_position['y']   
            
            sx, sy = self.clampRatio(direction_x, direction_y)
            
            self.features['lookAt']['x'] = self.features['lookAt']['x'] + sx
            self.features['lookAt']['y'] = self.features['lookAt']['y'] + sy
            
        # Weight the features depending on its importance
        self.weighting()
        msg = self.buildMessage()
        self.pub_direction.publish(msg)
        
    
    def time_callback(self, data):
        self.current_time = data.data
        rospy.loginfo(rospy.get_caller_id() + " Time spend in the same activity: %s", data.data)
        #Calculate the new vector to move towards
        direction_x = self.emotional_dictionary['boredom']['x'] - self.current_position['x']
        direction_y = self.emotional_dictionary['boredom']['y'] - self.current_position['y']
        
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
        

    def repetitions_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Repetitions performed by the children: %s", data.data)
        #Calculate the new vector to move towards
        direction_x = self.emotional_dictionary['boredom']['x'] - self.current_position['x']
        direction_y = self.emotional_dictionary['boredom']['y'] - self.current_position['y']
        
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

        
    def smile_robot_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "The children is smiling at the robot")
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
        
        
    def movement_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "The children is moving: %s", data.data)
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

              
    def proximity_callback(self, data):
        #Calculate the new vector to move towards
        if data.data > 90:
            #rospy.loginfo(rospy.get_caller_id() + "The children is close to the robot" + str(data.data))
            direction_x = self.emotional_dictionary['fear']['x'] - self.current_position['x']
            direction_y = self.emotional_dictionary['fear']['y'] - self.current_position['y']
        
            sx, sy = self.clampRatio(direction_x, direction_y)
        
            self.features['sizeHead']['x'] = self.features['sizeHead']['x'] + sx
            self.features['sizeHead']['y'] = self.features['sizeHead']['y'] + sy
        
            # Weight the features depending on its importance, pack and send
            self.weighting()
            msg = self.buildMessage()
            self.pub_direction.publish(msg)

            
    def novelty_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "The robot saw a novelty!")
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
        
        
    def response_callback(self, data):       
        # Convert to seconds
        rt_sec = data.data / 1000
        #bipolar(P_results, sigma)      
        rospy.loginfo(rt_sec)
        
                          
    # In order to test with a time response of 5 sec: rostopic pub -1 /responseTime std_msgs/Float32 '5'    
    def writing_callback(self, data):
        myModel = engagement_model()
        myModel.train_model()
        # TODO: Check why it does not show the plot
        myModel.plot_data()
        
        # From ms. to s. and get Prob of being engaged  
        wt_sec = data.data / 1000
        P_results = myModel.get_engagement(wt_sec)
        
        # Rescale to have a positive and negative engagement where 0 is 50
        engagement_level = int(P_results[0]*100)
        engagement_level = (engagement_level * 2) - 100
        
        #Store the result in the history
        self.engagement_history.append(engagement_level)
        self.decision_function()              
        
        rospy.loginfo(engagement_level)
        
        # Publish to the map
        point = Point()
        point.x = engagement_level    
        self.pub_engagement.publish(point)
        
    
    def stop_request_callback(self, data):
        rospy.signal_shutdown('Interaction exited')
        
        
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
