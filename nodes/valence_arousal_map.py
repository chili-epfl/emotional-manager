#!/usr/bin/env python
#coding: utf-8
"""
Created on Fri Mar 13 11:27:41 2015

@author: ferran
original idea: https://github.com/davesnowdon/nao-emotional-framework
"""
import numpy as np
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.graphics import Line, Color, Ellipse, Rectangle
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.config import Config

import rospy

from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Empty

# Set up the default size screen
Config.set('graphics', 'width', '1200')
Config.set('graphics', 'height', '600')

class emotionWidget(Widget):

    def __init__(self, **kwargs):
        super(emotionWidget, self).__init__(**kwargs)

        with self.canvas:
            Line(points=[400, 0, 400, 600], width = 1)
            Line(points=[0, 300, 800, 300], width = 1)

"""
Plots the current state in the map using a single red dot
"""
class pointCurrentState(Widget):
    
    def __init__(self, **kwargs):
        super(pointCurrentState, self).__init__(**kwargs)
        
        topic = 'current_emotion'
        self.pub = rospy.Publisher(topic, PointStamped, queue_size=10)

    
        with self.canvas:
            self.color = Color(1, 0, 0)
            self.d = 15.
            self.current_X_pos = 400 - self.d/2
            self.current_Y_pos = 300 - self.d/2
            self.updateX = 0
            self.updateY = 0
            self.prev_X_pos = 0
            self.prev_Y_pos = 0
            
            #Initial state at (0,0)
            self.previous_point = Ellipse(pos=(self.current_X_pos, self.current_Y_pos), size=(self.d, self.d))
                    
        # Initialize the subscriber, the topic and name it.
        topic = 'update_position'
        rospy.Subscriber(topic, PointStamped, self.updateCurrentPosition)

       
    """
    Callback that draws the point according to the new position
    """      
    def plotPoint(self, intp_x, intp_y):

        with self.canvas:
            
            self.color
            for i in range(intp_x.size):   
            
                # Remove the previous point.
                self.canvas.remove(self.previous_point)
                # The current point becomes the previous.
                self.previous_point = Ellipse(pos=(intp_x[i], intp_y[i]), size=(self.d, self.d))
                rospy.sleep(0.001)
                   
    
    """
    Callback that updates the point to the current position
    """   
    def updateCurrentPosition(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard something")
        self.prev_X_pos = self.current_X_pos
        self.prev_Y_pos = self.current_Y_pos
        
        self.updateX = data.point.x
        self.updateY = data.point.y
        self.current_emotion = [self.updateX, self.updateY]
        
        # Transform the points into the correspondent xy-space.
        self.updateX = (((self.updateX / 2) * 800) + 400) - self.d/2
        self.updateY = (((self.updateY / 2) * 600) + 300) - self.d/2
        
        # Does not update if the point is reaching the border of the map.
        if (self.updateX <= 800 and self.updateY <= 600 and self.updateX >= 0 and self.updateY >= 0):
            self.current_X_pos = self.updateX
            self.current_Y_pos = self.updateY
            
            diff_x = pow(self.prev_X_pos - self.updateX, 2)
            diff_y = pow(self.prev_Y_pos - self.updateY, 2)
            dist = np.sqrt(diff_x + diff_y)

            intp_x = np.linspace(self.prev_X_pos, self.updateX, round(dist))
            intp_y = np.linspace(self.prev_Y_pos, self.updateY, round(dist))

            #rospy.loginfo("diff: %s", intp_x)
            #rospy.loginfo("diff: %s", dist)

            # Callback to update the point position
            self.plotPoint(intp_x, intp_y)
            key = "custom"
            self.publishEmotion(key)     
                          
        else:
             rospy.loginfo("The update state reached the border. So, no update is possible")
            
    """
    Creates the message to be published
    """
    def buildMessage(self, dataMsg, key):        
        state = PointStamped()
        state.header.frame_id = key
        state.point.x = dataMsg[0]
        state.point.y = dataMsg[1]
        #state.position.y = dataMsg[1]
        
        return state

    def publishEmotion(self, key):
        strEmotion = "emotion: ", key       
        #rospy.loginfo(strEmotion)
        state_msg = self.buildMessage(self.current_emotion, key)
        self.pub.publish(state_msg)

            
"""
Engagement map class that draws the axis
"""
class engagementMap(Widget):
    def __init__(self, **kwargs):
        super(engagementMap, self).__init__(**kwargs)
        
        with self.canvas:
            self.color = Color(1, 1, 0)
            Line(points=[1050, 50, 1050, 550], width = 1)
            Line(points=[950, 300, 1150, 300], width = 1)


"""
Plots the current state in the engagement map using a rectangle
"""
class engageStatus(Widget):
    
    def __init__(self, **kwargs):
        super(engageStatus, self).__init__(**kwargs)
    
        with self.canvas:
            self.color = Color(1, 0, 0)
            self.pos = (1035,300)
            self.size = (30, 10)
            self.current_level = 10
            self.previous_level = Rectangle(pos=self.pos, size=self.size)
            self.updateHeight = 0
                    
        # Initialize the subscriber, the topic and name it.
        topic = 'level_engagement'
        rospy.Subscriber(topic, Point, self.updateEngLevel)


    """
    Callback that draws the level according to the new position
    """      
    def plotLevel(self, diff):
        #rospy.loginfo("diff: %s", diff)        
        with self.canvas:
            if diff >= 0.0:
                step = -1                
            else:
                step = 1
            while diff != 0.0:
                self.color
                self.current_level = self.current_level - step
                # Remove the previous level.
                self.canvas.remove(self.previous_level)
                # The current level becomes the previous.
                self.previous_level = Rectangle(pos=self.pos, size=(30, self.current_level))
                diff = diff + step
                #rospy.loginfo("diff: %s", diff)
                rospy.sleep(0.01)
            #self.canvas.clear(self.previous_level)
    
    
    """
    Callback that updates the point to the current position
    """   
    def updateEngLevel(self, data):
        update_engagement = data
        self.updateHeight = update_engagement.x
        diff = self.updateHeight - self.current_level
        self.plotLevel(diff)

           
class Map(App, engageStatus):
        
    def build(self):
        
        self.current_emotion = [0, 0]
        
        # Initialize the publisher, the topic and name it.        
        topic = 'current_emotion'
        self.pub = rospy.Publisher(topic, PointStamped, queue_size=10)
        rospy.Subscriber('stop_learning', Empty, self.stop_request_callback)    #listen for when to stop
        
        rospy.init_node('valence_arousal_map', anonymous=True)
        rospy.loginfo("I will publish to the topic %s", topic)

        self.emotional_dictionary = {"happiness" : (1.00, 0.75),
                            "boredom" : (-0.75, -0.75),
                            "anger" : (-0.75, 0.75),
                            "fear" : (-1.00, 0.00),
                            "surprise" : (0.00, 0.50),
                            "neutral" : (0.00, 0.00)}

        
        mainWidget = Widget()   # Main widget contains
        emo_map = Widget()      # the Emotional map
        eng_map = Widget()      # and the Engagement map
        
        # Emotional Map
        grid = emotionWidget()
        happy_btn = Button(text = 'happy', pos = (700, 500), size = (50, 50))
        angry_btn = Button(text = 'angry', pos = (100, 500), size = (50, 50))
        surprise_btn = Button(text = 'surprise', pos = (375, 400), size = (50, 50))
        fear_btn = Button(text = 'fear', pos = (50, 275), size = (50, 50))
        bored_btn = Button(text = 'bored', pos = (100, 100), size = (50, 50))
        label_valence = Label(text = 'valence', pos = (700, 225))
        label_arousal = Label(text = 'arousal', pos = (400, 0))
        label_eng_level = Label(text = 'Engagement level: ' + str(self.updateHeight), pos = (1000, 510))
        point_current_state = pointCurrentState()
                
        emo_map.add_widget(grid)
        emo_map.add_widget(happy_btn)
        emo_map.add_widget(angry_btn)
        emo_map.add_widget(surprise_btn)
        emo_map.add_widget(fear_btn)
        emo_map.add_widget(bored_btn)
        emo_map.add_widget(label_valence)
        emo_map.add_widget(label_arousal)
        emo_map.add_widget(point_current_state) 
        emo_map.add_widget(label_eng_level)
        
        happy_btn.bind(on_release=self.happy)
        angry_btn.bind(on_release=self.angry)
        surprise_btn.bind(on_release=self.surprise)
        fear_btn.bind(on_release=self.fear)
        bored_btn.bind(on_release=self.bored)
        
        # Engagement Map
        grid_eng = engagementMap()
        eng_map.add_widget(grid_eng)
        engage_status = engageStatus()              
        eng_map.add_widget(engage_status)
        
        mainWidget.add_widget(emo_map)
        mainWidget.add_widget(eng_map)

        return mainWidget                 
    
    
    """
    Creates the message to be published
    """
    def buildMessage(self, dataMsg, key):        
        state = PointStamped()
        state.header.frame_id = key
        state.point.x = dataMsg[0]
        state.point.y = dataMsg[1]
        #state.position.y = dataMsg[1]
        
        return state

    def publishEmotion(self, key):
        strEmotion = "emotion: ", key       
        rospy.loginfo(strEmotion)
        state_msg = self.buildMessage(self.current_emotion, key)
        self.pub.publish(state_msg)
        
    """
    Sends the correspondent values based on the button
    """        
    def happy(self, obj):
        key = "happiness"
        valence = self.emotional_dictionary[key][0]
        arousal = self.emotional_dictionary[key][1]
        self.current_emotion = [valence, arousal]
        self.publishEmotion(key)

    def angry(self, obj):
        key = "anger"
        valence = self.emotional_dictionary[key][0]
        arousal = self.emotional_dictionary[key][1]
        self.current_emotion = [valence, arousal]
        self.publishEmotion(key)
        
    def surprise(self, obj):
        key = "surprise"
        valence = self.emotional_dictionary[key][0]
        arousal = self.emotional_dictionary[key][1]
        self.current_emotion = [valence, arousal]
        self.publishEmotion(key)

    def fear(self, obj):
        key = "fear"
        valence = self.emotional_dictionary[key][0]
        arousal = self.emotional_dictionary[key][1]
        self.current_emotion = [valence, arousal]
        self.publishEmotion(key)

    def bored(self, obj):
        key = "boredom"
        valence = self.emotional_dictionary[key][0]
        arousal = self.emotional_dictionary[key][1]
        self.current_emotion = [valence, arousal]
        self.publishEmotion(key)
        
    
    def stop_request_callback(self, data):
        rospy.signal_shutdown('Interaction exited')
        App.get_running_app().stop()
    
    
    
def main():   
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        Map().run()

        #It is actually not necessary
        rospy.spin()
        
    except rospy.ROSInterruptException: pass

if __name__ == "__main__":
    main()
    
