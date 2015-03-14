# -*- coding: utf-8 -*-
"""
Created on Fri Mar 13 11:27:41 2015

@author: ferran
"""

from kivy.app import App
from kivy.uix.widget import Widget
from kivy.graphics import Line
from kivy.uix.button import Button
from kivy.uix.label import Label

import rospy

from geometry_msgs.msg import PoseStamped


class emotionWidget(Widget):

    def __init__(self, **kwargs):
        super(emotionWidget, self).__init__(**kwargs)

        with self.canvas:
            Line(points=[400, 0, 400, 600], width = 1)
            Line(points=[0, 300, 800, 300], width = 1)
            Label(text = 'valence')

           
class emotionMap(App):
        
    def build(self):
        
        self.current_emotion = [0, 0]
        
        # Initialize the node, the topic and name it.        
        topic = 'current_emotion'
        self.pub = rospy.Publisher(topic, PoseStamped, queue_size=10)
        rospy.init_node('valence_arousal_map', anonymous=True)
        rospy.loginfo("I will publish to the topic %s", topic)

        self.emotional_dictionary = {"happiness" : (1.00, 0.75),
                            "sadness" : (-0.75, -0.75),
                            "anger" : (-0.75, 0.75),
                            "fear" : (-1.00, 0.00),
                            "surprise" : (0.00, 0.50),
                            "disgust" : (-0.4, 0.25),
                            "thinking" : (0.25, 0.00),
                            "neutral" : (0.00, 0.00)
                             }

        parent = Widget()
        grid = emotionWidget()
        happy_btn = Button(text = 'happy', pos = (700, 500), size = (50, 50))
        angry_btn = Button(text = 'angry', pos = (100, 500), size = (50, 50))
        surprise_btn = Button(text = 'surprise', pos = (375, 400), size = (50, 50))
        disgust_btn = Button(text = 'disgust', pos = (200, 350), size = (50, 50))
        think_btn = Button(text = 'think', pos = (450, 275), size = (50, 50))
        fear_btn = Button(text = 'fear', pos = (50, 275), size = (50, 50))
        sad_btn = Button(text = 'sad', pos = (100, 100), size = (50, 50))
        label_valence = Label(text = 'valence', pos = (700, 225))
        label_arousal = Label(text = 'arousal', pos = (400, 0))
                
        parent.add_widget(grid)
        parent.add_widget(happy_btn)
        parent.add_widget(angry_btn)
        parent.add_widget(surprise_btn)
        parent.add_widget(disgust_btn)
        parent.add_widget(think_btn)
        parent.add_widget(fear_btn)
        parent.add_widget(sad_btn)
        parent.add_widget(label_valence)
        parent.add_widget(label_arousal)
        
        happy_btn.bind(on_release=self.happy)
        angry_btn.bind(on_release=self.angry)
        surprise_btn.bind(on_release=self.surprise)
        disgust_btn.bind(on_release=self.disgust)
        think_btn.bind(on_release=self.think)
        fear_btn.bind(on_release=self.fear)
        sad_btn.bind(on_release=self.sad)
               
        return parent                 
    
    """
    Creates the message to be published
    """
    def buildMessage(self, dataMsg, key):        
        state = PoseStamped()
        state.header.frame_id = key
        state.pose.position.x = dataMsg[0]
        state.pose.position.y = dataMsg[1]
        
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

    def disgust(self, obj):
        key = "disgust"
        valence = self.emotional_dictionary[key][0]
        arousal = self.emotional_dictionary[key][1]
        self.current_emotion = [valence, arousal]
        self.publishEmotion(key)
    
    def think(self, obj):
        key = "thinking"
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

    def sad(self, obj):
        key = "sadness"
        valence = self.emotional_dictionary[key][0]
        arousal = self.emotional_dictionary[key][1]
        self.current_emotion = [valence, arousal]
        self.publishEmotion(key)
    

def main():   
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        emotionMap().run()
        
    except rospy.ROSInterruptException: pass

if __name__ == "__main__":
    main()
