#!/usr/bin/env python
#coding: utf-8
"""
Created on Fri Mar 13 11:27:41 2015

@author: ferran
original idea: https://github.com/davesnowdon/nao-emotional-framework
"""

import sys
import time

#import numpy as np
import rospy
from geometry_msgs.msg import PointStamped

from naoqi import ALProxy
from naoqi import ALBroker

NAO_IP = "192.168.1.12"

# Global variable to store module instances
emotional_demo = None
memory = None

class action_manager():
    """ A simple module to change the eye LEDs colour to represent emotions.
    """
    def __init__(self):
        
        self.tts = ALProxy("ALTextToSpeech")
        self.leds = ALProxy("ALLeds")
        self.motion = ALProxy("ALMotion")
        
        # Get the ~private namespace parameters from command line or launch file.
        topic = rospy.get_param('~topic', 'current_emotion')
        rospy.loginfo("I will subscribe to the topic %s", topic)
        # Create a subscriber with appropriate topic, custom message and name of callback function.
        rospy.Subscriber(topic, PointStamped, self.express_current_emotion)
        # Wait for messages on topic, go to callback function when new messages arrive.
        rospy.spin()

        # Disable ALAutonomousLife to better demonstrate emotional actions.
        self.autonomous_life = ALProxy("ALAutonomousLife")
        if (self.autonomous_life.getState() != "disabled"):
            self.autonomous_life.setState("disabled")
        time.sleep(1.0)
        self.motion.wakeUp()
        
    #TODO: Run current emotion when a tactile is touch
        
    # Create a callback function for the subscriber.
    def express_current_emotion(self, data, *_args):
        """ 
        Expresses the current emotion from the current valence and arousal values in ALMemory.                        
        """

        # Motion
        motion_names = list()
        motion_times = list()
        motion_keys = list()
        
        # Eyes.
        eye_colour_lookup_table = [[(0xF82C35),(0xF82C35),(0xD55528),(0xD55528),(0xFF622B),(0xFF622B),(0xFFB047),(0xFFB047),(0xFFB047),(0xFFB047),(0xFFB047)],
                                [(0xF82C35),(0xF82C35),(0xD5542A),(0xD5542A),(0xE96A37),(0xFF8232),(0xFF8232),(0xFEB340),(0xFEB340),(0xFEB340),(0xFFFF00)],
                                [(0xF62D35),(0xF62D35),(0xF62D35),(0xE96A37),(0xE96A37),(0xFF984D),(0xFF8232),(0xFDC147),(0xFFB144),(0xFFFF00),(0xFFFF00)],
                                [(0xF72C32),(0xF72C32),(0xFF4048),(0xFE5761),(0xED8659),(0xFEB278),(0xFECE6A),(0xFECE6A),(0xFEE566),(0xFFFF00),(0xFFFF00)],
                                [(0xF6255C),(0xF6255C),(0xF9386F),(0xFD585E),(0xF78C84),(0xFFB379),(0xFEDEA1),(0xFEE67C),(0xFFE564),(0xFFFF00),(0xFFFF00)],
                                [(0xF6255C),(0xF93871),(0xF93871),(0xFE9EB9),(0xFE9EB9),(0xFFFFFF),(0xD0E7B3),(0xA5D277),(0x85B957),(0x6EAB34),(0x6EAB34)],
                                [(0xA82C72),(0xA82C72),(0xC03381),(0xDB5CA1),(0xE8A1C3),(0xD1E5F0),(0xCFDADE),(0x73B8B3),(0x87B958),(0x6EAB34),(0x6EAB34)],
                                [(0xA82C72),(0xA82C72),(0xC03381),(0x9C3F74),(0xB36893),(0xD1E4F2),(0x91C3E6),(0x91C3E6),(0x219A95),(0x00948E),(0x6BAC34)],
                                [(0xA82C72),(0xA82C72),(0x86305D),(0x86305D),(0x94C8D6),(0x93C8D8),(0x92C2E6),(0x3196CE),(0x009591),(0x009591),(0x009591)],
                                [(0xA62D72),(0x692850),(0x692850),(0x692850),(0x2D9DB1),(0x2C9FB2),(0x2F96CE),(0x0085BE),(0x00968D),(0x00968D),(0x00968D)],
                                [(0x692850),(0x692850),(0x692850),(0x692850),(0x037F9B),(0x037F9B),(0x0085BE),(0x0085BE),(0x0085BE),(0x0085BE),(0x0085BE)]
                                ]

        # Speech.
        # Speech parameter lookup table. Format (pitch modifier, volume modifier)
        speech_parameter_lookup_table = [((1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00)),
                                        ((1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00),(1.00,1.00)),
                                        ((1.00,0.75),(0.81,0.75),(0.00,0.00),(0.00,0.00),(-0.25,0.00),(0.50,1.00),(0.62,0.50),(0.75,),(0.75,),(0.75,0.75),(1.00,0.75)),
                                        ((1.00,0.50),(0.63,0.50),(-0.20,-0.50),(-1.00,-1.00),(-0.25,-0.50),(0.25,0.50),(0.25,0.50),(0.50,),(0.50,0.50),(0.50,0.50),(0.00,0.50)),
                                        ((1.00,0.25),(0.44,0.25),(0.40,-0.50),(0.30,-0.50),(0.25,-0.50),(0.25,0.00),(0.25,0.00),(0.25,0.25),(0.25,0.25),(0.25,0.25),(0.00,0.25)),
                                        ((1.00,0.00),(0.25,0.00),(0.10,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.10,0.00),(0.10,0.00),(0.10,0.00),(0.00,0.00)),
                                        ((0.25,-0.25),(0.06,-0.25),(-0.10,-0.25),(-0.20,0.00),(-0.20,0.00),(-0.10,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00)),
                                        ((-0.25,-0.50),(-0.13,-0.50),(-0.35,-0.50),(-0.20,-0.25),(-0.10,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00)),
                                        ((-0.25,-0.75),(-0.31,-0.75),(-0.35,-0.75),(-0.10,-0.50),(-0.10,-0.25),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00)),
                                        ((-0.50,-1.00),(-0.50,-1.00),(-0.40,-1.00),(-0.20,-0.75),(-0.10,-0.50),(0.00,-0.25),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00)),
                                        ((-0.50,-1.00),(-0.50,-1.00),(-0.50,-1.00),(-0.25,-0.75),(0.00,-0.50),(0.00,-0.25),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00),(0.00,0.00))]
        
        
        # CALCULATIONS
        # Get current emotional values and generic calcs.
        current_emotion = data
        valence = current_emotion.point.x
        arousal = current_emotion.point.y
        emotion_name = current_emotion.header.frame_id
        
        # Valence and arousal are normalised between -1 and 1, with an axis intersection at (0, 0)        
        # Convert axis intersection to index.
        valence_index = (int(valence * 5) + 5)
        arousal_index = 10 - (int(arousal * 5) + 5)

        # Speech.
        # The pitch and volume modifier values need scaled, final value to be determined. e.g. a value of 4 will divide the parameter by 4 to give a +/- of 25% of the default value
        speech_parameter_scaling_value = 4
        string_to_say = "I am feeling " + emotion_name
        scaled_pitch_modifier = 1 + (speech_parameter_lookup_table[arousal_index][valence_index][0] / speech_parameter_scaling_value)
        # NAO can only increase pitch! So need to check if a pitch reduction required and negate it. Range 1.0 - 4.0.
        if scaled_pitch_modifier < 1.0:
            scaled_pitch_modifier = 1.0
        # NAO volume (gain) range 0.0 - 1.0.
        scaled_volume_modifier = 0.5 + (speech_parameter_lookup_table[arousal_index][valence_index][1] / speech_parameter_scaling_value)
        self.tts.setParameter("pitchShift", scaled_pitch_modifier)
        self.tts.setVolume(scaled_volume_modifier)
        
        # Eyes.        
        hex_eye_colour = eye_colour_lookup_table[arousal_index][valence_index]
        eye_duration = 2.0

        # Motion.
        # Head pitch - inversely proportional to arousal.
        # Head pitch has a range of approx +0.5 to -0.5 radians so divide normalised arousal value by 2.
        head_pitch = arousal / 2 * -1

        motion_names.append("HeadPitch")
        motion_times.append([0.5, 2, 4])
        motion_keys.append([0.0, head_pitch, 0.0])

        # Stance (torso position + arms) - directly proportional to valence
        # Shoulders have a pitch of +2 to -2 radians.
        # Used in absolute mode, central pitch value is 1.4 radians.
        shoulder_pitch = 1.4 - valence * 0.5

        motion_names.append("LShoulderPitch")
        motion_times.append([0.5, 2, 4])
        motion_keys.append([1.45726, shoulder_pitch, 1.45726])

        motion_names.append("RShoulderPitch")
        motion_times.append([0.5, 2, 4])
        motion_keys.append([1.4, shoulder_pitch, 1.4])

        # Ankles have a pitch of approx +0.9 to -1.1radians.
        # Used in absolute mode, central pitch value is 0.08 radians.
        ankle_pitch = 0.08 - valence * 0.05

        motion_names.append("LAnklePitch")
        motion_times.append([0.5, 2, 4])
        motion_keys.append([0.08, ankle_pitch, 0.08])

        motion_names.append("RAnklePitch")
        motion_times.append([0.5, 2, 4])
        motion_keys.append([0.08, ankle_pitch, 0.08])
        

        # OUTPUTS
        # Speech.
        #self.tts.post.say(string_to_say)
        # Motion.
        self.motion.post.angleInterpolation(motion_names, motion_keys, motion_times, True)
        # Eyes.       
        self.leds.fadeRGB("FaceLeds", hex_eye_colour, eye_duration)
        time.sleep(5.0)
        self.leds.reset("FaceLeds")

        # Reset speech parameters to nominal.
        self.tts.setParameter("pitchShift", 0)
        self.tts.setVolume(0.5)


def main():
    
    myBroker = ALBroker("myBroker",
        "0.0.0.0",   
        0,           
        NAO_IP,         
        9559) 
            
    # Initialize the node and name it.
    rospy.init_node('action_manager', anonymous = True)
    # Go to the main loop.
    action_manager()
    
    try:
        pass

    except KeyboardInterrupt:
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)

# Main function.
if __name__ == '__main__':
    main()
