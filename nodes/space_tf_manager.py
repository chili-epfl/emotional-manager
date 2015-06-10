#!/usr/bin/env python  
import roslib
import rospy
import geometry_msgs.msg

import tf

def handle_pose(msg, head):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.position.x,msg.position.y,msg.position.z),
                     (msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w),
                     rospy.Time.now(),
                     head,
                     "camera")

if __name__ == '__main__':
    rospy.init_node('space_tf_manager')
    #head = 'head_pos'
    rospy.Subscriber('head_pos',
                     geometry_msgs.msg.Pose,
                     handle_pose,
                     'head')
    rospy.spin()
