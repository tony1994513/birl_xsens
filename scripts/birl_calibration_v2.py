#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)
from numpy.linalg import inv
import ipdb
import threading
import copy
import numpy
from numpy import pi,cos,sin

# xsense_tf_names = ["pelvis_xsens", "l5_abdomen_down_xsens", "l3_abdomen_up_xsens", "t12_sternum_down_xsens", "t8_sternum_up_xsens", "neck_xsens", "head_xsens",
# "right_shoulder_xsens", "right_upper_arm_xsens", "right_forearm_xsens", "right_hand_xsens", "left_shoulder_xsens", "left_upper_arm_xsens", "left_forearm_xsens",
# "left_hand_xsens", "right_upper_leg_xsens", "right_lower_leg_xsens", "right_foot_xsens", "right_toe_xsens", "left_upper_leg_xsens", "left_lower_leg_xsens", 
# "left_foot_xsens", "left_toe_xsens"]

xsense_tf_names = ["right_hand_xsens","left_hand_xsens", ]

# xsense_tf_new_names = ["pelvis_xsens_new", "l5_abdomen_down_xsens_new", "l3_abdomen_up_xsens_new", "t12_sternum_down_xsens_new", "t8_sternum_up_xsens_new", "neck_xsens_new", "head_xsens_new",
# "right_shoulder_xsens_new", "right_upper_arm_xsens_new", "right_forearm_xsens_new", "right_hand_xsens_new", "left_shoulder_xsens_new", "left_upper_arm_xsens_new", "left_forearm_xsens_new",
# "left_hand_xsens_new", "right_upper_leg_xsens_new", "right_lower_leg_xsens_new", "right_foot_xsens_new", "right_toe_xsens_new", "left_upper_leg_xsens_new", "left_lower_leg_xsens_new", 
# "left_foot_xsens_new", "left_toe_xsens_new"]

shared_msg = None
flag_marker = False

def cb(msg):
        global flag_marker,shared_msg
        if not flag_marker:
            flag_marker = True
            print 'marker signals is OK!'
        if msg is not None:
            if msg.markers[0].id == 0:   
                marker =  msg.markers[0]  # get marker info 
                pose = marker.pose.pose
                pos = pose.position
                ori = pose.orientation
                ori = numpy.array([0,0,0,1]) # make marker's orientation be the same with xsense's orientation
                base_to_marker_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori[0], ori[1], ori[2], ori[3]))) # marker matrix
                shared_msg = base_to_marker_mat
                


def main():
    rospy.init_node("xens_calibration")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)
    listener = tf.TransformListener()
    flag_marker = False
    broadcaster = tf.TransformBroadcaster()
    rospy.loginfo("Node started, shutdown in 3s if no tf coming ")
    rospy.loginfo("Calibrating")
    publish_rate = 50
    r = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():

            look_up_t = rospy.Time(0)                   
            listener.waitForTransform('t8_sternum_up_xsens', 'right_hand_xsens', look_up_t, rospy.Duration(3))
            try:
                t8_to_r_hand = listener.lookupTransform('t8_sternum_up_xsens', 'right_hand_xsens', look_up_t)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            t8_to_r_hand_quat = t8_to_r_hand[1]
            t8_to_r_hand_mat = listener.fromTranslationRotation(*t8_to_r_hand) 
            # ipdb.set_trace()
            trans = (t8_to_r_hand_mat[0][0],t8_to_r_hand_mat[0][1],t8_to_r_hand_mat[0][2])
            quat = (0,0,0,1)

            t8_to_r_hand_mat = numpy.dot(translation_matrix(trans), quaternion_matrix(quat)) # xsense tf matrix 

            if shared_msg is not None:
                base_to_marker_mat = shared_msg
                base_to_r_hand = numpy.dot(t8_to_r_hand_mat,base_to_marker_mat) # transform between xsense and base
                base_to_r_hand_trans = translation_from_matrix(base_to_r_hand)
                rospy.loginfo("Calibration result is %s" %base_to_r_hand_trans)
            
                broadcaster.sendTransform(
                    base_to_r_hand_trans,
                    t8_to_r_hand_quat,
                    rospy.Time.now(),
                    "right_hand_xsens_new",
                    'base', 
                )
                r.sleep()
    
if __name__ == '__main__':
    main()




