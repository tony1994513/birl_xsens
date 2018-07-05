#!/usr/bin/env python
import rospy
import tf
from tf.transformations import (
    translation_matrix,
    quaternion_matrix,
    translation_from_matrix,
    quaternion_from_matrix,
)
from numpy.linalg import inv
import ipdb
import copy
import numpy


if __name__ == '__main__':
    rospy.init_node("xens_tf_pub")
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    xsense_tf_names = ["pelvis_xsens", "l5_abdomen_down_xsens", "l3_abdomen_up_xsens", "t12_sternum_down_xsens", "t8_sternum_up_xsens", "neck_xsens", "head_xsens",
    "right_shoulder_xsens", "right_upper_arm_xsens", "right_forearm_xsens", "right_hand_xsens", "left_shoulder_xsens", "left_upper_arm_xsens", "left_forearm_xsens",
    "left_hand_xsens", "right_upper_leg_xsens", "right_lower_leg_xsens", "right_foot_xsens", "right_toe_xsens", "left_upper_leg_xsens", "left_lower_leg_xsens", 
    "left_foot_xsens", "left_toe_xsens"]

    xsense_tf_new_names = ["pelvis_xsens_new", "l5_abdomen_down_xsens_new", "l3_abdomen_up_xsens_new", "t12_sternum_down_xsens_new", "t8_sternum_up_xsens_new", "neck_xsens_new", "head_xsens_new",
    "right_shoulder_xsens_new", "right_upper_arm_xsens_new", "right_forearm_xsens_new", "right_hand_xsens_new", "left_shoulder_xsens_new", "left_upper_arm_xsens_new", "left_forearm_xsens_new",
    "left_hand_xsens_new", "right_upper_leg_xsens_new", "right_lower_leg_xsens_new", "right_foot_xsens_new", "right_toe_xsens_new", "left_upper_leg_xsens_new", "left_lower_leg_xsens_new", 
    "left_foot_xsens_new", "left_toe_xsens_new"]

    xsense_trans = [None] * 23
    xsense_tf_mat = [None] * 23
    xsense_tf_quat = [None] * 23

    calibrated_trans = ( 1.59246897 , 0.0763148 , -0.88838505)  # calibration result
    calibrated_quat = (0, 0, 0, 1)

    marker_pos = (1.66251914, -0.17133719,  0.28578182)

    xsens_base = numpy.dot(translation_matrix(calibrated_trans), quaternion_matrix(calibrated_quat))
    rospy.loginfo("Node is started, shutdown in 3s if no tf coming ")
    publish_rate = 50
    r = rospy.Rate(publish_rate)
    while not rospy.is_shutdown(): 
            for tf_index,tf_name in enumerate(xsense_tf_names):  
                look_up_t = rospy.Time(0)                   
                listener.waitForTransform('body_sensor', tf_name, look_up_t, rospy.Duration(3))
                try:
                    xsense_tf_mat_ = listener.lookupTransform('body_sensor', tf_name, look_up_t)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                   continue
                xsense_tf_quat[tf_index] = xsense_tf_mat_[1]
                xsense_tf_mat[tf_index] = listener.fromTranslationRotation(*xsense_tf_mat_) 
                trans = (xsense_tf_mat_[0][0],xsense_tf_mat_[0][1],xsense_tf_mat_[0][2])
                quat = (0,0,0,1)
                xsense_tf_mat[tf_index] = numpy.dot(translation_matrix(trans), quaternion_matrix(quat)) # xsense tf matrix 


            for tf_mat_index,tf_mat in enumerate(xsense_tf_mat):    
                xsense_each_base = numpy.dot(tf_mat,xsens_base)
                xsense_trans[tf_mat_index] = translation_from_matrix(xsense_each_base)
            
            for trans,quat,tf_name in zip(xsense_trans,xsense_tf_quat,xsense_tf_new_names): 
                broadcaster.sendTransform(
                    trans,
                    quat,
                    rospy.Time.now(),
                    tf_name,
                    'base', 
                )
            r.sleep()
            # dis = numpy.linalg.norm(xsense_trans[4] - numpy.array(marker_pos))
            # rospy.loginfo("error between marker and xsens %s" %dis)
                    
