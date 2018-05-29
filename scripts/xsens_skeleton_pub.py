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

writable = threading.Event()
writable.clear()
shared_msg = None

def cb(msg):
    global writable, shared_msg
    if writable.is_set():
        shared_msg = msg

def rotate_mat_fnc(mat):
    # rotate pi/2 around x axis
    rotate_mat_x = [[1,0,0,0],
                [0,cos(pi/2),-sin(pi/2),0],
                [0,sin(pi/2),cos(pi/2),0],
                [0,0,0,1],]

    rotate_mat_z = [
                [cos(-pi/2),-sin(-pi/2),0,0],
                [sin(-pi/2),cos(-pi/2),0,0],
                [0,0,1,0],
                [0,0,0,1],]
    tmp = numpy.dot(mat, rotate_mat_x)
    return numpy.dot(rotate_mat_z,tmp)


if __name__ == '__main__':
    rospy.init_node("xens_tf_pub")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)
    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    r = rospy.Rate(10)
    xsense_tf_names = ["pelvis_xsens", "l5_abdomen_down_xsens", "l3_abdomen_up_xsens", "t12_sternum_down_xsens", "t8_sternum_up_xsens", "neck_xsens", "head_xsens",
    "right_shoulder_xsens", "right_upper_arm_xsens", "right_forearm_xsens", "right_hand_xsens", "left_shoulder_xsens", "left_upper_arm_xsens", "left_forearm_xsens",
    "left_hand_xsens", "right_upper_leg_xsens", "right_lower_leg_xsens", "right_foot_xsens", "right_toe_xsens", "left_upper_leg_xsens", "left_lower_leg_xsens", 
    "left_foot_xsens", "left_toe_xsens"]

    xsense_tf_new_names = ["pelvis_xsens_new", "l5_abdomen_down_xsens_new", "l3_abdomen_up_xsens_new", "t12_sternum_down_xsens_new", "t8_sternum_up_xsens_new", "neck_xsens_new", "head_xsens_new",
    "right_shoulder_xsens_new", "right_upper_arm_xsens_new", "right_forearm_xsens_new", "right_hand_xsens_new", "left_shoulder_xsens_new", "left_upper_arm_xsens_new", "left_forearm_xsens_new",
    "left_hand_xsens_new", "right_upper_leg_xsens_new", "right_lower_leg_xsens_new", "right_foot_xsens_new", "right_toe_xsens_new", "left_upper_leg_xsens_new", "left_lower_leg_xsens_new", 
    "left_foot_xsens_new", "left_toe_xsens_new"]
    xsense_trans = [None] * 23
    xsense_quat = [None] * 23
    xsense_tf_mat = [None] * 23
    rospy.loginfo("Node is started, shutdown in 3s if no tf coming ")
    while not rospy.is_shutdown():
            writable.clear()
            msg = copy.deepcopy(shared_msg)
            writable.set()
            
            for tf_index,tf_name in enumerate(xsense_tf_names):  
                look_up_t = rospy.Time(0)                   
                listener.waitForTransform('body_sensor', tf_name, look_up_t, rospy.Duration(3))
                try:
                    xsense_tf_mat_ = listener.lookupTransform('body_sensor', tf_name, look_up_t)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                   continue
                xsense_tf_mat[tf_index] = listener.fromTranslationRotation(*xsense_tf_mat_) 
                trans = (xsense_tf_mat_[0][0],xsense_tf_mat_[0][1],xsense_tf_mat_[0][2])
                quat = (0,0,0,1)
                xsense_tf_mat[tf_index] = numpy.dot(translation_matrix(trans), quaternion_matrix(quat)) # xsense tf matrix 
                 
            if msg is not None:
                marker =  msg.markers[0]  # get marker info 
                pose = marker.pose.pose
                pos = pose.position
                ori = pose.orientation
                ori = numpy.array([0,0,0,1]) # make marker's orientation be the same with xsense's orientation
                base_to_marker_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori[0], ori[1], ori[2], ori[3]))) # marker matrix
                xsense_tf_mat[4][:3,:3] = numpy.array([1,0,0,0,1,0,0,0,1]).reshape(3,3)
                t8_mat = xsense_tf_mat[4]
                xsens_base = numpy.dot(base_to_marker_mat,inv(t8_mat)) # transform between xsense and base
                trans = translation_from_matrix(xsens_base)
                quat = quaternion_from_matrix(xsens_base)
                broadcaster.sendTransform(
                    trans,
                    quat,
                    rospy.Time.now(),
                    'xsens',
                    'base', 
                )

                for tf_mat_index,tf_mat in enumerate(xsense_tf_mat):    
                    xsense_each_base = numpy.dot(tf_mat,xsens_base)
                    xsense_trans[tf_mat_index] = translation_from_matrix(xsense_each_base)
                    xsense_quat[tf_mat_index] = quaternion_from_matrix(xsense_each_base)
                
                # broadcaster.sendTransform(
                #     xsense_trans[4],
                #     xsense_quat[4],
                #     rospy.Time.now(),
                #     't8',
                #     'base', 
                # )
                for trans,quat,tf_name in zip(xsense_trans,xsense_quat,xsense_tf_new_names): 
                    broadcaster.sendTransform(
                        trans,
                        quat,
                        rospy.Time.now(),
                        tf_name,
                        'base', 
                    )

                    
