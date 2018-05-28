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

writable = threading.Event()
writable.clear()
shared_msg = None

def cb(msg):
    global writable, shared_msg
    if writable.is_set():
        shared_msg = msg


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

    xsense_tf_mat = [None] * 23

    while not rospy.is_shutdown():
            writable.clear()
            msg = copy.deepcopy(shared_msg)
            writable.set()
            
            for tf_index,tf_name in enumerate(xsense_tf_names):
                look_up_t = rospy.Time(0)
                listener.waitForTransform('body_sensor', tf_name, look_up_t, rospy.Duration(3))
                xsense_tf_mat[tf_index] = listener.lookupTransform('body_sensor', tf_name, look_up_t)
                xsense_tf_mat[tf_index] = listener.fromTranslationRotation(*xsense_tf_mat[tf_index]) 

            for marker in msg.markers:
                pose = msg.markers[0].pose.pose
                pos = pose.position
                ori = pose.orientation
                base_to_marker_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))

                t8_to_bodysensor_mat_inv = inv(t8_to_bodysensor_mat)
                bodysensor_base = numpy.dot(base_to_marker_mat,t8_to_bodysensor_mat_inv)
        
                trans = translation_from_matrix(bodysensor_base)
                quat = quaternion_from_matrix(bodysensor_base)
                base_xsen = numpy.hstack((trans,quat))
                print "________"
                print "base to xsen transform"
                print base_xsen
                print "________"

                    
