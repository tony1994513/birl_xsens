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


if __name__ == '__main__':
    rospy.init_node("xens_calibration")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, cb)
    listener = tf.TransformListener()
    xsense_tf_names = ["pelvis_xsens", "l5_abdomen_down_xsens", "l3_abdomen_up_xsens", "t12_sternum_down_xsens", "t8_sternum_up_xsens", "neck_xsens", "head_xsens",
    "right_shoulder_xsens", "right_upper_arm_xsens", "right_forearm_xsens", "right_hand_xsens", "left_shoulder_xsens", "left_upper_arm_xsens", "left_forearm_xsens",
    "left_hand_xsens", "right_upper_leg_xsens", "right_lower_leg_xsens", "right_foot_xsens", "right_toe_xsens", "left_upper_leg_xsens", "left_lower_leg_xsens", 
    "left_foot_xsens", "left_toe_xsens"]


    xsense_tf_mat = [None] * 23
    xsense_tf_quat = [None] * 23
    calibrated_trans = ( 1.89846022,  1.06179248, -1.06540353)
    # marker_pos = (1.65030752, -0.17223643,  0.28370678)
    rospy.loginfo("Node started, shutdown in 3s if no tf coming ")
    rospy.loginfo("Calibrating")
    trans_list = []
    marker_list = []
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
                xsense_tf_quat[tf_index] = xsense_tf_mat_[1]
                xsense_tf_mat[tf_index] = listener.fromTranslationRotation(*xsense_tf_mat_) 
                trans = (xsense_tf_mat_[0][0],xsense_tf_mat_[0][1],xsense_tf_mat_[0][2])
                quat = (0,0,0,1)
                xsense_tf_mat[tf_index] = numpy.dot(translation_matrix(trans), quaternion_matrix(quat)) # xsense tf matrix 

            
            if msg is not None:
                # ipdb.set_trace()
                if msg.markers[0].id == 0:   
                    marker =  msg.markers[0]  # get marker info 
                    pose = marker.pose.pose
                    pos = pose.position
                    ori = pose.orientation
                    ori = numpy.array([0,0,0,1]) # make marker's orientation be the same with xsense's orientation
                    base_to_marker_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori[0], ori[1], ori[2], ori[3]))) # marker matrix
                    t8_mat = xsense_tf_mat[4]
                    t8_mat[:3,:3] = numpy.array([1,0,0,0,1,0,0,0,1]).reshape(3,3)
                    xsens_base = numpy.dot(base_to_marker_mat,inv(t8_mat)) # transform between xsense and base
                    trans = translation_from_matrix(xsens_base)
                    trans_list.append(trans)
                    marker_list.append(numpy.array([pos.x, pos.y, pos.z]))
                    if len(trans_list) > 500:
                        calibration_result = numpy.mean(numpy.array(trans_list), axis=0)
                        marker_mean = numpy.mean(numpy.array(marker_list), axis=0)
                        rospy.loginfo("Mean of marker is %s" %marker_mean)
                        rospy.loginfo("Calibration result is %s" %calibration_result)
                        rospy.loginfo("Complete")
                        break


                    # rospy.loginfo("calibration result is %s" %trans)
                    # marker_pos_now = numpy.array([pos.x, pos.y, pos.z])
                    # rospy.loginfo("present marker pose is %s" %marker_pos_now)
                    # rospy.loginfo("error bettwen present and previous marker is %s" %(marker_pos_now - marker_pos))
                    # rospy.loginfo("error bettwen present and previous calibration is %s" %(trans - calibrated_trans))

                    
