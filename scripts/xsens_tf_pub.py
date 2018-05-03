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

    while not rospy.is_shutdown():
            writable.clear()
            msg = copy.deepcopy(shared_msg)
            writable.set()

            if msg is not None:
                #ipdb.set_trace()
                look_up_t = rospy.Time(0)
                listener.waitForTransform('body_sonser', 't8', look_up_t, rospy.Duration(3))
                t8_to_bodysensor = listener.lookupTransform('body_sonser', 't8', look_up_t)
                t8_to_bodysensor_mat = listener.fromTranslationRotation(*t8_to_bodysensor) 


                for marker in msg.markers:
                    pose = marker.pose.pose
                    pos = pose.position
                    ori = pose.orientation
                    base_to_marker_mat = numpy.dot(translation_matrix((pos.x, pos.y, pos.z)), quaternion_matrix((ori.x, ori.y, ori.z, ori.w)))

                    t8_to_bodysensor_mat_inv = inv(t8_to_bodysensor_mat)
                    bodysensor_base = numpy.dot(base_to_marker_mat,t8_to_bodysensor_mat_inv)
            
                    trans = translation_from_matrix(bodysensor_base)
                    quat = quaternion_from_matrix(bodysensor_base)
                    broadcaster.sendTransform(
                        trans,
                        quat,
                        rospy.Time.now(),
                        'body_sonser',
                        'base', 
                    )
