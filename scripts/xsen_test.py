 #!/usr/bin/env python
import rospy
import tf
import ipdb
import copy
import numpy   



if __name__ == '__main__':
    rospy.init_node("xens_tf_pub")
    listener = tf.TransformListener()   
    publish_rate = 100
    r = rospy.Rate(publish_rate)
    data_list = []
    while not rospy.is_shutdown():   
            look_up_t = rospy.Time(0)                   
            listener.waitForTransform("right_hand_xsens_new","base", look_up_t, rospy.Duration(1))
            try:
                transform = listener.lookupTransform("right_hand_xsens_new","base", look_up_t)
                rospy.loginfo(transform[0])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            r.sleep()

 