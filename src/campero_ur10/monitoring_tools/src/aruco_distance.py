#!/usr/bin/env python  
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('aruco_distance_measure')

    listener = tf.TransformListener()

    rate = rospy.Rate(100.0)
    dis_vec = []

    rospy.loginfo("Obteniendo muestras")
    while not rospy.is_shutdown() and len(dis_vec) < 1000:
        try:
            (trans,rot) = listener.lookupTransform('/camera_color_optical_frame', '/aruco_marker_frame_1', rospy.Time(0))
            dis_vec.append(trans[0])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

    num_elem = len(dis_vec)
    rospy.loginfo("Num muestras: " + str(num_elem))
    sum_vec = sum(dis_vec)
    rospy.loginfo("Distancia Media: " + str(sum_vec / num_elem))
    rospy.loginfo("End Program")