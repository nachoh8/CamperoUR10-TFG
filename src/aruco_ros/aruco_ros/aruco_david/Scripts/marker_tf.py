#!/usr/bin/env python  
""" aruco_tf.py - Version 1.0 10-8-2020
    Autor: David Barrera
    Codigo modificado a partir de: http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29
"""

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster2')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(48)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.5,-0.5, 0.5, -0.5),
                         rospy.Time.now(),
                         "marker_tf",
                         "aruco_marker_frame")
        rate.sleep()
