#!/usr/bin/env python  
""" aruco_tf.py - Version 1.0 10-8-2020
    Autor: David Barrera
    Codigo modificado a partir de: http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29
"""

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(48)
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0,0.7071, 0.0, 0.7071),
                         rospy.Time.now(),
                         "camera_aux_tf",
                         "campero_front_ptz_camera_frame_link")
        rate.sleep()