#!/usr/bin/env python  
import rospy
import sys
import math
import tf
import geometry_msgs.msg

NUM_MUESTARS = 1000


markers = []
for i in range(1,len(sys.argv)):
	markers.append(sys.argv[i])

rospy.init_node('aruco_distance_measure')

listener = tf.TransformListener()

rate = rospy.Rate(100.0)
pos_vec = [[] for _ in markers]

rospy.loginfo("Obteniendo muestras")
finished = False
i = 0
while not rospy.is_shutdown() and i < NUM_MUESTARS:
	i += 1
	for n in range(len(markers)):
		try:
			if len(pos_vec[n]) < NUM_MUESTARS:
				(trans,_) = listener.lookupTransform('/campero_base_footprint', markers[n], rospy.Time(0))
				pos_vec[n].append(trans)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
	rate.sleep()
for n in range(len(markers)):
	pos_media = [0,0,0]
	for pos in pos_vec[n]:
		pos_media = [pos_media[0] + pos[0], pos_media[1] + pos[1], pos_media[2] + pos[2]]
	l = len(pos_vec[n])
	pos_media = [pos_media[0]/l, pos_media[1]/l, pos_media[2]/l]
	rospy.loginfo("Posicion Media " + markers[n] + " (X,Y,Z): " + str(pos_media))


rospy.loginfo("End Program")
