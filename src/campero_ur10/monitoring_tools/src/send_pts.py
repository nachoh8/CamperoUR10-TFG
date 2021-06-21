#!/usr/bin/env python  
import rospy
import sys
import math
import tf
import geometry_msgs.msg

from campero_ur10_msgs.msg import ImgPoint, ImgTrace, ImageDraw


NUM_MUESTARS = 1000


markers = []
for i in range(1,len(sys.argv)):
	markers.append(sys.argv[i])

rospy.init_node('aruco_distance_measure', anonymous=True)
pub = rospy.Publisher("/image_points", ImageDraw, queue_size=1)

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

res = []
img = ImageDraw()
img.W = -1
img.H = -1
div = 0.029
for n in range(len(markers)):
	pos_media = [0,0,0]
	for pos in pos_vec[n]:
		pos_media = [pos_media[0] + pos[0], pos_media[1] + pos[1], pos_media[2] + pos[2]]
	l = len(pos_vec[n])
	pos_media = [pos_media[0]/l, pos_media[1]/l, pos_media[2]/l]
	rospy.loginfo("Posicion Media " + markers[n] + " (X,Y,Z): " + str(pos_media))
	
	center = pos_media
	tl = [pos_media[0] - div, pos_media[1] - div, pos_media[2]]
	tr = [pos_media[0] - div, pos_media[1] + div, pos_media[2]]
	br = [pos_media[0] + div, pos_media[1] + div, pos_media[2]]
	bl = [pos_media[0] + div, pos_media[1] - div, pos_media[2]]
	
	_trace = ImgTrace()
	imgPoint = ImgPoint()
	imgPoint.x = center[0]
	imgPoint.y = center[1]
	imgPoint.z = center[2]
	_trace.points.insert(0, imgPoint)
	img.traces.insert(0, _trace)
	
	_trace = ImgTrace()
	imgPoint = ImgPoint()
	imgPoint.x = tl[0]
	imgPoint.y = tl[1]
	imgPoint.z = tl[2]
	_trace.points.insert(0, imgPoint)
	img.traces.insert(1, _trace)
	
	_trace = ImgTrace()
	imgPoint = ImgPoint()
	imgPoint.x = tr[0]
	imgPoint.y = tr[1]
	imgPoint.z = tr[2]
	_trace.points.insert(0, imgPoint)
	img.traces.insert(2, _trace)
	
	_trace = ImgTrace()
	imgPoint = ImgPoint()
	imgPoint.x = br[0]
	imgPoint.y = br[1]
	imgPoint.z = br[2]
	_trace.points.insert(0, imgPoint)
	img.traces.insert(3, _trace)
	
	_trace = ImgTrace()
	imgPoint = ImgPoint()
	imgPoint.x = bl[0]
	imgPoint.y = bl[1]
	imgPoint.z = bl[2]
	_trace.points.insert(0, imgPoint)
	img.traces.insert(4, _trace)
	

print(img)
pub.publish(img)

rate.sleep()

rospy.loginfo("End Program")
