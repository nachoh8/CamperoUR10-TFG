#!/usr/bin/env python
""" transform_listener.py - Version 1.0 10-8-2020
    Autor: David Barrera
    Codigo modificado a partir de: http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
"""

import rospy
import math
import tf
import numpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

x=0.0
y=0.0
z=0.0

q_x=0.0
q_y=0.0
q_z=0.0
q_w=0.0

x_list=[]
y_list=[]
theta_list=[]

pose=PoseStamped()	
	
if __name__ == '__main__':
	rospy.init_node('aruco_listener_distance',anonymous=True)
	pub = rospy.Publisher('aruco_distance', PoseStamped, queue_size=10)
	tf_listener=tf.TransformListener()
	tf_listener.waitForTransform('campero_base_footprint','marker_tf',rospy.Time(), rospy.Duration(0.5)) #Simulacion 0.1			#Cambiar aqui las transformaciones y abajo tambien
	rate = rospy.Rate(12)

	while not rospy.is_shutdown():
		try:		
			(trans,rot)=tf_listener.lookupTransform('campero_base_footprint','marker_tf',rospy.Time(0))			#Cambiar las transformaciones
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		x=trans[0]
		y=trans[1]
		z=trans[2]

		q_x=rot[0]
		q_y=rot[1]
		q_z=rot[2]
		q_w=rot[3]

		angles=tf.transformations.euler_from_quaternion(quaternion=(q_x, q_y, q_z, q_w))

		alpha=numpy.rad2deg(angles[0])
		beta=numpy.rad2deg(angles[1])
		theta=numpy.rad2deg(angles[2])

		pose.pose.position.x=x
		pose.pose.position.y=y
		pose.pose.position.z=z
		pose.pose.orientation.x=q_x
		pose.pose.orientation.y=q_y
		pose.pose.orientation.z=q_z
		pose.pose.orientation.w=q_w

		# El valor de la mediana no lo publicaremos solamente se utiliza para mostrarlo por pantalla
		x_list.append(x)
		if len(x_list)>9:
			x_list.remove(x_list[0])
		x_median=numpy.median(x_list)

		y_list.append(y)
		if len(y_list)>9:
			y_list.remove(y_list[0])
		y_median=numpy.median(y_list)

		theta_list.append(theta)
		if len(theta_list)>9:
			theta_list.remove(theta_list[0])
		theta_median=numpy.median(theta_list)

		rospy.loginfo('La marca se encuentra la posicion: x= %f , y= %f ,z= %f con la orientacion: alpha= %f , beta= %f , theta= %f', x,y,z,alpha,beta,theta)
		rospy.loginfo('x= %f ', x)
		rospy.loginfo('x_median= %f ', x_median)
		rospy.loginfo('y= %f ', y)
		rospy.loginfo('y_median= %f ', y_median)
		rospy.loginfo('theta= %f ', theta)
		rospy.loginfo('theta_median= %f ', theta_median)

		pub.publish(pose)
		rate.sleep()	
