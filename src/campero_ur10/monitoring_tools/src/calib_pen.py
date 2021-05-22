import sys
import numpy as np

from moveit_commander import move_group
import rospy
import moveit_commander
import tf
from geometry_msgs.msg import Pose
import tf_conversions.posemath as pm


from campero_ur10_msgs.msg import ArucoMarker, ArucoMarkerArray

NODE_NAME = "calib_pen"
TOPIC_ARUCO = "/aruco_detector/markers_pose"

EE_FRAME = "wrist_3_link"
CAMERA_FRAME = "camera_color_optical_frame"
WORLD_FRAME = "campero_base_footprint"
ARUCO_MARKER_ID = 3
ARUCO_FRAME = "aruco_marker_frame_" + str(ARUCO_MARKER_ID)

tf_listener = None

take_marker = False
marker_ref = None

def printTf(tf_pm):
	print(pm.toMsg(tf_pm))

def tfToNp(tf_m):
	t_mat = tf.transformations.translation_matrix(tf_m[0])
	r_mat = tf.transformations.quaternion_matrix(tf_m[1])
	return np.dot(t_mat, r_mat)

def process(markers_msg):
	global marker_ref, take_marker
	if not take_marker:
		return

	markers = markers_msg.markers
	for m in markers:
		if m.id == ARUCO_MARKER_ID:
			take_marker = False
			marker_ref = (m.id, m.pose)

def getTransform(child_frame, ref_frame):
	global tf_listener
	try:
		"""
		(trans, rot) = tf_listener.lookupTransform(ref_frame, child_frame, rospy.Time(0))
		t_mat = tf.transformations.translation_matrix(trans)
		r_mat = tf.transformations.quaternion_matrix(rot)
		return np.dot(t_mat, r_mat)
		"""
		tf_m = tf_listener.lookupTransform(ref_frame, child_frame, rospy.Time(0))
		#return tfToNp(tf_m)
		return pm.fromTf(tf_m)
		
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		print("Error consiguiendo la transformacion")
		exit(1)

def main():
	global tf_listener, marker_ref, take_marker
	
	rospy.init_node(NODE_NAME, anonymous=True)
	
	tf_listener = tf.TransformListener()
     
	rospy.Subscriber(TOPIC_ARUCO, ArucoMarkerArray, process, queue_size=1)

	rospy.loginfo("WORLD_FRAME: " + WORLD_FRAME)
	rospy.loginfo("EE_FRAME: " + EE_FRAME)
	rospy.loginfo("CAMERA_FRAME: " + CAMERA_FRAME)
	rospy.loginfo("ARUCO_FRAME: " + ARUCO_FRAME)

	rospy.loginfo("---Calib Ready---")
	
	_ = raw_input("---Press Enter To Take First Position--- ")
	
	tf1_EE_W = getTransform(EE_FRAME, WORLD_FRAME)
	tf_CAM_EE = getTransform(CAMERA_FRAME, EE_FRAME)

	print("TF1_EE_W")
	printTf(tf1_EE_W)
	
	print("TF_CAM_EE")
	printTf(tf_CAM_EE)

	_ = raw_input("---Press Enter To Take Second Position--- ")
	
	print("Esperando a reconocer marca aruco")
	take_marker=True
	while marker_ref is None:
		rospy.sleep(0.1)
	
	print("Marca Aruco: ", marker_ref)
	# tf_ARUCO_CAM = getTransform(ARUCO_FRAME + str(marker_ref[0]), CAMERA_FRAME)
	tf_ARUCO_CAM = pm.fromMsg(marker_ref[1])
	#tf_ARUCO_CAM = tfToNp(tf_ARUCO_CAM)
	print("TF_ARUCO_CAM")
	printTf(tf_ARUCO_CAM)


	tf2_EE_W = getTransform(EE_FRAME, WORLD_FRAME)
	print("TF2_EE_W")
	printTf(tf2_EE_W)

	# ----Operaciones----
	print("TF_ARUCO_CAM * TF_CAM_EE")
	res = tf_ARUCO_CAM * tf_CAM_EE
	printTf(res)
	print("res * tf2_EE_W")
	res = res * tf2_EE_W
	printTf(res)
	print("tf_PEN_EE = res * tf1_EE_W.inv")
	tf1_EE_W = pm.toMatrix(tf1_EE_W)
	tf1_EE_W = tf.transformations.inverse_matrix(tf1_EE_W)
	tf1_EE_W = pm.fromMatrix(tf1_EE_W)
	res = res * tf1_EE_W
	printTf(res)
	"""
	print("tf_CAM_W = TF_CAM_EE * tf2_EE_W")
	tf_CAM_W = tf_CAM_EE * tf2_EE_W
	printTf(tf_CAM_W)

	print("tf_ARUCO_W = tf_ARUCO_CAM * tf_CAM_W")
	tf_ARUCO_W = tf_ARUCO_CAM * tf_CAM_W
	printTf(tf_ARUCO_W)

	print("tf_PEN_EE = tf_ARUCO_W * tf1_EE_W.inv")
	#aux = pm.toMsg(tf1_EE_W) # pose
	#tf_aux = 
	tf1_EE_W = pm.toMatrix(tf1_EE_W)
	#print(tf1_EE_W)
	tf1_EE_W = tf.transformations.inverse_matrix(tf1_EE_W)
	#tf1_EE_W = pm.fromTf(tf1_EE_W)
	#printTf(tf_PEN_EE)

	tf_PEN_EE = tf_ARUCO_W * pm.fromMatrix(tf1_EE_W)
	printTf(tf_PEN_EE)
	"""

	rospy.loginfo("End program")
	

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
