import sys
import rospy

from visualization_msgs.msg import Marker, MarkerArray

try:
    rospy.init_node("test_marker", anonymous=True)

    pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)

    markerArray = MarkerArray()

    marker = Marker()
    marker.header.frame_id = "test"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = -0.59
    marker.pose.position.y = -0.19
    marker.pose.position.z = 0.79
       
    markerArray.markers.append(marker)
    pub.publish(markerArray)

except rospy.ROSInterruptException:
    pass
