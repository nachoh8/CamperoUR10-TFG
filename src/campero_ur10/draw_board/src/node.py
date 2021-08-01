#!/usr/bin/env python
import rospy
from campero_ur10_msgs.msg import ImageDraw

TOPIC_NAME="image_points"
NODE_NAME="draw_board"

class DrawNode:
    def __init__(self, suffix = ""):
        self.pub = rospy.Publisher(TOPIC_NAME, ImageDraw, queue_size=1)
        name = NODE_NAME if suffix is None or len(suffix) == 0 else NODE_NAME + "_" + suffix
        rospy.init_node(name, anonymous=True)

    def publishImage(self, image):
        rospy.loginfo("Send Iamge")
        self.pub.publish(image)
    
    def close(self):
        rospy.loginfo("End Node")
        self.pub.unregister()
        rospy.signal_shutdown("Board Closed")

    def isClosed(self):
        return rospy.is_shutdown()