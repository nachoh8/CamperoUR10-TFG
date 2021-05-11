#!/usr/bin/env python
import rospy
from campero_ur10_msgs.msg import ImageDraw

TOPIC_NAME="image_points"
NODE_NAME="draw_board"

class DrawBoardNode:
    def __init__(self):
        self.pub = rospy.Publisher(TOPIC_NAME, ImageDraw, queue_size=1)
        rospy.init_node(NODE_NAME, anonymous=True)

    def publishImage(self, image):
        rospy.loginfo("Send Iamge")
        self.pub.publish(image)
    
    def close(self):
        rospy.loginfo("End Node")
        self.pub.unregister()
        rospy.signal_shutdown("Board Closed")

    def isClosed(self):
        return rospy.is_shutdown()