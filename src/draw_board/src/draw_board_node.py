#!/usr/bin/env python
import argparse
import rospy
from geometry_msgs.msg import Pose, PoseArray
from campero_ur10_msgs.msg import ImagePoint, ImageDraw

import draw_board_cv as board

TOPIC_NAME="image_points"
NODE_NAME="draw_board"

def createImg(points, size):
    if len(points) == 0:
        return None
    img = ImageDraw()
    i = 0
    for pt in points:
        x,y = pt
        if x >= 0 and x < size and y >= 0 and y < size:
            imgPoint = ImagePoint()
            imgPoint.x = pt[0]
            imgPoint.y = pt[1]
            img.points.insert(i, imgPoint)
            i += 1
    
    img.size = size

    return img
        

def main():
    args = parser.parse_args()

    board_size = args.size
    board.setupBoard(board_size)
    

    pub = rospy.Publisher(TOPIC_NAME, ImageDraw, queue_size=1)
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        code = board.main()
        if code == board.K_ESC:
            break
        elif code == board.K_SEND:
            img = createImg(board.getPoints(), board_size)
            if img == None:
                rospy.loginfo("Image Empty")
            else:
                rospy.loginfo("Send Image")
                pub.publish(img)

        #pub.publish()
        rate.sleep()
    
    rospy.loginfo("End program")
    board.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Draw on board')
    parser.add_argument("-s", "--size", type = float, help = 'size of screen NxN', default = board.SIZE_DEFAULT)

    try:
        main()
    except rospy.ROSInterruptException:
        pass
