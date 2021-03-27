#!/usr/bin/env python
import argparse
import rospy
from geometry_msgs.msg import Pose, PoseArray
from campero_ur10_msgs.msg import ImageDraw

from draw_board_cv import BoardCV
#from draw_board_turtle import BoardTurtle

TOPIC_NAME="image_points"
NODE_NAME="draw_board"

SIZE_DEFAULT = 512

CV_TYPE=0
TURTLE_TYPE=1

K_ESC = 27 # escape
K_SEND = 115 # s

def getBoard(_type, size):
    if _type == CV_TYPE:
        return BoardCV(size)
    elif _type == TURTLE_TYPE:
        return None#BoardTurtle(size)
    else:
        return None


"""
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
"""

def main():
    args = parser.parse_args()

    board_size = args.size
    board = getBoard(args.type, board_size)
    
    if board == None:
        print("Board not valid")
        exit(1)
    

    pub = rospy.Publisher(TOPIC_NAME, ImageDraw, queue_size=1)
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(10)

    print("Board Ready")
    while not rospy.is_shutdown():
        code = board.main()
        if code == K_ESC:
            break
        elif code == K_SEND:
            img = board.getImgDraw()
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
    parser.add_argument("-s", "--size", type = float, help = 'size of screen NxN', default = SIZE_DEFAULT)
    parser.add_argument("-t", "--type", type = int , help = 'board type', default = CV_TYPE)


    try:
        main()
    except rospy.ROSInterruptException:
        pass
