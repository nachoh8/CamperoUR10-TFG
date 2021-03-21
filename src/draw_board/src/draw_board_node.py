#!/usr/bin/env python
import argparse
import rospy
from geometry_msgs.msg import Pose, PoseArray

import draw_board_cv as board

TOPIC_NAME="image_points"
NODE_NAME="draw_board"

REAL_BOARD_SIZE = 50

def points2Pose(points, div):
    poses = PoseArray()
    i = 0
    for pt in points:
        x,y = pt
        pose = Pose()
        pose.position.x = x * div
        pose.position.y = y * div
        poses.poses.insert(i, pose)
        i += 1
    
    return poses
        

def main():
    args = parser.parse_args()

    board_size = args.size
    real_size = args.len
    div = float(real_size)/float(board_size)
    print(div)
    board.setupBoard(board_size)
    

    pub = rospy.Publisher(TOPIC_NAME, PoseArray, queue_size=10)
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        code = board.main()
        if code == board.K_ESC:
            break
        elif code == board.K_SEND:
            rospy.loginfo("Send Image")
            poses = points2Pose(board.getPoints(), div)
            pub.publish(poses)

        #pub.publish()
        rate.sleep()
    
    rospy.loginfo("End program")
    board.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Draw on board')
    parser.add_argument("-s", "--size", type = float, help = 'size of screen NxN', default = board.SIZE_DEFAULT)
    parser.add_argument("-l", "--len", type = float, help = 'tamano del tablero real', default = REAL_BOARD_SIZE)

    try:
        main()
    except rospy.ROSInterruptException:
        pass
