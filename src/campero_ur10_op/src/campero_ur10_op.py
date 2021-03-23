import argparse

import rospy
from campero_ur10_msgs.msg import MoveOp

TOPIC_NAME="campero_ur10_move"
NODE_NAME="campero_ur10_operator"

def main():
    args = parser.parse_args()
    
    pub = rospy.Publisher(TOPIC_NAME, MoveOp, queue_size=10)
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(100) 

    while not rospy.is_shutdown():
        msg = MoveOp()
        msg.type = MoveOp.MOVE_JOINT
        msg.id = MoveOp.C_UR10_SHOULDER_PAN_JOINT_IDX
        msg.value = 0.1
        pub.publish(msg)
        rate.sleep()
        
    rospy.loginfo("End program")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='CamperoUR10 Operator')
    #parser.add_argument("-s", "--size", type = float, help = 'size of screen NxN', default = board.SIZE_DEFAULT)
    #parser.add_argument("-l", "--len", type = float, help = 'tamano del tablero real', default = REAL_BOARD_SIZE)

    try:
        main()
    except rospy.ROSInterruptException:
        pass