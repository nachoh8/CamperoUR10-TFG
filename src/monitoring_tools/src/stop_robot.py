import sys
import rospy
import moveit_commander

NODE_NAME = "stop_campero_ur10"
C_UR10_PLANNING_GROUP = "manipulator"

TIME_SLEEP = 0.1

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(NODE_NAME, anonymous=True)

    group = moveit_commander.MoveGroupCommander(C_UR10_PLANNING_GROUP)

    rospy.loginfo("---Stop Program Ready---")
    
    finish = False
    
    while not rospy.is_shutdown() and not finish:
        s = raw_input("---Press Enter to Stop the Robot or z to end program--- ")
        if s == "":
            rospy.loginfo("Stopping robot")
            group.stop()
        elif s == "z":
            finish = True
        
        rospy.sleep(TIME_SLEEP)

    rospy.loginfo("End program")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass