import sys
from moveit_commander import move_group
import rospy
import moveit_commander

from campero_ur10_msgs.msg import JointSpeed

NODE_NAME = "stop_campero_ur10"
C_UR10_PLANNING_GROUP = "manipulator"
TOPIC_SUB="joint_speed"

TIME_SLEEP = 0.1

MAX_SPEED = 0.5

SPEED_MONITORING = False

move_group = None

def stop():
    global move_group

    rospy.loginfo("Stopping robot")
    move_group.stop()

def process(joint_speed):
    data = joint_speed.data
    
    # solo comprobar shoulder_pan_joint, shoulder_lift_joint, elbow
    if data[JointSpeed.C_UR10_SHOULDER_PAN_JOINT_IDX] > MAX_SPEED or data[JointSpeed.C_UR10_SHOULDER_LIFT_JOINT_IDX] > MAX_SPEED or data[JointSpeed.C_UR10_ELBOW_JOINT_IDX] > MAX_SPEED:
       stop()

       rospy.loginfo("Warning: Velocidad maxima alcanzada")
       rospy.loginfo(data)



def main():
    global move_group

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node(NODE_NAME, anonymous=True)

    move_group = moveit_commander.MoveGroupCommander(C_UR10_PLANNING_GROUP)

    if SPEED_MONITORING:
        rospy.loginfo("---Monitoring Speed: Active---")
        rospy.Subscriber(TOPIC_SUB, JointSpeed, process, queue_size=1)

    rospy.loginfo("---Stop Program Ready---")
    
    finish = False
    
    while not rospy.is_shutdown() and not finish:
        s = raw_input("---Press Enter to Stop the Robot or z to end program--- ")
        if s == "":
            stop()
        elif s == "z":
            finish = True
        
        rospy.sleep(TIME_SLEEP)

    rospy.loginfo("End program")


if __name__ == '__main__':
    """
    Uso:
        python stop_robot.py [-v [<max_speed>]]
        -v: opcional, activa detencion al alcanzar max_speed
        <max_speed>: opcional, por defecto es 0.5 rad/s
    """
    if len(sys.argv) == 2 and sys.argv[1] == "-v":
        SPEED_MONITORING = True
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass