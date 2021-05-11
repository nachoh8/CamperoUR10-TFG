import argparse
import rospy

from campero_ur10_msgs.msg import MoveOp
import keyboard_teleop as k_op

TOPIC_NAME="campero_ur10_move"
NODE_NAME="campero_ur10_operator"

modeJoint = True #true -> joint, false->carthesian
step = 0.01
increase_step = 0.01
max_step = 0.1
time_sleep = 1

def getID(ch):
    """
    devuelve el id del joint/axis correspondiente a la tecla ch, -1 si no existe
    """
    global modeJoint

    if ch == "q" or ch == "1":
        if modeJoint:
            return MoveOp.C_UR10_SHOULDER_PAN_JOINT_IDX
        else:
            return MoveOp.X_AXIS
    elif ch == "w" or ch == "2":
        if modeJoint:
            return MoveOp.C_UR10_SHOULDER_LIFT_JOINT_IDX
        else:
            return MoveOp.Y_AXIS
    elif ch == "e" or ch == "3":
        if modeJoint:
            return MoveOp.C_UR10_ELBOW_JOINT_IDX
        else:
            return MoveOp.Z_AXIS
    elif ch == "r" or ch == "4":
        if modeJoint:
            return MoveOp.C_UR10_W1_JOINT_IDX
        else:
            return MoveOp.RX_AXIS
    elif ch == "t" or ch == "5":
        if modeJoint:
            return MoveOp.C_UR10_W2_JOINT_IDX
        else:
            return MoveOp.RY_AXIS
    elif ch == "y" or ch == "6":
        if modeJoint:
            return MoveOp.C_UR10_W3_JOINT_IDX
        else:
            return MoveOp.RZ_AXIS
    
    return -1

def buildMsg(ch):
    """
    Devuelve un mensaje MoveOp a partir de la tecla ch y del step y modo actual
    """
    global step, modeJoint

    id = getID(ch)
    if id == -1: # not valid key
        return None

    v = 0
    try:
        n = int(ch) # if key is a number -> +
        v = step
    except ValueError: # else -> -
        v = -1*step
    
    # build msg
    msg = MoveOp()
    msg.type = MoveOp.MOVE_JOINT if modeJoint else MoveOp.MOVE_CARTHESIAN
    msg.id = id
    msg.value = v

    return msg

def help():
    rospy.loginfo("\n-----HELP-----\n"
            +"*Key:\n"
            +"- x -> change mode (MOVE_JOINT | MOVE_CARTHESIAN)\n"
            +"- z -> end program\n"
            +"- h -> show help\n"
            +"- i -> show info\n"
            +"- +/- -> increase/decrease amount of step\n\n"

            +"*MOVE_JOINT Mode:\n"
            +"- [1-6] -> move on positive direction joint [0-5]\n"
            +"- [q-y] -> move on negative direction joint [0-5]\n\n"

            +"*MOVE_CARTHESIAN Mode:\n"
            +"- [1-6] -> move endeffector on positive direction on [x,y,z,rx,ry,rz]\n"
            +"- [q-y] -> move endeffector on negative direction on [x,y,z,rx,ry,rz]\n")

def printInfo():
    global modeJoint, step, time_sleep, increase_step, max_step

    rospy.loginfo("\n-----INFO-----\nCurrent mode: "
                + ("MOVE_JOINT" if modeJoint else "MOVE_CARTHESIAN")
                + "\nStep: " + str(step)
                + "\nMax Step: " + str(max_step)
                + "\nMin Step: " + str(increase_step)
                + "\nTime_sleep: " + str(time_sleep))

def main():
    global modeJoint, step, time_sleep, increase_step, max_step

    # SETUP
    args = parser.parse_args()

    max_step = args.MaxStep
    increase_step = args.Step
    time_sleep = args.time
    
    pub = rospy.Publisher(TOPIC_NAME, MoveOp, queue_size=1)
    rospy.init_node(NODE_NAME, anonymous=True)

    help()
    printInfo()

    # INIT
    rospy.loginfo("-----READY-----")
    finished = False

    while not rospy.is_shutdown() and not finished:
        k_op.clear_input()
        ch = k_op.read_key().lower()

        if ch == "z": # exit
            finished = True
        elif ch == "x": # change mode
            modeJoint = not modeJoint
            rospy.loginfo("Mode changed to " + ("Joint" if modeJoint else "Carthesian"))
            printInfo()
        elif ch == "h": # show help
            help()
        elif ch == "i": # show info
            printInfo()
        elif ch == "+": # increase step
            v = step + increase_step
            step = v if v < max_step else max_step
            rospy.loginfo("Step: " + str(step))

        elif ch == "-": # decrease step
            v = step - increase_step
            step = v if v > increase_step else increase_step
            rospy.loginfo("Step: " + str(step))

        else:
            msg = buildMsg(ch)
            if msg is not None:
                #rospy.loginfo("Moving")
                pub.publish(msg)

        rospy.sleep(time_sleep)
        
    rospy.loginfo("End program")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='CamperoUR10 Operator')
    parser.add_argument("-Ms", "--MaxStep", type = float, help = 'max step of each movement', default = max_step)
    parser.add_argument("-s", "--Step", type = float, help = 'step of each movement', default = increase_step)
    parser.add_argument("-t", "--time", type = float, help = 'time to sleep in seconds', default = time_sleep)

    try:
        main()
    except rospy.ROSInterruptException:
        pass