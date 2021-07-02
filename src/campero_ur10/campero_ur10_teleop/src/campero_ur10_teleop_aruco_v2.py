import argparse
import rospy
import sys
import math
import tf
import geometry_msgs.msg

from campero_ur10_msgs.msg import MoveOp, ArucoMarkerArray, ArucoMarker
import keyboard_teleop as k_op

WORLD_FRAME='/campero_base_footprint'

AXIS_X = "x"
X_IDX = 0
AXIS_Y = "y"
Y_IDX = 1

NUM_SAMPLES = 1000 # num muestras para medir una posicon

MARKER_ID = 0 # id de la marca a seguir

# MIN_D_TIME = 0.5 # minimo tiempo en segundos que tiene que transcurrir entre dos posiciones distintas

MIN_D_POS = 0.01 # minima distancia en metros que tiene que moverse la marca entre dos posiciones distintas

# dist = lambda p1,p2: math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

pub = None

read_markers = False

last_pos = None
move_op = None

def send(v):
    global pub

    if pub is None: return

    msg = MoveOp()
    msg.type = MoveOp.MOVE_CARTHESIAN
    msg.id = MoveOp.ALL_CARTH_AXIS
    msg.vx = v[0]
    msg.vy = v[1]
    msg.vz = 0

    rospy.loginfo("Send Operation? y -> yes")
    k_op.clear_input()
    ch = k_op.read_key().lower()

    if ch == "y": # send message
        rospy.loginfo("Operation send")
        pub.publish(msg)
    else:
        rospy.loginfo("Operation canceled")

def move_robot(last_pos, new_pos):
    global MIN_D_POS
        
    dX = last_pos[0] - new_pos[0]
    dY = new_pos[1] - last_pos[1]
    v = [0,0]

    if abs(dX) > MIN_D_POS:
        v[0] = dX

    if abs(dY) > MIN_D_POS:
        v[1] = dY

    if v[0] != 0 or v[1] != 0:
        rospy.loginfo("Move: " + str(v))
        send(v)
        return new_pos

    return last_pos

def rec_position(last_pos, listener, xx, yy):
    global WORLD_FRAME, NUM_SAMPLES, MARKER_ID
    while True:
        try:
            samples_x = 0.0
            samples_y = 0.0
            for _ in range(NUM_SAMPLES):
                (trans,_) = listener.lookupTransform(WORLD_FRAME, "/aruco_marker_frame_" + str(MARKER_ID), rospy.Time(0))
                samples_x += trans[xx]
                samples_y += trans[yy]
            new_pos = [samples_x / float(NUM_SAMPLES), samples_y / float(NUM_SAMPLES)]
            
            if last_pos is None:
                rospy.loginfo("New Pos: " + str(new_pos))
                return new_pos
            else:
                rospy.loginfo("Last Pos: " + str(last_pos) + " New Pos: " + str(new_pos))
                return move_robot(last_pos, new_pos)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

def receive_markers(data):
    global MARKER_ID, X_IDX, Y_IDX, read_markers, last_pos, move_op

    markers = data.markers
    if not read_markers or len(markers) == 0:
        return
    
    for marker in markers:
        if marker.id == MARKER_ID:
            new_pos = 


    

def main():
    global WORLD_FRAME, AXIS_X, AXIS_Y, X_IDX, Y_IDX, NUM_SAMPLES, MARKER_ID, MIN_D_POS, pub, read_markers

    rospy.init_node('campero_ur10_teleop_aruco')

    listener = tf.TransformListener()

    rate = rospy.Rate(100.0)

    pub = rospy.Publisher("campero_ur10_move", MoveOp, queue_size=1)
    rospy.Subscriber("/aruco_detector/markers_pose", ArucoMarkerArray, receive_markers, queue_size=1)

    if AXIS_X.lower() == "z":
        X_IDX = 2
    elif AXIS_X.lower() == "y":
        X_IDX = 1
    else:
        X_IDX = 0
    
    if AXIS_Y.lower() == "z":
        Y_IDX = 2
    elif AXIS_Y.lower() == "x":
        Y_IDX = 0
    else:
        Y_IDX = 1

    info = "World frame: " + str(WORLD_FRAME) + "\n"
    info += "Aruco Marker id: " + str(MARKER_ID) + "\n"
    info += "Num. samples per position: " + str(NUM_SAMPLES) + "\n"
    info += "Min. distance between positions: " + str(MIN_D_POS) + "\n"
    info += "Original Axis " + AXIS_X + " remap to X\n"
    info += "Original Axis " + AXIS_Y + " remap to Y\n"
    rospy.loginfo(info)

    rospy.loginfo("Teleop Ready")

    last_pos = None
    finished = False
    while not rospy.is_shutdown() and not finished:
        k_op.clear_input()
        ch = k_op.read_key().lower()

        if ch == "z": # exit
            finished = True
        elif ch == "r": # save position
            # last_pos = rec_position(last_pos, listener, xx, yy)
            read_markers = True
        elif ch == "s": # send movement
            # last_pos = rec_position(last_pos, listener, xx, yy)
            send(None)
        elif ch == "c": # clear last pos
            rospy.loginfo("Last Position Removed")
            last_pos = None
            
        rate.sleep()
    

    rospy.loginfo("End Program")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Campero UR10 ARUCO Teleoperator')
    parser.add_argument("-ID", "--id", type = int, help = 'marker id', default = MARKER_ID)
    parser.add_argument("-S", "--ns", type = int, help = 'num samples', default = NUM_SAMPLES)
    parser.add_argument("-D", "--dp", type = float, help = 'min distance', default = MIN_D_POS)
    parser.add_argument("-W", "--w", type = str, help = 'world_frame', default = WORLD_FRAME)
    parser.add_argument("-A_X", "--x", type = str, help = 'original axis to remap x', default = AXIS_X)
    parser.add_argument("-A_Y", "--y", type = str, help = 'original axis to remap y', default = AXIS_Y)

    args = parser.parse_args()
    try:
        WORLD_FRAME = args.w
        MARKER_ID = args.id
        NUM_SAMPLES = args.ns
        MIN_D_POS = args.dp
        AXIS_X = args.x
        AXIS_Y = args.y

        main()
    except rospy.ROSInterruptException:
        pass