import argparse
import rospy
import sys
import math
import tf
import geometry_msgs.msg

from campero_ur10_msgs.msg import MoveOp, ArucoMarkerArray, ArucoMarker
import keyboard_teleop as k_op

NUM_SAMPLES = 1000 # num muestras para medir una posicon

MARKER_ID = 0 # id de la marca a seguir

# MIN_D_TIME = 0.5 # minimo tiempo en segundos que tiene que transcurrir entre dos posiciones distintas

MIN_D_POS = 0.01 # minima distancia en metros que tiene que moverse la marca entre dos posiciones distintas
MAX_D_POS = 0.01

# dist = lambda p1,p2: math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

pub = None

read_markers = False

last_pos = None
new_pos = [0.0, 0.0]
avg_count = 0

def move_robot(last_pos, new_pos):
    global MIN_D_POS
        
    dX = new_pos[0] - last_pos[0]
    dY = new_pos[1] - last_pos[1]
    v = [0,0]

    if abs(dX) > MIN_D_POS:
        v[0] = dX

    if abs(dY) > MIN_D_POS:
        v[1] = dY

    return v

def send():
    global pub, last_pos, new_pos, avg_count

    if pub is None: 
        rospy.logerr("Error: no hay publisher")
        return
    
    if last_pos is None: 
        rospy.logerr("Error: no hay posicion inicial")
        return
    
    if new_pos[0] == 0 and new_pos[1] == 0:
        rospy.logerr("Error: no hay posicion final")
        return

    op = move_robot(last_pos, new_pos)
    last_pos = new_pos
    new_pos = [0.0, 0.0]
    avg_count = 0

    if op[0] == 0 and op[1] == 0:
        rospy.loginfo("No se ha superado el limite minimo de distancia")
        return
    
    rospy.loginfo("Operation: " + str(op))
    rospy.loginfo("Send Operation? y -> yes")
    k_op.clear_input()
    ch = k_op.read_key().lower()

    if ch == "y": # send message
        msg = MoveOp()
        msg.type = MoveOp.MOVE_CARTHESIAN
        msg.id = MoveOp.ALL_CARTH_AXIS
        msg.vy = op[0]
        msg.vx = -op[1]
        msg.vz = 0

        pub.publish(msg)

        rospy.loginfo("Operation send")
    else:
        rospy.loginfo("Operation canceled")

def receive_markers(data):
    global MARKER_ID, NUM_SAMPLES, read_markers, last_pos, new_pos, avg_count

    markers = data.markers
    if not read_markers or len(markers) == 0:
        return
    
    for marker in markers:
        if marker.id == MARKER_ID:
            new_pos[0] += marker.pose.position.x
            new_pos[1] += -marker.pose.position.y
            avg_count += 1

            if avg_count < NUM_SAMPLES:
                break
            
            read_markers = False
            new_pos = [new_pos[0] / float(NUM_SAMPLES), new_pos[1] / float(NUM_SAMPLES)]
            if last_pos is None:
                last_pos = new_pos
                new_pos = [0.0, 0.0]
                avg_count = 0
                rospy.loginfo("New Pos: " + str(last_pos))
            else:
                rospy.loginfo("Last Pos: " + str(last_pos) + " New Pos: " + str(new_pos))
            
            break

def main():
    global NUM_SAMPLES, MARKER_ID, MIN_D_POS, pub, last_pos, new_pos, read_markers, avg_count

    rospy.init_node('campero_ur10_teleop_aruco')

    rate = rospy.Rate(100.0)

    pub = rospy.Publisher("campero_ur10_move", MoveOp, queue_size=1)
    rospy.Subscriber("/aruco_detector/markers_pose", ArucoMarkerArray, receive_markers, queue_size=1)

    info = "Aruco Marker id: " + str(MARKER_ID) + "\n"
    info += "Num. samples per position: " + str(NUM_SAMPLES) + "\n"
    info += "Min. distance between positions: " + str(MIN_D_POS) + "\n"
    rospy.loginfo(info)

    rospy.loginfo("Teleop Ready")

    finished = False
    while not rospy.is_shutdown() and not finished:
        k_op.clear_input()
        ch = k_op.read_key().lower()

        if ch == "z": # exit
            finished = True
        elif ch == "r": # save position
            if not read_markers:
                rospy.loginfo("Start saving position")
                new_pos = [0.0, 0.0]
                avg_count = 0
                read_markers = True
        elif ch == "s": # send movement
            # last_pos = rec_position(last_pos, listener, xx, yy)
            send()
        elif ch == "c": # clear last pos
            rospy.loginfo("Reset positions")
            read_markers = False
            last_pos = None
            new_pos = [0.0, 0.0]
            avg_count = 0
            
        rate.sleep()
    

    rospy.loginfo("End Program")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Campero UR10 ARUCO Teleoperator')
    parser.add_argument("-ID", "--id", type = int, help = 'marker id', default = MARKER_ID)
    parser.add_argument("-S", "--ns", type = int, help = 'num samples', default = NUM_SAMPLES)
    parser.add_argument("-D", "--dp", type = float, help = 'min distance', default = MIN_D_POS)

    args = parser.parse_args()
    try:
        MARKER_ID = args.id
        NUM_SAMPLES = args.ns
        MIN_D_POS = args.dp

        main()
    except rospy.ROSInterruptException:
        pass
