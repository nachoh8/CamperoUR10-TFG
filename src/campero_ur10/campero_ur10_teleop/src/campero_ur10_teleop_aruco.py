import argparse
import rospy
import sys
import math
import tf
import geometry_msgs.msg

WORLD_FRAME='/campero_base_footprint'

AXIS_X = "x"
AXIS_Y = "y"

NUM_SAMPLES = 1000 # num muestras para medir una posicon

MARKER_ID = 0 # id de la marca a seguir

# MIN_D_TIME = 0.5 # minimo tiempo en segundos que tiene que transcurrir entre dos posiciones distintas

MIN_D_POS = 0.05 # minima distancia en metros que tiene que moverse la marca entre dos posiciones distintas

# dist = lambda p1,p2: math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
dist = lambda p1,p2: [p2[0] - p1[0], p2[1] - p1[1]]

def move_robot(last_pos, new_pos):
    global MIN_D_POS

    # if last_pos is None:
    #     return new_pos
        
    dX = last_pos[0] - new_pos[0]
    dY = new_pos[1] - last_pos[1]
    v = [0,0]

    if abs(dX) > MIN_D_POS:
        v[0] = dX

    if abs(dY) > MIN_D_POS:
        v[1] = dY

    if v[0] != 0 or v[1] != 0:
        # TODO: send operation
        rospy.loginfo("Last Pos: " + str(last_pos) + " New Pos: " + str(new_pos))
        rospy.loginfo("Move: " + str(v))
        return new_pos

    return last_pos

def main():
    rospy.init_node('campero_ur10_teleop_aruco')

    listener = tf.TransformListener()

    rate = rospy.Rate(100.0)

    if AXIS_X.lower() == "z":
        xx = 2
    elif AXIS_X.lower() == "y":
        xx = 1
    else:
        xx = 0
    
    if AXIS_Y.lower() == "z":
        yy = 2
    elif AXIS_Y.lower() == "x":
        yy = 0
    else:
        yy = 1

    info = "World frame: " + str(WORLD_FRAME) + "\n"
    info += "Aruco Marker id: " + str(MARKER_ID) + "\n"
    info += "Num. samples per position: " + str(NUM_SAMPLES) + "\n"
    info += "Min. distance between positions: " + str(MIN_D_POS) + "\n"
    info += "Original Axis " + AXIS_X + " remap to X\n"
    info += "Original Axis " + AXIS_Y + " remap to Y\n"
    rospy.loginfo(info)

    rospy.loginfo("Teleop Ready")

    last_pos = None 
    i = 0
    while not rospy.is_shutdown():

        try:
            samples_x = 0.0
            samples_y = 0.0
            for _ in range(NUM_SAMPLES):
                (trans,_) = listener.lookupTransform(WORLD_FRAME, "/aruco_marker_frame_" + str(MARKER_ID), rospy.Time(0))
                samples_x += trans[xx]
                samples_y += trans[yy]
            new_pos = [samples_x / float(NUM_SAMPLES), samples_y / float(NUM_SAMPLES)]
            if i < 5 or last_pos is None: # eliminar ruido inicial
                last_pos = new_pos
                i += 1
            else:
                last_pos = move_robot(last_pos, new_pos)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
            
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