#!/usr/bin/env python

# Publica la velocidad de los joints en el topic /joint_speed
# La velocidad se calcula a partir de la posicion del instante anterior y actual que se publica en /joint_states

import sys

import rospy
from sensor_msgs.msg import JointState

import matplotlib.pyplot as plt

from campero_ur10_msgs.msg import JointSpeed

class JointPosition:
    def __init__(self, time, positions, v_is_zero):
        self.time = time
        self.positions = positions
        self.v_is_zero = v_is_zero

class PlotGraphic:
    def __init__(self):
        self._init_vars()
    
    def _init_vars(self):
        self.joint_speed = [[] for _ in range(NUM_JOINTS)]
        self.time = []

    def add(self, dt, v):
        if len(self.time) == 0:
            tt = 0.0
        else:
            tt = self.time[len(self.time)-1] + dt

        self.time.insert(len(self.time), tt)

        for i in range(NUM_JOINTS):
            v_j = v[i]
            self.joint_speed[i].insert(len(self.joint_speed[i]), v_j)
    
    def plot(self):
        for i in range(NUM_JOINTS):
            plt.plot(self.time, self.joint_speed[i], JOINT_COLORS[i], label=JOINT_NAMES[i])
        
        plt.ylabel("speed(rad/s)")
        plt.xlabel("time(s)")
        plt.legend(loc="best")
        plt.show()
        
        del self.time[:]
        del self.joint_speed[:]
        self._init_vars()

JOINT_COLORS = ["-b", "-r", "-g", "-y", "-k", "-m"]
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow",
                "wrist_1", "wrist_2", "wrist_3"]

TOPIC_NAME="/joint_states"
NODE_NAME="joint_speed_monitor"
TOPIC_PUB="joint_speed"

NUM_JOINTS = 6

last_pos = None # last joint position
pub = None # /joint_speed publisher

pg = None # if -plt -> plot speed

def process(data):
    global last_pos, pub, pg

    t = data.header.stamp.to_sec()
    pos = data.position[NUM_JOINTS::] # descartamos posiciones de joints no validos
    
    v = [0.0 for _ in range(NUM_JOINTS)]

    is_zero = True
    dt = 0.0
    if last_pos is not None:
        dt = t - last_pos.time
        
        for i in range(NUM_JOINTS):
            dv = pos[i] - last_pos.positions[i]
            v[i] = dv / dt
            if v[i] != 0.0:
                is_zero = False
        
    if last_pos is None or not (last_pos.v_is_zero and is_zero): # se evita que se publique siempre velocidad 0 cuando esta parado
            rospy.loginfo("[" + str(t) + "s]Speed: " + str(v))
            pub.publish(v)
            if pg is not None:
                pg.add(dt, v)
                if last_pos is not None and is_zero: # end of trayectory
                    pg.plot()

    last_pos = JointPosition(t, pos, is_zero)

def main():
    global pub
    rospy.init_node(NODE_NAME, anonymous=True)
    pub = rospy.Publisher(TOPIC_PUB, JointSpeed, queue_size=1)
    rospy.Subscriber(TOPIC_NAME, JointState, process, queue_size=1)

    rospy.loginfo("Start monitoring")

    rospy.spin()

    rospy.loginfo("End program")


if __name__ == '__main__':
    if len(sys.argv) == 2:
        if sys.argv[1] == "-plt":
            pg = PlotGraphic()
    try:
        main()
    except rospy.ROSInterruptException:
        pass