#!/usr/bin/env python
import rospy
from moveit_msgs.msg import MoveGroupActionResult
import matplotlib.pyplot as plt

TOPIC_NAME="/move_group/result"
NODE_NAME="speed_monitor"


JOINT_COLORS = ["-b", "-r", "-g", "-y", "-k", "-m"]
JOINT_COLORS2 = ["blue", "red", "green", "yellow", "black", "magenta"]
JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_1_joint"]
NUM_JOINTS = len(JOINT_NAMES)

def printNames():
    for i in range(NUM_JOINTS):
        print(JOINT_COLORS2[i] + " -> " + JOINT_NAMES[i])

def process(data):
    rospy.loginfo("----Message received----")
    
    rospy.loginfo("Staus: " + data.status.text)
    
    res = data.result
    joint_trajectory = res.planned_trajectory.joint_trajectory
    
    # process data
    times = []
    speed = [[] for _ in JOINT_NAMES]
    speed_up = [[] for _ in JOINT_NAMES]

    for pt in joint_trajectory.points:
        t = pt.time_from_start.to_sec()
        vs = pt.velocities
        accs = pt.accelerations
        times.insert(len(times), t)
        for i in range(NUM_JOINTS):
            v = vs[i]
            a = accs[i]
            speed[i].insert(len(speed[i]), v)
            speed_up[i].insert(len(speed_up[i]), a)

    # plot speed data
    plt.subplot(2,1,1)
    for i in range(NUM_JOINTS):
        plt.plot(times, speed_up[i], JOINT_COLORS[i], label=JOINT_NAMES[i])
    plt.ylabel("speed_up(rad/s^2)")

    # plot speed_up data
    plt.subplot(2,1,2)
    for i in range(NUM_JOINTS):
        plt.plot(times, speed[i], JOINT_COLORS[i], label=JOINT_NAMES[i])
    plt.ylabel("speed(rad/s)")
    
    # show plot
    plt.legend(loc="best")
    plt.xlabel("time(s)")
    plt.show()

    rospy.loginfo("----Callback end----")

def main():
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.Subscriber(TOPIC_NAME, MoveGroupActionResult, process, queue_size=1)

    rospy.loginfo("Start monitoring")

    rospy.spin()

    print("End program")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass