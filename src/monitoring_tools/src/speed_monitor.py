#!/usr/bin/env python
import rospy
from moveit_msgs.msg import MoveGroupActionResult
import matplotlib.pyplot as plt

TOPIC_NAME="/move_group/result"
NODE_NAME="speed_monitor"


JOINT_COLORS = ["-b", "-r", "-g", "-y", "-k", "-m"]
JOINT_NAMES = ["campero_ur10_shoulder_pan_joint", "campero_ur10_shoulder_lift_joint", "campero_ur10_elbow_joint",
                "campero_ur10_wrist_1_joint", "campero_ur10_wrist_2_joint", "campero_ur10_wrist_1_joint"]
NUM_JOINTS = len(JOINT_NAMES)

def process(data):
    rospy.loginfo("----Message received----")
    
    rospy.loginfo("Staus: " + data.status.text)
    
    res = data.result
    joint_trajectory = res.planned_trajectory.joint_trajectory
    
    # process data
    """
    names = []
    for n in joint_trajectory.joint_names:
        names.insert(len(names), n)
    """

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

    # plot data
    for i in range(NUM_JOINTS):
        plt.plot(times, speed[i], JOINT_COLORS[i], label=JOINT_NAMES[i])
    
    plt.legend(loc="best")
    plt.xlabel("time(s)")
    plt.ylabel("speed(rad/s)")
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