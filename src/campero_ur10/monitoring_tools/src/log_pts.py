#!/usr/bin/env python  
import rospy
import sys
import math
import tf
import geometry_msgs.msg

from campero_ur10_msgs.msg import ImgPoint, ImgTrace, ImageDraw

def process(data):
    W = data.W
    H = data.H
    trazos = data.traces
    f = open("/home/nacho8/ROS_workspaces/campero_ur10_ws/test/plot_log.txt", "w")
    
    print("Start")
    xx = "["
    yy = "["
    for t in trazos:
        l = len(t.points)
        print(l)
        for i in range(l):
            pt = t.points[i]
            xx += str(pt.y)
            yy += str(pt.z)
            if i + 1 < l:
                xx += " "
                yy += " "
    res = xx + "]\n" + yy + "]"
    f.write(res)
    f.close()

    print("End")

            

rospy.init_node('log_pts', anonymous=True)
rospy.Subscriber("/image_points", ImageDraw, process, queue_size=1)

rospy.loginfo("Start monitoring")

rospy.spin()

rospy.loginfo("End program")