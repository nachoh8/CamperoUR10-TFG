import rospy
#import matplotlib.pyplot as plt

from campero_ur10_msgs.msg import ImgPoint, ImgTrace, ImageDraw

def process(data):
    print("--NEW DRAW--")
    if data.type == ImageDraw.BOARD_POINTS:
        print("W: " + str(data.W) + " H: " + str(data.H))

    traces = data.traces
    i = 1
    for trace in traces:
        xx = []
        yy = []
        for pt in trace.points:
            xx.append(pt.x)
            yy.append(pt.y)
        print("-TRACE " + str(i) + "-")
        print("xx = " + str(xx))
        print("yy = " + str(yy))
        i += 1
        #plt.plot(yy, xx)
        #plt.show()
        


rospy.init_node("plot_img_draw", anonymous=True)
rospy.Subscriber("/image_points", ImageDraw, process, queue_size=1)

rospy.loginfo("Start monitoring")

rospy.spin()

rospy.loginfo("End program")
