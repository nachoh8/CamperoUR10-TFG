import rospy
#import matplotlib.pyplot as plt

from campero_ur10_msgs.msg import ImageDraw

def process(data):
    print("--NEW DRAW--")
    if data.type == ImageDraw.BOARD_POINTS:
        print("W: " + str(data.W) + " H: " + str(data.H))

    traces = data.traces
    i = 1
    for trace in traces:
        xStr = "xKS3 = ["
        xx = []
        yStr = "yKS3 = ["
        yy = []
        for pt in trace.points:
            xStr += str(pt.x) + " "
            xx.append(pt.x)
            yStr += str(pt.y) + " "
            yy.append(pt.y)
        xStr += "];"
        yStr += "];"
        print("-TRACE " + str(i) + "-")
        print(xStr)
        print(yStr)
        i += 1
        #plt.plot(yy, xx)
        #plt.show()
        


rospy.init_node("plot_img_draw", anonymous=True)
rospy.Subscriber("/image_points", ImageDraw, process, queue_size=1)

rospy.loginfo("Start monitoring")

rospy.spin()

rospy.loginfo("End program")
