import rospy

from std_msgs.msg import String

import numpy as np
import cv2 as cv

WINDOW_NAME = "Image Inpainting GUI"

W = 640
H = 480

H_DIV = H/4
BUTTON_SEND = [(0,0), (W, H_DIV)]
BUTTON_RESET = [(0, H_DIV), (W, H_DIV * 2)]
BUTTON_UPDATE = [(0, H_DIV * 2), (W, H_DIV * 3)]
BUTTON_RECALCULATE = [(0, H_DIV * 3), (W, H_DIV * 4)]

pub = None

can_update = True

canny_th1 = 100
canny_th2 = 300
blur_ksize = 3

def mouse_callback(event, x, y, flags, param):
    global pub, can_update, canny_th1, canny_th2, blur_ksize

    if pub is None:
        rospy.logwarn("No hay publisher")
        return

    if event == cv.EVENT_LBUTTONUP:
        if x >= BUTTON_SEND[0][0] and x <= BUTTON_SEND[1][0] and y >= BUTTON_SEND[0][1] and y <= BUTTON_SEND[1][1]:
            print("Send image to robot")
            pub.publish("send")
        elif x >= BUTTON_RESET[0][0] and x <= BUTTON_RESET[1][0] and y >= BUTTON_RESET[0][1] and y <= BUTTON_RESET[1][1]:
            print("BUTTON_RESET")
            pub.publish("reset")
        elif x >= BUTTON_UPDATE[0][0] and x <= BUTTON_UPDATE[1][0] and y >= BUTTON_UPDATE[0][1] and y <= BUTTON_UPDATE[1][1]:
            print("BUTTON_UPDATE")
            can_update = not can_update
            pub.publish("update_" + ("on" if can_update else "off"))
        elif x >= BUTTON_RECALCULATE[0][0] and x <= BUTTON_RECALCULATE[1][0] and y >= BUTTON_RECALCULATE[0][1] and y <= BUTTON_RECALCULATE[1][1]:
            print("BUTTON_RECALCULATE")
            rospy.set_param('/image_inpainting/canny_th1', canny_th1)
            rospy.set_param('/image_inpainting/canny_th2', canny_th2)
            rospy.set_param('/image_inpainting/blur_ksize', blur_ksize)
            pub.publish("update_image")

        

def trackbar_callback(idx, v):
    global canny_th1, canny_th2, blur_ksize
    if idx == 0:
        canny_th1 = v
    elif idx == 1:
        canny_th2 = v
    elif idx == 2:
        blur_ksize = v

img_settings = np.zeros((H, W, 3), np.uint8)

img_settings = cv.rectangle(img_settings, BUTTON_SEND[0], BUTTON_SEND[1], (0,255,0), -1)
text_pos = ((BUTTON_SEND[0][0] + BUTTON_SEND[1][0]) // 2, (BUTTON_SEND[0][1] + BUTTON_SEND[1][1]) // 2)
img_settings = cv.putText(img_settings, 'Send', text_pos, cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2, cv.LINE_AA)

img_settings = cv.rectangle(img_settings, BUTTON_RESET[0], BUTTON_RESET[1], (0,0,255), -1)
text_pos = ((BUTTON_RESET[0][0] + BUTTON_RESET[1][0]) // 2, (BUTTON_RESET[0][1] + BUTTON_RESET[1][1]) // 2)
img_settings = cv.putText(img_settings, 'Reset', text_pos, cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2, cv.LINE_AA)

img_settings = cv.rectangle(img_settings, BUTTON_UPDATE[0], BUTTON_UPDATE[1], (128,128,128), -1)
text_pos = ((BUTTON_UPDATE[0][0] + BUTTON_UPDATE[1][0]) // 2, (BUTTON_UPDATE[0][1] + BUTTON_UPDATE[1][1]) // 2)
img_settings = cv.putText(img_settings, 'Update On', text_pos, cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2, cv.LINE_AA)

img_settings = cv.rectangle(img_settings, BUTTON_RECALCULATE[0], BUTTON_RECALCULATE[1], (255,0,0), -1)
text_pos = ((BUTTON_RECALCULATE[0][0] + BUTTON_RECALCULATE[1][0]) // 2, (BUTTON_RECALCULATE[0][1] + BUTTON_RECALCULATE[1][1]) // 2)
img_settings = cv.putText(img_settings, 'Process', text_pos, cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2, cv.LINE_AA)

rospy.init_node("image_inpainting_gui", anonymous=True)
pub = rospy.Publisher("/image_inpainting/cmd", String, queue_size=1)

canny_th1 = rospy.get_param('/image_inpainting/canny_th1', 100)
canny_th2 = rospy.get_param('/image_inpainting/canny_th2', 300)
blur_ksize = rospy.get_param('/image_inpainting/blur_ksize', 3)

cv.namedWindow(WINDOW_NAME)

cv.createTrackbar('Canny Threshold 1', WINDOW_NAME, canny_th1, 1000, lambda v: trackbar_callback(0,v))
cv.createTrackbar('Canny Threshold 2', WINDOW_NAME, canny_th2, 1000, lambda v: trackbar_callback(1,v))
cv.createTrackbar('Blur Kernel Size', WINDOW_NAME, blur_ksize, 5, lambda v: trackbar_callback(2,v))

cv.setMouseCallback(WINDOW_NAME, mouse_callback)

while True:
    cv.imshow(WINDOW_NAME, img_settings)
    key = cv.waitKey(5)
    if key == 27:
        break

cv.destroyAllWindows()

pub.unregister()
rospy.signal_shutdown("Gui Closed")