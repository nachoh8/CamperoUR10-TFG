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

max_dist_error = 7

contour_method = 0 # {0: NOT_WATERSHED, 1: WATERSHED}
min_contour_size = 60 # px
binary_thresh = 0
conectivity_way = 0

apply_sharp = 0

canny_th1 = 100
canny_th2 = 300

blur_ksize = 3

number_iterations = 9
max_kernel_size = 21
max_erode_dilate_type = 3
erode_type = 0 # {0: NOT_APPLY, 1: MORPH_RECT, 2: MORPH_CROSS, 3: MORPH_ELLIPSE}
erode_size = 0
dilate_type = 0 # {0: NOT_APPLY, 1: MORPH_RECT, 2: MORPH_CROSS, 3: MORPH_ELLIPSE}
dilate_size = 0


def mouse_callback(event, x, y, flags, param):
    global pub, can_update, canny_th1, canny_th2, blur_ksize, contour_method, min_contour_size, apply_sharp, max_dist_error, binary_thresh, conectivity_way, number_iterations

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
            """
            rospy.set_param('/image_inpainting/canny_th1', canny_th1)
            rospy.set_param('/image_inpainting/canny_th2', canny_th2)
            """
            rospy.set_param('/image_inpainting/number_iterations', number_iterations)
            rospy.set_param('/image_inpainting/erode_type', erode_type)
            rospy.set_param('/image_inpainting/erode_size', erode_size)
            rospy.set_param('/image_inpainting/dilate_type', dilate_type)
            rospy.set_param('/image_inpainting/dilate_size', dilate_size)
            rospy.set_param('/image_inpainting/contour_method', contour_method)
            rospy.set_param('/image_inpainting/min_contour_size', min_contour_size)
            rospy.set_param('/image_inpainting/apply_sharp', bool(apply_sharp))
            rospy.set_param('/image_inpainting/max_dist_error', max_dist_error)
            rospy.set_param('/image_inpainting/binary_thresh', binary_thresh)
            rospy.set_param('/image_inpainting/conectivity_way', (4 if conectivity_way == 0 else 8))
            rospy.set_param('/image_inpainting/blur_ksize', blur_ksize)
            pub.publish("update_image")
        
def trackbar_callback(idx, v):
    global canny_th1, canny_th2, blur_ksize, erode_type, erode_size, dilate_type, dilate_size, contour_method, min_contour_size, apply_sharp, max_dist_error, binary_thresh, conectivity_way, number_iterations
    if idx == 0:
        canny_th1 = v
    elif idx == 1:
        canny_th2 = v
    elif idx == 2:
        blur_ksize = v
    elif idx == 3:
        erode_type = v
    elif idx == 4:
        erode_size = v
    elif idx == 5:
        dilate_type = v
    elif idx == 6:
        dilate_size = v
    elif idx == 7:
        contour_method = v
    elif idx == 8:
        min_contour_size = v
    elif idx == 9:
        apply_sharp = v
    elif idx == 10:
        max_dist_error = v
    elif idx == 11:
        binary_thresh = v
    elif idx == 12:
        conectivity_way = v
    elif idx == 13:
        number_iterations = v
        

img_settings = np.zeros((H, W, 3), np.uint8)

# Draw Buttons
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

# Init Ros
rospy.init_node("image_inpainting_gui", anonymous=True)
pub = rospy.Publisher("/image_inpainting/cmd", String, queue_size=1)

# Load params
contour_method = rospy.get_param('/image_inpainting/contour_method', 0)
min_contour_size = rospy.get_param('/image_inpainting/min_contour_size', 60)
apply_sharp = int(rospy.get_param('/image_inpainting/apply_sharp', False))
max_dist_error = rospy.get_param('/image_inpainting/max_dist_error', 7)
erode_type = int(rospy.get_param('/image_inpainting/erode_type', 1))
erode_size = rospy.get_param('/image_inpainting/erode_size', 3)
dilate_type = int(rospy.get_param('/image_inpainting/dilate_type', 1))
dilate_size = rospy.get_param('/image_inpainting/dilate_size', 3)
binary_thresh = rospy.get_param('/image_inpainting/binary_thresh', 0)
conectivity_way = rospy.get_param('/image_inpainting/conectivity_way', 0)
blur_ksize = rospy.get_param('/image_inpainting/blur_ksize', 3)
number_iterations = rospy.get_param('/image_inpainting/number_iterations', 9)

"""
canny_th1 = rospy.get_param('/image_inpainting/canny_th1', 100)
canny_th2 = rospy.get_param('/image_inpainting/canny_th2', 300)
"""

# Draw Trackbars
cv.namedWindow(WINDOW_NAME)

cv.createTrackbar('max_dist_error', WINDOW_NAME, max_dist_error, 50, lambda v: trackbar_callback(10,v))
cv.createTrackbar('contour_method', WINDOW_NAME, contour_method, 1, lambda v: trackbar_callback(7,v))
cv.createTrackbar('min_contour_size', WINDOW_NAME, min_contour_size, 100, lambda v: trackbar_callback(8,v))
cv.createTrackbar('binary_thresh', WINDOW_NAME, binary_thresh, 255, lambda v: trackbar_callback(11,v))
cv.createTrackbar('conectivity_way(0: 4-way, 1: 8-way)', WINDOW_NAME, conectivity_way, 1, lambda v: trackbar_callback(12,v))
cv.createTrackbar('number_iterations', WINDOW_NAME, number_iterations, 100, lambda v: trackbar_callback(13,v))
cv.createTrackbar('Erode type(MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE)', WINDOW_NAME, erode_type, max_erode_dilate_type, lambda v: trackbar_callback(3,v))
cv.createTrackbar('Erode', WINDOW_NAME, erode_size, max_kernel_size, lambda v: trackbar_callback(4,v))
cv.createTrackbar('Dilate type(MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE)', WINDOW_NAME, dilate_type, max_erode_dilate_type, lambda v: trackbar_callback(5,v))
cv.createTrackbar('Dilate', WINDOW_NAME, dilate_size, max_kernel_size, lambda v: trackbar_callback(6,v))
cv.createTrackbar('apply_sharp', WINDOW_NAME, apply_sharp, 1, lambda v: trackbar_callback(9,v))
cv.createTrackbar('Blur Kernel Size', WINDOW_NAME, blur_ksize, 5, lambda v: trackbar_callback(2,v))

"""
cv.createTrackbar('Canny Threshold 1', WINDOW_NAME, canny_th1, 1000, lambda v: trackbar_callback(0,v))
cv.createTrackbar('Canny Threshold 2', WINDOW_NAME, canny_th2, 1000, lambda v: trackbar_callback(1,v))
"""

cv.setMouseCallback(WINDOW_NAME, mouse_callback)

while True:
    cv.imshow(WINDOW_NAME, img_settings)
    key = cv.waitKey(5)
    if key == 27:
        break

cv.destroyAllWindows()

pub.unregister()
rospy.signal_shutdown("Gui Closed")