import rospy
from rospy.core import is_shutdown

from std_msgs.msg import String

import numpy as np
import cv2 as cv

### Constants

WINDOW_NAME = "Image Inpainting GUI"

W = 320
H = 240

H_DIV = H/4
BUTTON_SEND = [(0,0), (W, H_DIV)]
BUTTON_RESET = [(0, H_DIV), (W, H_DIV * 2)]
BUTTON_UPDATE = [(-W/2, H_DIV * 2), (W, H_DIV * 3)]
BUTTON_RECALCULATE = [(-W/2, H_DIV * 3), (W, H_DIV * 4)]

### Params
pub = None # ros publisher

can_update = True

# Calculate transform params
max_dist_error = 7

# common proc params
contour_method = 0 # {0: SIMPLE (NOT_WATERSHED), 1: WATERSHED}
min_contour_size = 60 # px
blur_ksize = 3
conectivity_way = 0

# concaveman params
apply_concaveman = 1 # 0-1
concaveman_alpha = 100

# smoothPath params
apply_smooth_path = 1 # 0-1
smooth_path_kernel = 11

# not_watershed proc method params
apply_sharp = 0 # 0-1

# watershed proc method params
MAX_KERNEL_SIZE_ERODE_DILATE = 21
number_iterations = 9
max_erode_dilate_type = 3
erode_type = 0 # {0: NOT_APPLY, 1: MORPH_RECT, 2: MORPH_CROSS, 3: MORPH_ELLIPSE}
erode_size = 0
dilate_type = 0 # {0: NOT_APPLY, 1: MORPH_RECT, 2: MORPH_CROSS, 3: MORPH_ELLIPSE}
dilate_size = 0

### Functions
def mouse_callback(event, x, y, flags, param):
    global pub, can_update, blur_ksize, contour_method, min_contour_size, apply_sharp, max_dist_error, conectivity_way, number_iterations, apply_concaveman, concaveman_alpha, apply_smooth_path, smooth_path_kernel
    if pub is None:
        rospy.logwarn("No hay publisher")
        return

    if event == cv.EVENT_LBUTTONUP:
        if x >= BUTTON_SEND[0][0] and x <= BUTTON_SEND[1][0] and y >= BUTTON_SEND[0][1] and y <= BUTTON_SEND[1][1]:
            print("CMD -> Send image to robot")
            pub.publish("send")
        elif x >= BUTTON_RESET[0][0] and x <= BUTTON_RESET[1][0] and y >= BUTTON_RESET[0][1] and y <= BUTTON_RESET[1][1]:
            print("CMD -> RESET image and contours")
            pub.publish("reset")
        elif x >= BUTTON_UPDATE[0][0] and x <= BUTTON_UPDATE[1][0] and y >= BUTTON_UPDATE[0][1] and y <= BUTTON_UPDATE[1][1]:
            can_update = not can_update
            print("CMD -> UPDATE " + ("ON" if can_update else "OFF"))
            pub.publish("update_" + ("on" if can_update else "off"))
        elif x >= BUTTON_RECALCULATE[0][0] and x <= BUTTON_RECALCULATE[1][0] and y >= BUTTON_RECALCULATE[0][1] and y <= BUTTON_RECALCULATE[1][1]:
            print("CMD -> UPDATE PARAMS and IMAGE")
            rospy.set_param('/image_inpainting/number_iterations', number_iterations)
            rospy.set_param('/image_inpainting/erode_type', erode_type)
            rospy.set_param('/image_inpainting/erode_size', erode_size)
            rospy.set_param('/image_inpainting/dilate_type', dilate_type)
            rospy.set_param('/image_inpainting/dilate_size', dilate_size)
            rospy.set_param('/image_inpainting/contour_method', contour_method)
            rospy.set_param('/image_inpainting/min_contour_size', min_contour_size)
            rospy.set_param('/image_inpainting/apply_concaveman', bool(apply_concaveman))
            rospy.set_param('/image_inpainting/concaveman_alpha', (float(concaveman_alpha)/100.0))
            rospy.set_param('/image_inpainting/apply_sharp', bool(apply_sharp))
            rospy.set_param('/image_inpainting/max_dist_error', max_dist_error)
            rospy.set_param('/image_inpainting/conectivity_way', (4 if conectivity_way == 0 else 8))
            rospy.set_param('/image_inpainting/blur_ksize', blur_ksize)
            rospy.set_param('/image_inpainting/apply_smooth_path', bool(apply_smooth_path))
            rospy.set_param('/image_inpainting/smooth_path_kernel', smooth_path_kernel)
            
            pub.publish("update_image")
        
def trackbar_callback(idx, v):
    global blur_ksize, erode_type, erode_size, dilate_type, dilate_size, contour_method, min_contour_size, apply_sharp, max_dist_error, conectivity_way, number_iterations, apply_concaveman, concaveman_alpha, apply_smooth_path, smooth_path_kernel
    if idx == 2:
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
    elif idx == 12:
        conectivity_way = v
    elif idx == 13:
        number_iterations = v
    elif idx == 14:
        apply_concaveman = v
    elif idx == 15:
        concaveman_alpha = v
    elif idx == 16:
        apply_smooth_path = v
    elif idx == 17:
        smooth_path_kernel = v 

### Init Image
img_settings = np.zeros((H, W, 3), np.uint8)

### Draw Buttons
img_settings = cv.rectangle(img_settings, BUTTON_SEND[0], BUTTON_SEND[1], (0,255,0), -1)
text_pos = ((BUTTON_SEND[0][0] + BUTTON_SEND[1][0]) // 2, (BUTTON_SEND[0][1] + BUTTON_SEND[1][1]) // 2)
img_settings = cv.putText(img_settings, 'Send', text_pos, cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2, cv.LINE_AA)

img_settings = cv.rectangle(img_settings, BUTTON_RESET[0], BUTTON_RESET[1], (0,0,255), -1)
text_pos = ((BUTTON_RESET[0][0] + BUTTON_RESET[1][0]) // 2, (BUTTON_RESET[0][1] + BUTTON_RESET[1][1]) // 2)
img_settings = cv.putText(img_settings, 'Reset', text_pos, cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2, cv.LINE_AA)

img_settings = cv.rectangle(img_settings, BUTTON_UPDATE[0], BUTTON_UPDATE[1], (128,128,128), -1)
text_pos = ((BUTTON_UPDATE[0][0] + BUTTON_UPDATE[1][0]) // 2, (BUTTON_UPDATE[0][1] + BUTTON_UPDATE[1][1]) // 2)
img_settings = cv.putText(img_settings, 'Update on/off', text_pos, cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv.LINE_AA)

img_settings = cv.rectangle(img_settings, BUTTON_RECALCULATE[0], BUTTON_RECALCULATE[1], (255,0,0), -1)
text_pos = ((BUTTON_RECALCULATE[0][0] + BUTTON_RECALCULATE[1][0]) // 2, (BUTTON_RECALCULATE[0][1] + BUTTON_RECALCULATE[1][1]) // 2)
img_settings = cv.putText(img_settings, 'Update Image', text_pos, cv.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv.LINE_AA)

### Init Ros
rospy.init_node("image_inpainting_gui", anonymous=True)
pub = rospy.Publisher("/image_inpainting/cmd", String, queue_size=1)

### Load params
max_dist_error = rospy.get_param('/image_inpainting/max_dist_error', 7)

contour_method = rospy.get_param('/image_inpainting/contour_method', 0)
min_contour_size = rospy.get_param('/image_inpainting/min_contour_size', 60)
conectivity_way = rospy.get_param('/image_inpainting/conectivity_way', 0)
blur_ksize = rospy.get_param('/image_inpainting/blur_ksize', 3)

apply_concaveman = int(rospy.get_param('/image_inpainting/apply_concaveman', True))
concaveman_alpha = int(rospy.get_param('/image_inpainting/concaveman_alpha', 1.0)*100)

apply_sharp = int(rospy.get_param('/image_inpainting/apply_sharp', False))

erode_type = int(rospy.get_param('/image_inpainting/erode_type', 1))
erode_size = rospy.get_param('/image_inpainting/erode_size', 3)
dilate_type = int(rospy.get_param('/image_inpainting/dilate_type', 1))
dilate_size = rospy.get_param('/image_inpainting/dilate_size', 3)
number_iterations = rospy.get_param('/image_inpainting/number_iterations', 9)

apply_smooth_path = int(rospy.get_param('/image_inpainting/apply_smooth_path', True))
smooth_path_kernel = rospy.get_param('/image_inpainting/smooth_path_kernel', 11)

### Prepare Window & Draw Trackbars
cv.namedWindow(WINDOW_NAME)

cv.createTrackbar('max_dist_error', WINDOW_NAME, max_dist_error, 50, lambda v: trackbar_callback(10,v))

cv.createTrackbar('contour_method', WINDOW_NAME, contour_method, 1, lambda v: trackbar_callback(7,v))
cv.createTrackbar('min_contour_size', WINDOW_NAME, min_contour_size, 1000, lambda v: trackbar_callback(8,v))
cv.createTrackbar('conectivity_way(0: 4-way, 1: 8-way)', WINDOW_NAME, conectivity_way, 1, lambda v: trackbar_callback(12,v))
cv.createTrackbar('blur_kernel_size', WINDOW_NAME, blur_ksize, 5, lambda v: trackbar_callback(2,v))

cv.createTrackbar('apply_concaveman', WINDOW_NAME, apply_concaveman, 1, lambda v: trackbar_callback(14,v))
cv.createTrackbar('concaveman_alpha(v/100)', WINDOW_NAME, concaveman_alpha, 500, lambda v: trackbar_callback(15,v))

cv.createTrackbar('apply_smooth_path', WINDOW_NAME, apply_smooth_path, 1, lambda v: trackbar_callback(16,v))
cv.createTrackbar('smooth_path_kernel(impares)', WINDOW_NAME, smooth_path_kernel, 11, lambda v: trackbar_callback(17,v))

cv.createTrackbar('number_iterations', WINDOW_NAME, number_iterations, 100, lambda v: trackbar_callback(13,v))
cv.createTrackbar('erode _type(NOT_APPLY,MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE)', WINDOW_NAME, erode_type, max_erode_dilate_type, lambda v: trackbar_callback(3,v))
cv.createTrackbar('erode_size', WINDOW_NAME, erode_size, MAX_KERNEL_SIZE_ERODE_DILATE, lambda v: trackbar_callback(4,v))
cv.createTrackbar('dilate_type(NOT_APPLY,MORPH_RECT,MORPH_CROSS,MORPH_ELLIPSE)', WINDOW_NAME, dilate_type, max_erode_dilate_type, lambda v: trackbar_callback(5,v))
cv.createTrackbar('dilate_size', WINDOW_NAME, dilate_size, MAX_KERNEL_SIZE_ERODE_DILATE, lambda v: trackbar_callback(6,v))

cv.createTrackbar('apply_sharp', WINDOW_NAME, apply_sharp, 1, lambda v: trackbar_callback(9,v))

cv.setMouseCallback(WINDOW_NAME, mouse_callback)

### Main
while True and not rospy.is_shutdown():
    cv.imshow(WINDOW_NAME, img_settings)
    key = cv.waitKey(5)
    if key == 27:
        break

### Finish Program
cv.destroyAllWindows()

if not rospy.is_shutdown():
    pub.unregister()
    rospy.signal_shutdown("Gui Closed")
