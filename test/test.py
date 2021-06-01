from __future__ import print_function
import cv2 as cv
import argparse
max_lowThreshold = 100
window_name = 'Edge Map'
title_trackbar = 'Min Threshold:'
ratio = 3
kernel_size = 3

def CannyThreshold(val):
    low_threshold = val

    detected_edges = cv.Canny(src_gray, low_threshold, low_threshold*ratio, kernel_size)
    res = cv.resize(detected_edges, (877, 621), interpolation = cv.INTER_AREA)
    """
    _,contours,_ = cv.findContours(image=detected_edges, mode=cv.RETR_TREE, method=cv.CHAIN_APPROX_NONE)

    # draw contours on the original image
    cv.drawContours(image=src, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv.LINE_AA)
    
    cnts = cv.findContours(detected_edges.copy(), cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[0]
    
    cv.drawContours(src, cnts, -1, 0, -1)
    """
    cv.imshow(window_name, res)

parser = argparse.ArgumentParser(description='Code for Canny Edge Detector tutorial.')
parser.add_argument('--input', help='Path to input image.', default='fruits.jpg')
args = parser.parse_args()

src = cv.imread(args.input)

if src is None:
    print('Could not open or find the image: ', args.input)
    exit(0)

src_gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)

cv.namedWindow(window_name)

cv.createTrackbar(title_trackbar, window_name , 0, max_lowThreshold, CannyThreshold)
CannyThreshold(0)


cv.waitKey()
cv.destroyAllWindows()