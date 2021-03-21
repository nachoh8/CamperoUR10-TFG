import cv2
import numpy as np 

SIZE_DEFAULT = 512
WINDOW_NAME = "Board"

#MAX_UNDO = 3

K_ESC = 27 # escape
K_CLEAR = 99 # c
K_SEND = 115 # s
#K_UNDO = 122

drawing = False # true if mouse is pressed
pt1_x , pt1_y = None , None
dim = SIZE_DEFAULT
screen = np.zeros((dim, dim, 3), np.uint8)

points = []

def clearBoard():
    global screen, dim, points
    screen = np.zeros((dim, dim, 3), np.uint8)
    del points[:]

# mouse callback function
def line_drawing(event, x, y, flags, param):
    global pt1_x, pt1_y, drawing, points

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        pt1_x, pt1_y = x, y
        points.insert(len(points), (x,y))

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            cv2.line(screen,(pt1_x,pt1_y),(x,y),color=(255,255,255),thickness=3)
            pt1_x,pt1_y = x,y
            points.insert(len(points), (x,y))
            
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.line(screen,(pt1_x,pt1_y),(x,y),color=(255,255,255),thickness=3)

def setupBoard(size):
    global dim
    dim = SIZE_DEFAULT if size < 0 else size

    cv2.namedWindow(WINDOW_NAME)
    clearBoard()
    cv2.setMouseCallback(WINDOW_NAME, line_drawing)

def getPoints():
    global points
    return points

def close():
    cv2.destroyAllWindows()

def main():
    cv2.imshow(WINDOW_NAME, screen)
        
    key = cv2.waitKey(1)
    if key == K_ESC:
        close()
    elif key == K_CLEAR:
        clearBoard()
    """
    elif key == K_UNDO:
        undo()
    """
    
    return key
    
    

