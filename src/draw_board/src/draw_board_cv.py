import cv2
import argparse
import numpy as np
from rospy import ROSInterruptException

import sys
from os import path

sys.path.insert(1, path.dirname(path.dirname(__file__))+"/lib") # add path to ../lib

from image import MyImageDraw, SIZE_DEFAULT
from node import DrawBoardNode

WINDOW_NAME = "BoardCV"

#MAX_UNDO = 3

K_ESC = 27 # escape
K_CLEAR = 99 # c
K_SEND = 115 # s
K_UNDO = 122 # z


class BoardCV:
    def __init__(self, size=SIZE_DEFAULT):
        self.size = SIZE_DEFAULT if size < 0 else size

        self.node = DrawBoardNode()
        self.image = MyImageDraw(self.size)

        self.drawing = False # true if mouse is pressed
        self.pt1_x , self.pt1_y = None , None
        self.screen = np.zeros((self.size, self.size, 3), np.uint8)
        #self.last_screen = self.screen.copy()

        cv2.namedWindow(WINDOW_NAME)
        self.clearBoard()
        cv2.setMouseCallback(WINDOW_NAME, self.line_drawing)

    def clearBoard(self):
        self.screen = np.zeros((self.size, self.size, 3), np.uint8)
        self.image.clear()

    # mouse callback function
    def line_drawing(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.pt1_x, self.pt1_y = x, y
            self.image.addTrace()
            self.image.addPoint(x,y)

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing == True:
                cv2.line(self.screen,(self.pt1_x, self.pt1_y),(x,y),color=(255,255,255),thickness=1)
                self.pt1_x, self.pt1_y = x,y
                self.image.addPoint(x,y)
                
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            cv2.line(self.screen,(self.pt1_x, self.pt1_y),(x,y),color=(255,255,255),thickness=1)
    
    def getImgDraw(self):
        return self.image.getImgDraw()

    def close(self):
        cv2.destroyAllWindows()
        self.node.close()
    
    def undo(self):
        pass
    
    def main(self):
        while not self.node.isClosed():
            cv2.imshow(WINDOW_NAME, self.screen)

            key = cv2.waitKey(1)
            if key == K_ESC:
                self.close()
            elif key == K_CLEAR:
                self.clearBoard()
            elif key == K_SEND:
                self.node.publishImage(self.image.getImgDraw())
            elif key == K_UNDO:
                self.undo()
            
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Draw on board')
    parser.add_argument("-s", "--size", type = int, help = 'size of screen NxN', default = SIZE_DEFAULT)

    args = parser.parse_args()
    try:
        board = BoardCV(args.size)
        board.main()
    except ROSInterruptException:
        pass
