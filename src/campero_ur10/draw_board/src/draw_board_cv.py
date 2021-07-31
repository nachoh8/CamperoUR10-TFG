import cv2
import numpy as np
from rospy import ROSInterruptException

from draw_board import Board, getBoardArgs

WINDOW_NAME = "BoardCV"

K_ESC = 27 # escape
K_CLEAR = 99 # c
K_SEND = 115 # s

class BoardCV(Board):
    def __init__(self, w, h):
        self.super = super(BoardCV, self)
        self.super.__init__(w, h)

        self.drawing = False # true if mouse is pressed
        self.pt1_x , self.pt1_y = None , None
        
        self.screen = np.zeros((self.height(), self.width(), 3), np.uint8)

        cv2.namedWindow(WINDOW_NAME)
        self.clearBoard()
        cv2.setMouseCallback(WINDOW_NAME, self.line_drawing)

    def clearBoard(self):
        self.screen = np.zeros((self.height(), self.width(), 3), np.uint8)
        self.super.clearBoard()

    # mouse callback function
    def line_drawing(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.super.addTrace()
            self.addPoint(x,y)

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing == True:
                cv2.line(self.screen,(self.pt1_x, self.pt1_y),(x,y),color=(255,255,255),thickness=1)
                self.addPoint(x,y)
                
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            cv2.line(self.screen,(self.pt1_x, self.pt1_y),(x,y),color=(255,255,255),thickness=1)
    
    def addPoint(self, x, y):
        self.pt1_x, self.pt1_y = x, y
        self.super.addPoint(x, y)
    
    def close(self):
        cv2.destroyAllWindows()
        self.super.close()

    def main(self):
        while not self.node.isClosed():
            cv2.imshow(WINDOW_NAME, self.screen)

            key = cv2.waitKey(1)
            if key == K_ESC:
                self.close()
            elif key == K_CLEAR:
                self.clearBoard()
            elif key == K_SEND:
                self.send()        
            
if __name__ == '__main__':
    args = getBoardArgs()

    try:
        board = BoardCV(args.width, args.height)
        board.main()
    except ROSInterruptException:
        pass
