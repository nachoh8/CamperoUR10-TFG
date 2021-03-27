import cv2
import numpy as np 

from campero_ur10_msgs.msg import ImgPoint, ImgTrace, ImageDraw

SIZE_DEFAULT = 512
WINDOW_NAME = "BoardCV"

#MAX_UNDO = 3

K_ESC = 27 # escape
K_CLEAR = 99 # c
K_SEND = 115 # s
K_UNDO = 122 # z

class MyImageDraw:
    def __init__(self, size=SIZE_DEFAULT):
        self.size = SIZE_DEFAULT if size < 0 else size
        self.traces = []

    def clear(self):
        del self.traces[:]
    
    def addTrace(self):
        self.traces.insert(len(self.traces), [])

    def addPoint(self, x, y):
        pts = self.traces[len(self.traces) - 1]
        pts.insert(len(pts), (x,y))

    def processTrace(self, trace):
        if len(trace) == 0:
            return None
        
        _trace = ImgTrace()
        i = 0
        for pt in trace:
            x,y = pt
            if x >= 0 and x < self.size and y >= 0 and y < self.size:
                imgPoint = ImgPoint()
                imgPoint.x = x
                imgPoint.y = y
                _trace.points.insert(i, imgPoint)
                i += 1
        
        return _trace

    def getImgDraw(self):
        if len(self.traces) == 0:
            return None

        img = ImageDraw()
        i = 0
        for trace in self.traces:
            _trace = self.processTrace(trace)
            if _trace is not None:
                img.traces.insert(i, _trace)
                i += 1
        
        img.size = self.size

        return img

class BoardCV:
    def __init__(self, size=SIZE_DEFAULT):
        self.size = SIZE_DEFAULT if size < 0 else size

        self.image = MyImageDraw(self.size)

        self.drawing = False # true if mouse is pressed
        self.pt1_x , self.pt1_y = None , None
        self.screen = np.zeros((self.size, self.size, 3), np.uint8)
        # self.points = []
        self.last_screen = self.screen.copy()

        cv2.namedWindow(WINDOW_NAME)
        self.clearBoard()
        cv2.setMouseCallback(WINDOW_NAME, self.line_drawing)

    def clearBoard(self):
        self.screen = np.zeros((self.size, self.size, 3), np.uint8)
        #del self.points[:]
        self.image.clear()

    # mouse callback function
    def line_drawing(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.pt1_x, self.pt1_y = x, y
            #self.points.insert(len(self.points), (x,y))
            self.image.addTrace()
            self.image.addPoint(x,y)

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing == True:
                cv2.line(self.screen,(self.pt1_x, self.pt1_y),(x,y),color=(255,255,255),thickness=3)
                self.pt1_x, self.pt1_y = x,y
                #self.points.insert(len(self.points), (x,y))
                self.image.addPoint(x,y)
                
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            cv2.line(self.screen,(self.pt1_x, self.pt1_y),(x,y),color=(255,255,255),thickness=3)


    def getPoints(self):
        pass
    
    def getImgDraw(self):
        """
        if len(self.points) == 0:
            return None
        img = ImageDraw()
        i = 0
        for pt in self.points:
            x,y = pt
            if x >= 0 and x < self.size and y >= 0 and y < self.size:
                imgPoint = ImagePoint()
                imgPoint.x = pt[0]
                imgPoint.y = pt[1]
                img.points.insert(i, imgPoint)
                i += 1
        
        img.size = self.size

        return img
        """
        return self.image.getImgDraw()

    def close(self):
        cv2.destroyAllWindows()
    
    def undo(self):
        pass

    def main(self):
        cv2.imshow(WINDOW_NAME, self.screen)
            
        key = cv2.waitKey(1)
        if key == K_ESC:
            self.close()
        elif key == K_CLEAR:
            self.clearBoard()
        elif key == K_UNDO:
            self.undo()
        
        return key
