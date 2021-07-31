import argparse

from image import MyImageDraw
from node import DrawBoardNode

SIZE_DEFAULT = 512
W_DEFAULT = SIZE_DEFAULT
H_DEFAULT = SIZE_DEFAULT
XOFFSET_DEFAULT = 0
YOFFSET_DEFAULT = 0

class Board(object):
    def __init__(self, w = W_DEFAULT, h = H_DEFAULT, xoffset = XOFFSET_DEFAULT, yoffset = YOFFSET_DEFAULT):
        self.node = DrawBoardNode()
        self.image = MyImageDraw(w, h, xoffset, yoffset)
        self.okey()
    
    def okey(self):
        w,h = self.size()
        if w < 0 or h < 0:
            print("Error: Negative Image Size " + str(w) + "x" + str(h))
            self.close()
            exit(1)
    
    def clearBoard(self):
        self.image.clear()
    
    def addTrace(self):
        self.image.addTrace()

    def addPoint(self, x, y):
        self.image.addPoint(x,y)

    def size(self):
        return [self.width(), self.height()]
    
    def width(self):
        return self.image.W
    
    def height(self):
        return self.image.H

    def send(self):
        self.node.publishImage(self.image.getImgDraw())
    
    def close(self):
        self.node.close()
    
    def main(self):
        print("Main Loop")


# Args
board_args_parser = argparse.ArgumentParser(description='Draw on board')
board_args_parser.add_argument("-W", "--width", type = int, help = 'width of screen', default = W_DEFAULT)
board_args_parser.add_argument("-H", "--height", type = int, help = 'height of screen', default = H_DEFAULT)

def getBoardArgs():
    return board_args_parser.parse_args()
