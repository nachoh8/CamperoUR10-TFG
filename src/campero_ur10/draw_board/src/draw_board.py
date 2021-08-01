import argparse

from draw import Draw
from node import DrawNode

SIZE_DEFAULT = 512
W_DEFAULT = SIZE_DEFAULT
H_DEFAULT = SIZE_DEFAULT

class Board(object):
    def __init__(self, w = W_DEFAULT, h = H_DEFAULT, suffix = ""):
        self.node = DrawNode(suffix)
        self.image = Draw(w, h)
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
    
    def isOkROS(self):
        return not self.node.isClosed()

    def close(self):
        self.clearBoard()
        self.node.close()


# Args
board_args_parser = argparse.ArgumentParser(description='Draw on board')
board_args_parser.add_argument("-W", "--width", type = int, help = 'width of screen', default = W_DEFAULT)
board_args_parser.add_argument("-H", "--height", type = int, help = 'height of screen', default = H_DEFAULT)

def getBoardArgs():
    return board_args_parser.parse_args()
