import argparse
import turtle

from campero_ur10_msgs.msg import ImagePoint, ImageDraw

#import tkSimpleDialog

"""
Board Config
"""

SIZE_DEFAULT = 512

STEP_DISTANCE = 1

K_ESC = 27 # escape
K_SEND = 115 # s

class BoardTurtle:
    def __init__(self, size = SIZE_DEFAULT):
        self.size = SIZE_DEFAULT if size < 0 else size

        self.screen = turtle.Screen()
        self.screen.setup(self.size, self.size, 0, 0)
        self.t = turtle.Turtle()
        self.t.speed(-1)

        self.points = []

        self.key = -1

        self.screen.onkey(self.up_down_pen, "p")
        self.screen.onkey(self.move_up, "Up")
        self.screen.onkey(self.move_down, "Down")
        self.screen.onkey(self.move_left, "Left")
        self.screen.onkey(self.move_right, "Right")
        self.screen.onkey(self.clearBoard, "c")
        self.screen.onkey(self.exit, "z")
        self.screen.onkey(self.send, "s")
        self.screen.listen()

        turtle.mainloop()
    
    def up_down_pen(self):
        if self.t.isdown():
            print("Pen Up")
            self.t.penup()
        else:
            print("Pen Down")
            self.t.pendown()
    
    def clearBoard(self):
        del self.points[:]
        self.t.clear()

    def move(self):
        if self.t.isdown():
            self.points.insert(len(self.points), self.t.pos())
        self.t.forward(STEP_DISTANCE)

    def move_up(self):
        self.t.setheading(90)
        self.move()
    
    def move_down(self):
        self.t.setheading(270)
        self.move()
    
    def move_left(self):
        self.t.setheading(180)
        self.move()
    
    def move_right(self):
        self.t.setheading(0)
        self.move()

    def close(self):
        turtle.bye()

    def exit(self):
        self.key = K_ESC
    
    def send(self):
        self.key = K_SEND

    def getImgDraw(self):
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

    def main(self):
        k_last = self.key
        self.key = -1
        return k_last


#board = BoardTurtle()
#turtle.mainloop()
