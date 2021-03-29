import argparse
import turtle
from rospy import ROSInterruptException

import sys
sys.path.insert(1, '/home/nacho8/ROS_workspaces/campero_ur10_ws/src/draw_board/lib')

from image import MyImageDraw, SIZE_DEFAULT
from node import DrawBoardNode


STEP_DISTANCE = 1

class BoardTurtle:
    def __init__(self, size = SIZE_DEFAULT):
        self.size = SIZE_DEFAULT if size < 0 else size

        self.node = DrawBoardNode()
        self.image = MyImageDraw(self.size, self.size/2, -self.size/2)

        self.screen = turtle.Screen()
        self.screen.setup(self.size, self.size, 0, 0)
        self.t = turtle.Turtle()
        self.t.speed(-1)
        self.t.penup()

        self.key = -1

        self.screen.onkey(self.up_down_pen, "p")
        self.screen.onkey(self.move_up, "Up")
        self.screen.onkey(self.move_down, "Down")
        self.screen.onkey(self.move_left, "Left")
        self.screen.onkey(self.move_right, "Right")
        self.screen.onkey(self.clearBoard, "c")
        self.screen.onkey(self.close, "z")
        self.screen.onkey(self.send, "s")
        self.screen.listen()

        turtle.mainloop()
    
    def up_down_pen(self):
        if self.t.isdown():
            print("Pen Up")
            self.t.penup()
        else:
            print("Pen Down")
            self.image.addTrace()
            self.t.pendown()
    
    def clearBoard(self):
        self.image.clear()
        self.t.clear()

    def move(self):
        if self.t.isdown():
            x,y = self.t.pos()
            self.image.addPoint(x, y)
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
        self.node.close()
        turtle.bye()
    
    def send(self):
        self.node.publishImage(self.image.getImgDraw())

    def getImgDraw(self):
        return self.image.getImgDraw()

    def main(self):
        k_last = self.key
        self.key = -1
        return k_last


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Draw on board')
    parser.add_argument("-s", "--size", type = int, help = 'size of screen NxN', default = SIZE_DEFAULT)

    args = parser.parse_args()
    try:
        board = BoardTurtle()
        turtle.mainloop()
    except ROSInterruptException:
        pass