import turtle
from rospy import ROSInterruptException

from draw_board import Board, getBoardArgs, board_args_parser

STEP_DISTANCE = 1

class BoardTurtle(Board):
    def __init__(self, w, h, step = STEP_DISTANCE):
        self.super = super(BoardTurtle, self)
        self.super.__init__(w, h, w/2, -h/2)
        
        if step < 1:
            print("Error: Step Value less than 1")
            self.super.close()
            exit(1)
        
        self.step = step

        self.screen = turtle.Screen()
        self.screen.setup(self.width(), self.height(), 0, 0)
        self.t = turtle.Turtle()
        self.t.speed(-1)
        self.t.penup()

        self.screen.onkey(self.up_down_pen, "p")
        self.screen.onkey(self.move_up, "Up")
        self.screen.onkey(self.move_down, "Down")
        self.screen.onkey(self.move_left, "Left")
        self.screen.onkey(self.move_right, "Right")
        self.screen.onkey(self.clearBoard, "c")
        self.screen.onkey(self.close, "z")
        self.screen.onkey(self.send, "s")
        self.screen.listen()
    
    def up_down_pen(self):
        if self.t.isdown():
            print("Pen Up")
            self.t.penup()
        else:
            print("Pen Down")
            self.super.addTrace()
            self.t.pendown()
    
    def clearBoard(self):
        self.t.clear()
        self.t.penup()
        self.super.clearBoard()

    def move(self):
        if self.t.isdown():
            x,y = self.t.pos()
            self.super.addPoint(x, y)
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
        self.super.close()
        turtle.bye()
    
    def main(self):
        turtle.mainloop()

if __name__ == '__main__':
    board_args_parser.add_argument("-S", "--step", type = int, help = 'move step', default = STEP_DISTANCE)

    args = getBoardArgs()
    try:
        board = BoardTurtle(args.width, args.height, args.step)
        board.main()
    except ROSInterruptException:
        pass