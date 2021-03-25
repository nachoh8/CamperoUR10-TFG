import argparse
import turtle

"""
Board Config
"""

SIZE_DEFAULT = 500

screen = turtle.Screen()
t = turtle.Turtle("turtle")
t.speed(-1)
is_pen_down = False
count = 0

def dragging(x, y):  # These parameters will be the mouse position
    if not is_pen_down:
        pass

    t.ondrag(None)
    t.setheading(t.towards(x, y))
    t.goto(x, y)
    t.ondrag(dragging)

def clearBoard():
    t.clear()

def up_down_pen():
    turtle.penup()

def setupBoard(dim):
    screen.setup(dim, dim, 0, 0)
    
    t.ondrag(dragging)
    screen.onkey(clearBoard, "c")
    screen.onkey(endProgram, "q")
    screen.onkey(up_down_pen, "p")

    screen.listen()

def endProgram():
    exit(0)

def main():  # This will run the program
    args = parser.parse_args()
    w_h = SIZE_DEFAULT if args.size < 0 else args.size
    setupBoard(w_h)

    
    turtle.mainloop()  # This will continue running main() 
    print("h")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Draw on board')
    parser.add_argument("-s", "--size", type = int, help = 'size of screen NxN', default = SIZE_DEFAULT)

    main()