from __future__ import print_function

CAN_READ = False

try:
    import sys
    import select
    import tty
    import termios
    import atexit


    def key_pressed():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
        
    def read_key():
        return sys.stdin.read(1)
    
    def read_line():
        return sys.stdin.readline(1)
    
    def restore_settings():
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def clear_input():
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    
    atexit.register(restore_settings)
    old_settings = termios.tcgetattr(sys.stdin)
    
    tty.setcbreak(sys.stdin.fileno())
except:
    print("Can't deal with your keyboard!")

def print_key(ch):
    print("You pressed <<%s>>" % ch)

def canRead(v):
    global CAN_READ
    CAN_READ = v

"""
if __name__ == "__main__":

    print("Press any key")
    while True:
        c = read_key()
        print("You pressed <<%s>>" % c)
"""

      