from __future__ import print_function

try:
    import sys
    import select
    import tty
    import termios
    import atexit


    def key_pressed():
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
        
    def read_key():
        """lee un caracter de la entrada stdin"""
        return sys.stdin.read(1)
    
    def restore_settings():
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def clear_input():
        """ limpia la entrada stdin """
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    
    atexit.register(restore_settings)
    old_settings = termios.tcgetattr(sys.stdin)
    
    tty.setcbreak(sys.stdin.fileno()) # reedirige la salida del stdin
except:
    print("Can't deal with your keyboard!")

def print_key(ch):
    print("Pressed <<%s>>" % ch)
      