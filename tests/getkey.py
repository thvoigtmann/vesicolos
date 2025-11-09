import os, sys
import termios,tty
import select

key_mapping = {
  9: 'tab', 10: 'return', 27: 'esc', 32: 'space',
  (91,65): 'up', (91,66): 'down', (91,67): 'right', (91,68): 'left',
  (79,80): 'f1', (79,81): 'f2', (79,82): 'f3', (79,83): 'f4',
  (91,53): 'pgup', (91,54): 'pgdown',
  127: 'backspace'
}


def getkey():
    """Read key from keyboard using low-level os.read()
       Return string describing the key.
       Adapted from https://stackoverflow.com/a/47197390/5802289"""
    global key_mapping
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        r, w, e = select.select([ sys.stdin ], [], [], 1)
        if not sys.stdin in r:
            res = None
        else:
            b = os.read(sys.stdin.fileno(), 4).decode()
            if len(b) >= 3:
                k = ord(b[2])
                res = key_mapping.get((ord(b[1]),k), chr(k))
            else:
                k = ord(b)
                res = key_mapping.get(k, chr(k))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return res

while True:
    ch = None
    while ch is None:
        print("press key")
        ch = getkey()
    print("CH",ch)
    ch = ch.upper()
    if ch == 'ESC':
       break
