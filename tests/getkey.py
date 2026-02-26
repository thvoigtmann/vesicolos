import os, sys
import termios,tty
import select

from enum import IntEnum

class Keys(IntEnum):
    TAB = 9
    RETURN = 10
    ESC = 27
    SPACE = 32
    UP = 91*255 + 65
    DOWN = 91*255 + 66
    RIGHT = 91*255 + 67
    LEFT = 91*255 + 68
    F1 = 79*255 + 81
    F2 = 79*255 + 82
    F3 = 79*255 + 83
    F4 = 79*255 + 84
    PGUP = 91*255 + 53
    PGDOWN = 91*255 + 54
    BACKSPACE = 127

    @classmethod
    def has_value(cls, value: int) -> bool:
        return value in cls._value2member_map_

SERVO_CMDS = { int(k):m for k,m in {
    Keys.LEFT:   { 'axis': 'Y', 'dir': +1 },
    Keys.RIGHT:  { 'axis': 'Y', 'dir': -1 },
    Keys.DOWN:   { 'axis': 'X', 'dir': -1 },
    Keys.UP:     { 'axis': 'X', 'dir': +1 },
    Keys.PGUP:   { 'axis': 'Z', 'dir': -1 },
    Keys.PGDOWN: { 'axis': 'Z', 'dir': +1 }
}.items() }




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
                res = ord(b[1])*255 + ord(b[2])
                #res = key_mapping.get((ord(b[1]),k), chr(k))
            else:
                k = ord(b)
                res = k
                #res = key_mapping.get(k, chr(k))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return res

while True:
    ch = None
    while ch is None:
        print("press key")
        ch = getkey()
    print("CH",ch)
    if ch == Keys.ESC:
       break
    match ch:
        case ch if ch >= Keys.PGUP and ch <= Keys.LEFT:
            print(SERVO_CMDS)
            ax = SERVO_CMDS[ch]['axis']
            direction = SERVO_CMDS[ch]['dir']
            print("move",ax,direction)

    #ch = ch.upper()
    #if ch == 'ESC':
    #   break
