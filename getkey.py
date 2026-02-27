import sys, os
import termios, tty, select

from enum import IntEnum

# how to map keys to integers
# for ASCII, all is fine: a single integer in the range 0-255 is returned
# this excludes a set of notable keys on the keyboard:
# arrow keys, pgup/dn keys, home/end etc
# for these, we assume the keyboard to send ANSI sequences
# for documentation, see https://invisible-island.net/xterm/ctlseqs/ctlseqs.txt
# since the first byte is always Esc, we can omit that in our mapping
# common start sequences are CSI = Esc-[ and SS3 = Esc-O
class Keys(IntEnum):
    TAB = 9
    RETURN = 10
    ESC = 27
    SPACE = 32
    BACKSPACE = 127
    F1 = 79*255 + 80         # ANSI Esc-O P
    F2 = 79*255 + 81         # ANSI Esc-O Q
    F3 = 79*255 + 82         # ANSI Esc-O R
    F4 = 79*255 + 83         # ANSI Esc-O S
    F5 = 91*255*255 + 49*255 + 53          # ANSI Esc-[ 1 5 ~
    F6 = 91*255*255 + 49*255 + 55          # ANSI Esc-[ 1 7 ~
    F7 = 91*255*255 + 49*255 + 56          # ANSI Esc-[ 1 8 ~
    F8 = 91*255*255 + 49*255 + 57          # ANSI Esc-[ 1 9 ~
    F9 = 91*255*255 + 50*255 + 48          # ANSI Esc-[ 2 0 ~
    F10 = 91*255*255 + 50*255 + 49         # ANSI Esc-[ 2 1 ~
    F11 = 91*255*255 + 50*255 + 51         # ANSI Esc-[ 2 3 ~
    F12 = 91*255*255 + 50*255 + 52         # ANSI Esc-[ 2 4 ~
    INSERT = 91*255*255 + 50*255 + 126     # ANSI Esc-[ 2 ~
    DELETE = 91*255*255 + 51*255 + 126     # ANSI Esc-[ 3 ~
    PGUP = 91*255*255 + 53*255 + 126       # ANSI Esc-[ 5 ~
    PGDOWN = 91*255*255 + 54*255 + 126     # ANSI Esc-[ 6 ~
    UP = 91*255 + 65         # ANSI Esc-[ A
    DOWN = 91*255 + 66       # ANSI Esc-[ B
    RIGHT = 91*255 + 67      # ANSI Esc-[ C
    LEFT = 91*255 + 68       # ANSI Esc-[ D
    END = 91*255 + 70        # ANSI Esc-[ F
    HOME = 91*255 + 72       # ANSI Esc-[ H

    @classmethod
    def has_value(cls, value: int) -> bool:
        return value in cls._value2member_map_


def getkey(timeout=1):
    """Read key from keyboard using low-level os.read()
    Return a number describing the key.
    Adapted from https://stackoverflow.com/a/47197390/5802289

    Parameters:
    -----------
    timeout : float, optional
        Timeout in seconds passed to `select.select`.

    Returns:
    --------
    keycode : int
        For ASCII characters, returns the corresponding ASCII code.
        For some special keys (arrows, some function keys), returns
        a code built from the ANSI sequence sent by the keyboard.
        These correspond to the `Keys` enum.
    """
    global key_mapping
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        r, w, e = select.select([ sys.stdin ], [], [], timeout)
        if not sys.stdin in r:
            res = None
        else:
            b = os.read(sys.stdin.fileno(), 5).decode()
            if len(b) >= 3:
                l = min(len(b),4)
                # ignore b[0] which is always Esc
                res = 0
                for k in range(1,l):
                    res = 255*res + ord(b[k])
            else:
                k = ord(b)
                res = k
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return res


