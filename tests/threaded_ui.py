import threading
import time
import json

import sys
sys.path.append('../')
from vesicolos_utils import Keys, getkey

status = { 'LO': False }

def wait_for_lo (stop_event):
    global status
    t0 = time.time()
    while not time.time() - t0 > 10 and not stop_event.is_set():
        time.sleep(1)
    if not stop_event.is_set():
        print ("LO")
        status['LO'] = True


class UI:
    def __init__ (self, keymap, movement_map={}):
        self.keymap = keymap
        for k, m in SERVO_CMDS.items():
            self.keymap[k] = (UI.movement, m['axis'], m['dir'])
    def start (self, stop_event):
        print("UI")
        self.stop = False
        self.stop_event = stop_event
        while not stop_event.is_set():
            ch = getkey()
            if ch is None:
                continue
            if ch == Keys.ESC:
                self.stop = True
                print("bye, user exit")
                stop_event.set()
                continue
            if ch in self.keymap:
                func,*args = (*self.keymap[ch],)
                if func(self,*args) == False:
                    print("this was key",ch)
        print ("UI exit")
    def user_lo (self):
        print ("manual LO")
        self.stop_event.set()
    def recall_position (self, num):
        print ("GOTO",num)
    def movement (self, ax, direction):
        print ("MOVE",ax,direction)
    def test (self):
        print ("TEST")
        return False

keymap = {
                ord('x'): (UI.user_lo,),
                ord('1'): (UI.recall_position,1),
                Keys.UP: (UI.movement,'x',1),
                Keys.F1: (UI.test,)
        }
SERVO_CMDS = {
    Keys.LEFT:   { 'axis': 'Y', 'dir': +1 },
    Keys.RIGHT:  { 'axis': 'Y', 'dir': -1 },
    Keys.DOWN:   { 'axis': 'X', 'dir': -1 },
    Keys.UP:     { 'axis': 'X', 'dir': +1 },
    Keys.PGUP:   { 'axis': 'Z', 'dir': -1 },
    Keys.PGDOWN: { 'axis': 'Z', 'dir': +1 }
}




stop = False
cli = UI(keymap)
stop_event = threading.Event()
threading.Thread(target=wait_for_lo,args=[stop_event]).start()
threading.Thread(target=cli.start,args=[stop_event]).start()
while not status['LO']:
    if stop_event.is_set():
        print("exit")
        stop = True
        break
stop_event.set()
if cli.stop:
    print ("user stopped this")
else:
    print ("LO came")
