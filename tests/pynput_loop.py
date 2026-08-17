import time
from pynput import keyboard
from threading import Event

global ui_loop

ui_interrupt = Event()


# the structure of this steering would be:
# main loop: updates things
# keyboard thread by pynput: processes key events (high prio)
# motor driver thread: moves the motor to where it should go
# global x, y, z: written in main loop, read by motor driver thread (soll-Pos.)
# global dx, dy, dz: set by keyboard thread, read by main thread
# motor_x etc: set by motor driver thread (ist-Position)

dx = 0
dy = 0
dz = 0
sf = 1

class KeyHandler:
    def __init__ (self):
        self.dx, self.dy, self.dz = 0, 0, 0
        self.sf = 1 
    def on_press(self,key):
        match key:
            case keyboard.Key.up:
                self.dy = +self.sf
            case keyboard.Key.down:
                self.dy = -self.sf
            case keyboard.Key.left:
                self.dx = -self.sf
            case keyboard.Key.right:
                self.dx = +self.sf
            case keyboard.Key.page_up:
                self.dz = +self.sf
            case keyboard.Key.page_down:
                self.dz = -self.sf
            case keyboard.Key.shift:
                self.sf = 10
    def on_release(self,key):
        if key == keyboard.Key.esc:
            ui_interrupt.set()
            return True
        match key:
            case keyboard.Key.up:
                self.dy = 0
            case keyboard.Key.down:
                self.dy = 0
            case keyboard.Key.left:
                self.dx = 0
            case keyboard.Key.right:
                self.dx = 0
            case keyboard.Key.page_up:
                self.dz = 0 
            case keyboard.Key.page_down:
                self.dz = 0
            case keyboard.Key.shift:
                self.sf = 1

kh = KeyHandler()
kbd = keyboard.Listener(on_press=kh.on_press,on_release=kh.on_release,suppress=True)
kbd.start()

ui_loop = True
x, y, z = 0, 0, 0
try:
    while ui_loop and not ui_interrupt.is_set():
        x += kh.dx
        y += kh.dy
        z += kh.dz
        print("pos",x,y,z)
        ui_interrupt.wait(.2) # this is interruptible by pressing Esc
except KeyboardInterrupt:
    pass
