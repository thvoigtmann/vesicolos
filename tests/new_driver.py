# TODO document environment variables: ST_DEVICE, RASPI_MODEL, DEBUG

import sys, os, tty, termios, select, psutil
import time
import threading, signal
import json
import logging

# local repositories
sys.path.append('python-st3215/src')
from python_st3215 import ST3215

## global settings

# hardware settings
# motor settings
ST_DEVICE = { '_default_': '/dev/ttyS0', # UART device (Raspberry 4)
              '5': '/dev/ttyAMA0' }      # UART device (Raspberry 5)

# servo configuration for the three axes
SERVO_AXIS_MAP = { 0: 'X', 9: 'Y', 1: 'Z' }
# TODO cleanup
SERVOS = {
  'X': { 'ID': 0, 'DEFAULT_SPEED': 2400, 'SPEED_INC': 200, 'current_speed': 0 },
  'Y': { 'ID': 9, 'DEFAULT_SPEED': 2400, 'SPEED_INC': 200, 'current_speed': 0 },
  'Z': { 'ID': 1, 'DEFAULT_SPEED':  200, 'SPEED_INC':  40, 'MAX_WRAP': 1, 'current_speed': 0 }
}
#SERVOS = {
#    'X': { 'ID': 0 },
#    'Y': { 'ID': 9 },
#    'Z': { 'ID': 1 }
#}
# mapping of keyboard controls to axis/direction for servo
SERVO_CMDS = {
    'LEFT':   { 'axis': 'Y', 'dir': +1 },
    'RIGHT':  { 'axis': 'Y', 'dir': -1 },
    'DOWN':   { 'axis': 'X', 'dir': -1 },
    'UP':     { 'axis': 'X', 'dir': +1 },
    'PGUP':   { 'axis': 'Z', 'dir': -1 },
    'PGDOWN': { 'axis': 'Z', 'dir': +1 }
}




## methods to deal with keyboard input

# these are the scan codes that we get from the ANSI terminal
# (or at least enough bytes after the esc byte for the special keys
# in order to recognize them reliably enough)
key_mapping = {
  9: 'tab', 10: 'return', 27: 'esc', 32: 'space',
  (91,65): 'up', (91,66): 'down', (91,67): 'right', (91,68): 'left',
  (79,80): 'f1', (79,81): 'f2', (79,82): 'f3', (79,83): 'f4',
  (91,53): 'pgup', (91,54): 'pgdown',
  127: 'backspace'
}

# method to read keyboard input
# adapted to make this non-blocking (i.e., return after specified timeout)
# this allows the main loop waiting for user input to recognize
# if a status variable changed even if the user doesn't do anything
def getkey(timeout=1):
    """Read key from keyboard using low-level os.read()
       Return string describing the key.
       Adapted from https://stackoverflow.com/a/47197390/5802289"""
    global key_mapping
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        r, w, e = select.select([ sys.stdin ], [], [], timeout)
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



log = logging.getLogger("VESICOLOS")
if os.environ.get("DEBUG"):
    log.setLevel(logging.DEBUG)
else:
    log.setLevel(logging.INFO)
_console_handler = logging.StreamHandler()
_console_handler.setFormatter(logging.Formatter("[%(levelname)s] %(name)s: %(message)s"))
log.addHandler(_console_handler)


class Motors:
    mode_names = { 0: "position", 1: "wheel", 2: "PWM", 3: "stepper" }
    def __init__ (self, device, axes_map={}):
        self.controller = ST3215(device)
        log.info("Scanning for servos.")
        old_level = self.controller.logger.level
        self.controller.logger.setLevel(logging.ERROR)
        self.axes = []
        self._servos = {}
        self.servo_ids = self.controller.list_servos()
        if not self.servo_ids:
            log.error("no servos found")
        else:
            self.controller.logger.setLevel(old_level)
            log.info("found %d servos" % len(self.servo_ids))
            for servo_id in self.servo_ids:
                servo = self.controller.wrap_servo(servo_id)
                if servo_id in axes_map:
                    axis = axes_map[servo_id]
                    infostr = axis + " axis "
                else:
                    axis = None
                    infostr = ''
                log.info(f"{infostr}SERVO ID {servo_id} pos {servo.sram.read_current_location()} mode {self.mode_names.get(servo.eeprom.read_operating_mode(),'UNKNOWN')}")
                log.debug(f"SERVO ID {servo_id} U={servo.sram.read_current_voltage() / 10:.1f}V, T={servo.sram.read_current_temperature()}°C")
                self._servos[servo_id] = servo
                if axis:
                    self._servos[axis] = servo
            found_all_axes = True
            for ax in sorted(list(set(axes_map.values()))):
                if ax in self._servos:
                    self.axes.append(ax)
                else:
                    found_all_axes = False
            if not found_all_axes:
                log.err("unexpected servo configuration found, continuing anyway")
    def __enter__ (self):
        return self
    def __exit__ (self, exc_type, exc_value, traceback):
        log.debug("stopping all motors")
        self.stop_all()
        log.debug("closing motor controller")
        self.controller.close()
    def stop_all (self):
        """Stop all servos"""
        self.controller.broadcast.sram.sync_write_running_speed(
                {servo.id: 0 for servo in self._servos.values()}
        )
        # update current_speed variables?


# the stuff that follows here would be ideally in the Servo API?
def ServoWheelMode (servo):
    servo.eeprom.write_operating_mode(1)
    #servo.sram.torque_enable()



## MAIN

# set UART device depending on raspberry model or environment variables
rpi_model = os.environ.get("RASPI_MODEL","_default_")
log.info(f"assuming Raspberry model {rpi_model}")
st_device = ST_DEVICE.get(rpi_model,ST_DEVICE['_default_'])
st_device = os.environ.get("ST_DEVICE",st_device)
log.info(f"using {st_device} as UART device")

# ideas for user interaction loop:
# make getKey return numerical values, should be faster to test against
# make a mapping of key -> function (lambda func), just call that,
# this will make the main loop much more readable
# also will make the help consistent with the functions
# (same style as ArgParser, maybe use that to allow cmdline arguments?)
# all the variables that would be used should be stored in a state dict
status = { 'LO': 0 }
with Motors(device=st_device, axes_map=SERVO_AXIS_MAP) as motors:

    motors.stop_all()
    for ax in motors.axes:
        ServoWheelMode(motors._servos[ax])

    ## PHASE 1: USER INTERACTION BEFORE LIFT OFF
    log.info("PHASE 1: INTERACTIVE MODE")
    while not status['LO']:
        ch = None
        ch = getkey()
        if ch is None:
            # motor_moving time out
            continue
        ch = ch.upper()
        log.debug(f"keypress {ch}")
        match ch:
            case 'ESC':
                stop = True
                print("GOOD-BYE")
                log.info("user exit (esc)")
                break
            case 'UP' | 'DOWN' | 'LEFT' | 'RIGHT' | 'PGUP' | 'PGDOWN':
                ax = SERVO_CMDS[ch]['axis']
                direction = SERVO_CMDS[ch]['dir']
                vel = SERVOS[ax]['current_speed'] \
                        + direction * SERVOS[ax]['SPEED_INC']
                print("setting speed",vel)
                motors._servos[ax].sram.write_running_speed(vel)
            case '0':
                motors.stop_all()

    # cleanup
    motors.stop_all()
    for ax in motors.axes:
        ServoWheelMode(motors._servos[ax])

log.info("EXIT")
